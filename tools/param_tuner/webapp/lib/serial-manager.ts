import { EventEmitter } from "node:events";
import fs from "node:fs";
import path from "node:path";
import { load as loadYaml } from "js-yaml";
import { SerialPort } from "serialport";
import { ReadlineParser } from "@serialport/parser-readline";
import { ByteLengthParser } from "@serialport/parser-byte-length";

// This must stay a single, persistent connection: console.sh (rx_term.js)
// and update_param.sh (tx_term.js -> send_file.py) used to each open their
// own exclusive lock on the tty, so RX monitoring and TX param pushes could
// never run at the same time. Multiplexing both over one SerialManager is
// the whole point of this app.
const BAUD_RATE = 3_000_000;
// Matches rx_term.js's waitForPico() poll cadence.
const SEARCH_INTERVAL_MS = 200;
const ACK_TIMEOUT_MS = 10_000;

// webapp/ is the Next.js server cwd; tools/param_tuner/ is one level up.
const PARAM_TUNER_ROOT = path.join(process.cwd(), "..");
const LOGS_DIR = path.join(PARAM_TUNER_ROOT, "logs");
const MAZE_LOGS_DIR = path.join(PARAM_TUNER_ROOT, "maze_logs");
const PROFILE_DIR = path.join(PARAM_TUNER_ROOT, "profile");

// Ported from tx_term.js: these three are always read from profile/ directly
// (not profile/<mode>/), regardless of which mode is selected.
const BASE_FILES = ["system.yaml", "hardware.yaml", "am32.yaml"];

type FieldType = "float" | "int" | "short";
interface FieldDef {
  name: string;
  type: FieldType;
  size: number;
}

interface DumpState {
  dumpToCsvReady: boolean;
  dumpToCsvText: boolean;
  dumpToMap: boolean;
  dataStruct: FieldDef[];
  fileName: string;
  record: string;
  totalBytes: number;
}

interface PendingAck {
  resolve: (line: string) => void;
  reject: (err: Error) => void;
  timer: NodeJS.Timeout;
}

export interface PortInfo {
  path: string;
  manufacturer?: string;
  serialNumber?: string;
}

export type ConnectionStatus = "disconnected" | "connecting" | "connected";

export interface StatusInfo {
  status: ConnectionStatus;
  path: string | null;
  autoConnect: boolean;
}

// Fixed to the Pico's ttyACM* CDC device, matching rx_term.js's own
// serialNumber-gated match (rules out non-device ttyACM entries).
function isPicoPort(p: { path: string; serialNumber?: string }): boolean {
  return Boolean(p.serialNumber) && /ttyACM/.test(p.path);
}

export interface ProfileList {
  base: string[];
  mode: string[];
}

export type SendScope = "base" | "mode";

function nowStamp(ext: string): string {
  const dt = new Date();
  const p = (n: number) => String(n).padStart(2, "0");
  return `${dt.getFullYear()}${p(dt.getMonth() + 1)}${p(dt.getDate())}_${p(dt.getHours())}${p(dt.getMinutes())}${p(dt.getSeconds())}.${ext}`;
}

// Matches ANSI CSI sequences such as the ESC[2J / ESC[0;0H clear-screen +
// cursor-home pair some test modes (e.g. dump1() in main_task_test_misc.cpp)
// print before repainting a live status screen.
const ANSI_CSI_RE = /\x1b\[[0-9;]*[A-Za-z]/g;
const ANSI_CLEAR_RE = /\x1b\[2J/;

// Mirrors send_file.py's readline_skip_sensor(): while waiting for a command
// ack, ignore telemetry/debug lines instead of treating them as the response.
function isTelemetryOrDebugLine(line: string): boolean {
  return (
    line.includes("ADC0:") ||
    line.includes("Gx:") ||
    line.includes("Enc0:") ||
    line.startsWith("[")
  );
}

// Ported from rx_term.js/tx_term.js: the firmware's maze dump/upload stores
// only the lower triangle, so the upper triangle has to be mirrored from it.
function swapMazeTriangle(list: number[]): number[] {
  const size = list.length > 300 ? 32 : 16;
  const out = list.slice();
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      if (x >= y) continue;
      const i1 = y * size + x;
      const i2 = x * size + y;
      const tmp = out[i1];
      out[i1] = out[i2];
      out[i2] = tmp;
    }
  }
  return out;
}

function freshDumpState(): DumpState {
  return {
    dumpToCsvReady: false,
    dumpToCsvText: false,
    dumpToMap: false,
    dataStruct: [],
    fileName: nowStamp("csv"),
    record: "",
    totalBytes: 0,
  };
}

class SerialManager extends EventEmitter {
  private port: SerialPort | null = null;
  private parser: ReadlineParser | ByteLengthParser | null = null;
  private binaryMode = false;
  private dump: DumpState = freshDumpState();
  private pendingAck: PendingAck | null = null;
  private connectedPath: string | null = null;
  private status: ConnectionStatus = "disconnected";
  private autoConnectEnabled = true;
  private searchTimer: NodeJS.Timeout | null = null;

  constructor() {
    super();
    this.startSearchLoop();
  }

  getStatus(): StatusInfo {
    return { status: this.status, path: this.connectedPath, autoConnect: this.autoConnectEnabled };
  }

  async listPorts(): Promise<PortInfo[]> {
    const ports = await SerialPort.list();
    return ports
      .filter(isPicoPort)
      .map((p) => ({ path: p.path, manufacturer: p.manufacturer, serialNumber: p.serialNumber }));
  }

  // Re-arms auto-connect after disconnect(); the search loop (already
  // running) picks up the next matching ttyACM* device on its own.
  enableAutoConnect(): void {
    this.autoConnectEnabled = true;
    this.emit("status", this.getStatus());
    void this.trySearch();
  }

  disconnect(): void {
    this.autoConnectEnabled = false;
    this.rejectPendingAck(new Error("disconnected"));
    this.port?.close();
    this.port = null;
    this.connectedPath = null;
    this.setStatus("disconnected");
  }

  // Ported from rx_term.js's waitForPico(): poll for a matching device and
  // connect the moment one shows up, instead of requiring the user to pick a
  // port. Runs continuously so it also covers reconnecting after unplug.
  private startSearchLoop() {
    if (this.searchTimer) return;
    this.searchTimer = setInterval(() => void this.trySearch(), SEARCH_INTERVAL_MS);
  }

  private async trySearch() {
    if (!this.autoConnectEnabled || this.status !== "disconnected") return;
    let ports: PortInfo[];
    try {
      ports = await this.listPorts();
    } catch {
      return;
    }
    const found = ports[0];
    if (found && this.autoConnectEnabled && this.status === "disconnected") {
      this.openPort(found.path);
    }
  }

  listModes(): string[] {
    if (!fs.existsSync(PROFILE_DIR)) return [];
    return fs
      .readdirSync(PROFILE_DIR, { withFileTypes: true })
      .filter((d) => d.isDirectory())
      .map((d) => d.name)
      .sort();
  }

  listProfiles(mode: string): ProfileList {
    const modeDir = path.join(PROFILE_DIR, mode);
    const files = fs.existsSync(modeDir) ? fs.readdirSync(modeDir) : [];
    const modeFiles = files.filter((f) => /\.(yaml|maze)$/.test(f)).sort();
    return { base: BASE_FILES, mode: modeFiles };
  }

  // Editing is limited to *.yaml (the *.maze files are plain int lists, not
  // YAML). The filename charset excludes "/" so this can't escape PROFILE_DIR.
  private resolveYamlPath(mode: string, scope: SendScope, file: string): string {
    if (!/^[\w.-]+\.yaml$/.test(file)) {
      throw new Error("不正なファイル名です");
    }
    if (scope === "base") {
      if (!BASE_FILES.includes(file)) throw new Error("不明なファイルです");
      return path.join(PROFILE_DIR, file);
    }
    return path.join(PROFILE_DIR, mode, file);
  }

  readProfileFile(mode: string, scope: SendScope, file: string): string {
    return fs.readFileSync(this.resolveYamlPath(mode, scope, file), "utf-8");
  }

  writeProfileFile(mode: string, scope: SendScope, file: string, content: string): void {
    const filePath = this.resolveYamlPath(mode, scope, file);
    try {
      loadYaml(content);
    } catch (err) {
      throw new Error(`YAML構文エラー: ${(err as Error).message}`);
    }
    fs.writeFileSync(filePath, content, "utf-8");
    this.emit("log", `[edit] saved: ${file}`);
  }

  // Ported from tx_term.js's per-index send branch.
  async sendFile(mode: string, scope: SendScope, file: string): Promise<void> {
    if (scope === "base") {
      const content = fs.readFileSync(path.join(PROFILE_DIR, file), "utf-8");
      const remoteName = file.replace("yaml", "txt");
      await this.writeAndWaitAck(remoteName, JSON.stringify(loadYaml(content)));
      this.emit("log", `[send] ${file} -> ${remoteName}: OK`);
      return;
    }

    const filePath = path.join(PROFILE_DIR, mode, file);
    if (/\.maze$/.test(file)) {
      const text = fs.readFileSync(filePath, "utf-8");
      const list = text.split(",").map((e) => parseInt(e.trim(), 10) | 0xf0);
      const swapped = swapMazeTriangle(list);
      await this.writeAndWaitAck("maze.txt", swapped.join(","));
      this.emit("log", `[send] ${file} -> maze.txt: OK`);
      return;
    }

    const remoteName = file.replace("yaml", mode);
    const content = fs.readFileSync(filePath, "utf-8");
    await this.writeAndWaitAck(remoteName, JSON.stringify(loadYaml(content)));
    this.emit("log", `[send] ${file} -> ${remoteName}: OK`);
  }

  // Ported from tx_term.js's "all" branch: mode dir's *.yaml (not *.maze),
  // then the three base files, in that order.
  async sendAll(mode: string): Promise<void> {
    const modeDir = path.join(PROFILE_DIR, mode);
    const files = fs.existsSync(modeDir) ? fs.readdirSync(modeDir) : [];
    for (const file of files) {
      if (/\.yaml$/.test(file)) {
        await this.sendFile(mode, "mode", file);
      }
    }
    for (const file of BASE_FILES) {
      await this.sendFile(mode, "base", file);
    }
  }

  private setStatus(status: ConnectionStatus) {
    this.status = status;
    this.emit("status", this.getStatus());
  }

  private openPort(portPath: string) {
    this.setStatus("connecting");
    const port = new SerialPort(
      { path: portPath, baudRate: BAUD_RATE, highWaterMark: 256 * 1024 },
      (err) => {
        if (err) {
          this.emit("log", `[serial] open failed: ${err.message}`);
          // Leave status as "disconnected" here (set below) so the ongoing
          // search loop retries on its own next tick.
          this.setStatus("disconnected");
        }
      }
    );
    this.port = port;

    port.on("open", () => {
      this.connectedPath = portPath;
      this.setStatus("connected");
      this.emit("log", `[serial] connected: ${portPath}`);
      this.dump = freshDumpState();
      this.switchLineMode();
    });

    // The search loop is always running, so simply going back to
    // "disconnected" is enough for it to pick up a reconnect.
    port.on("close", () => {
      this.connectedPath = null;
      this.setStatus("disconnected");
      this.emit("log", "[serial] disconnected");
      this.rejectPendingAck(new Error("port closed"));
    });

    port.on("error", (err) => {
      this.emit("log", `[serial] error: ${err.message}`);
    });
  }

  private rejectPendingAck(err: Error) {
    if (this.pendingAck) {
      clearTimeout(this.pendingAck.timer);
      this.pendingAck.reject(err);
      this.pendingAck = null;
    }
  }

  // ===== RX: ported from rx_term.js's switchLineMode/switchToBinaryMode =====

  private switchLineMode() {
    if (!this.port) return;
    if (this.parser) this.port.unpipe(this.parser as NodeJS.WritableStream);
    this.binaryMode = false;
    const parser = this.port.pipe(new ReadlineParser({ delimiter: "\r\n" }));
    this.parser = parser;
    parser.on("data", (data: string) => this.handleLine(data));
  }

  private handleLine(raw: string) {
    // The ESC[2J/ESC[0;0H pair (and similar CSI codes) are printf'd with no
    // trailing \r\n, so they arrive as a prefix glued onto whatever line
    // comes right after. Strip them for display and tell the client to
    // clear its scrollback so live-redraw screens (e.g. dump1()) render as
    // a refreshing dashboard instead of an ever-growing wall of raw codes.
    if (ANSI_CLEAR_RE.test(raw)) {
      this.emit("clear");
    }
    const data = raw.replace(ANSI_CSI_RE, "");
    if (data.length > 0) {
      this.emit("log", data);
    }

    // TX ack consumption: resolve a pending sendFile() write, independent of
    // (and before) the RX dump state machine below, which never produces a
    // line matching send_file.py's OK/ERR ack shape.
    if (this.pendingAck && data.length > 0 && !isTelemetryOrDebugLine(data)) {
      const ack = this.pendingAck;
      this.pendingAck = null;
      clearTimeout(ack.timer);
      ack.resolve(data);
    }

    const dump = this.dump;

    if (dump.dumpToCsvText) {
      if (/^end___/.test(data)) {
        dump.dumpToCsvText = false;
        this.writeLogFile(dump.fileName, dump.record);
        this.emit("saved", { type: "csv", file: dump.fileName });
      } else {
        dump.record += `${data}\n`;
      }
      return;
    }

    if (dump.dumpToCsvReady) {
      const parts = data.split(":");
      if (parts.length === 3) {
        dump.dataStruct.push({
          name: parts[0],
          type: parts[1] as FieldType,
          size: parseInt(parts[2], 10),
        });
      }
    } else if (dump.dumpToMap) {
      if (/^end___/.test(data)) {
        dump.dumpToMap = false;
        const list = dump.record
          .split(",")
          .map((e) => e.trim())
          .filter((e) => e.length > 0)
          .map((e) => parseInt(e, 10));
        const swapped = swapMazeTriangle(list);
        fs.mkdirSync(MAZE_LOGS_DIR, { recursive: true });
        fs.writeFileSync(path.join(MAZE_LOGS_DIR, dump.fileName), swapped.join(","), {
          flag: "w+",
        });
        this.emit("saved", { type: "maze", file: dump.fileName });
      } else {
        dump.record += `${data}\n`;
      }
    }

    if (/^csv___/.test(data)) {
      dump.dumpToCsvText = true;
      dump.fileName = nowStamp("csv");
      dump.record = "";
    }

    if (/^ready___/.test(data)) {
      dump.dumpToCsvReady = true;
      dump.fileName = nowStamp("csv");
      dump.record = "";
      dump.dataStruct = [];
    }

    if (/^start___/.test(data)) {
      dump.fileName = nowStamp("csv");
      dump.record = "";
      dump.totalBytes = parseInt(data.split(":")[1] ?? "0", 10);
      this.switchToBinaryMode();
    }

    if (/^map___/.test(data)) {
      dump.dumpToMap = true;
      dump.fileName = nowStamp("maze");
      dump.record = "";
    }
  }

  private switchToBinaryMode() {
    if (!this.port) return;
    const dump = this.dump;
    const totalBytes = dump.totalBytes;

    // A garbled start___ line (serial noise) can yield a non-numeric or
    // negative byte count. Without totalBytes we don't know how many bytes
    // to frame, so there's nothing safe to do but drop the dump and stay in
    // line mode - better than misframing the stream.
    if (!Number.isFinite(totalBytes) || totalBytes <= 0) {
      this.emit("log", `[LoggingTask] invalid start___ byte count (${dump.totalBytes}); dropping dump`);
      this.dump = freshDumpState();
      return;
    }

    const fieldCount = dump.dataStruct.length;
    const recordByteSize = dump.dataStruct.reduce((s, d) => s + d.size, 0);
    const recordNum = recordByteSize > 0 ? Math.floor(totalBytes / recordByteSize) : 0;

    this.binaryMode = true;
    if (this.parser) this.port.unpipe(this.parser as NodeJS.WritableStream);
    const parser = this.port.pipe(
      new ByteLengthParser({ length: totalBytes, highWaterMark: 256 * 1024 })
    );
    this.parser = parser;

    const header = dump.dataStruct.map((d) => d.name).join(",");
    const fieldReaders = dump.dataStruct.map((d) => {
      if (d.type === "float") return (buf: Buffer, off: number) => buf.readFloatLE(off);
      if (d.type === "int") return (buf: Buffer, off: number) => buf.readInt32LE(off);
      if (d.type === "short") return (buf: Buffer, off: number) => buf.readInt16LE(off);
      throw new Error(`Unsupported type: ${d.type}`);
    });
    const fieldSizes = dump.dataStruct.map((d) => d.size);

    parser.once("data", (binaryData: Buffer) => {
      // dataStruct comes from ready___-preceded name:type:size lines; if
      // that header was missing or corrupted (e.g. dropped by serial noise,
      // or start___ arrived without a ready___ first), fieldCount/recordByteSize
      // are 0 and there's no valid layout to parse - discard the payload
      // instead of building a bogus (or, for NaN/negative recordNum, crashing)
      // array.
      if (fieldCount === 0 || recordByteSize <= 0) {
        this.emit(
          "log",
          `[LoggingTask] dump header missing/corrupt (fields=${fieldCount}, recordByteSize=${recordByteSize}); discarding ${binaryData.length} bytes`
        );
      } else {
        const rows = new Array<string>(recordNum + 1);
        rows[0] = header;
        const record = new Array<number>(fieldCount);
        for (let j = 0; j < recordNum; j++) {
          let offset = j * recordByteSize;
          for (let i = 0; i < fieldCount; i++) {
            record[i] = fieldReaders[i](binaryData, offset);
            offset += fieldSizes[i];
          }
          rows[j + 1] = record.join(",");
        }
        const content = rows.join("\n") + "\n";
        this.writeLogFile(dump.fileName, content);
        this.emit("log", `[LoggingTask] dump done: ${recordNum} records -> ${dump.fileName}`);
        this.emit("saved", { type: "csv", file: dump.fileName });
      }
      this.dump = freshDumpState();
      this.switchLineMode();
    });
  }

  private writeLogFile(fileName: string, content: string) {
    fs.mkdirSync(LOGS_DIR, { recursive: true });
    const filePath = path.join(LOGS_DIR, fileName);
    fs.writeFileSync(filePath, content, { flag: "w+" });
    fs.copyFileSync(filePath, path.join(LOGS_DIR, "latest.csv"));
  }

  // ===== TX: ported from send_file.py's cmd_write() request/response =====

  private async writeAndWaitAck(remoteName: string, content: string): Promise<void> {
    if (!this.port || this.status !== "connected") {
      throw new Error("シリアル未接続です");
    }
    if (this.binaryMode || this.dump.dumpToMap || this.dump.dumpToCsvText) {
      throw new Error("ログ/迷路ダンプ受信中のため送信できません。しばらく待って再試行してください");
    }
    if (this.pendingAck) {
      throw new Error("別の送信が進行中です");
    }

    const payload = `${remoteName}@${content}\n`;
    return new Promise<void>((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pendingAck = null;
        reject(new Error(`応答タイムアウト (${remoteName})`));
      }, ACK_TIMEOUT_MS);

      this.pendingAck = {
        resolve: (line) => {
          if (line === "OK") resolve();
          else reject(new Error(`デバイス応答: ${line}`));
        },
        reject,
        timer,
      };

      this.port!.write(payload, (err) => {
        if (err) {
          clearTimeout(timer);
          this.pendingAck = null;
          reject(err);
        }
      });
    });
  }
}

declare global {
  var __serialManager: SerialManager | undefined;
}

// Survive Next.js dev-mode module reloads (same pattern as the usual Prisma
// client singleton) so we never hold two SerialManagers / two port locks.
export const serialManager = globalThis.__serialManager ?? new SerialManager();
if (process.env.NODE_ENV !== "production") {
  globalThis.__serialManager = serialManager;
}
