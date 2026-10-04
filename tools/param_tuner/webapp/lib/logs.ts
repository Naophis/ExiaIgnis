import { spawn } from "node:child_process";
import fs from "node:fs";
import path from "node:path";
import { getLogMachines, logsDir, readRegistry } from "./machines";
import { LOGS_DIR, TOOL_ROOT } from "./paths";

const PROFILE_XML_PATH = path.join(TOOL_ROOT, "profile.xml");

// ログ(csv)の置き場所(2026-10-05〜):
//   machines/<機体>/logs/  その機体の基板から受信したログ
//   logs/                  共通。機体を分ける前のログと、未登録の基板から受信したログ
// 迷路(maze_logs)は全機体で共通のまま。

export interface LogFileInfo {
  name: string;
  mtimeMs: number;
  size: number;
  machine?: string; // このログを出した機体(機体のフォルダにあるもの)
  common?: boolean; // 共通の logs/ にあるもの(どの機体のものか分からない)
}

function listDir(dir: string): { name: string; mtimeMs: number; size: number }[] {
  if (!fs.existsSync(dir)) return [];
  return fs
    .readdirSync(dir)
    .filter((f) => f.endsWith(".csv"))
    .map((name) => {
      const stat = fs.statSync(path.join(dir, name));
      return { name, mtimeMs: stat.mtimeMs, size: stat.size };
    });
}

// machine = 表示中の機体。その機体のログと、共通のログを返す(新しい順)。
// null(詳細ログ解析ページを直接開いたときなど)なら、全機体のログ + 共通。
// 同じ名前(latest.csv)が両方にあるときは機体のものを出す。
export function listLogFiles(machine: string | null): LogFileInfo[] {
  const out: LogFileInfo[] = [];
  const seen = new Set<string>();
  const ids = machine ? [machine] : readRegistry().machines.map((m) => m.id);
  for (const id of ids) {
    for (const f of listDir(logsDir(id))) {
      if (seen.has(f.name)) continue;
      seen.add(f.name);
      out.push({ ...f, machine: id });
    }
  }
  // 共通の logs/。保存先を分ける前の 1 日だけ付けていた印があるログは、その機体のものとして扱う
  const tags = getLogMachines();
  for (const f of listDir(LOGS_DIR)) {
    if (seen.has(f.name)) continue;
    const tag = tags[f.name];
    if (tag && machine && tag !== machine) continue; // 別の機体のログ
    seen.add(f.name);
    out.push(tag ? { ...f, machine: tag } : { ...f, common: true });
  }
  return out.sort((a, b) => b.mtimeMs - a.mtimeMs);
}

// 名前からログの場所を決める: 指定の機体 → 共通 → ほかの機体、の順に探す
// (名前は受信した日時なので、機体をまたいでも重ならない。latest.csv だけは機体ごとにある)。
// Filename charset excludes "/" so this can't escape the log directories.
export function resolveLogPath(name: string, machine: string | null): string {
  if (!/^[\w.-]+\.csv$/.test(name)) throw new Error("不正なファイル名です");
  const dirs: string[] = [];
  if (machine) dirs.push(logsDir(machine));
  dirs.push(LOGS_DIR);
  for (const m of readRegistry().machines) if (m.id !== machine) dirs.push(logsDir(m.id));
  for (const dir of dirs) {
    const filePath = path.join(dir, name);
    if (fs.existsSync(filePath)) return filePath;
  }
  throw new Error("ファイルが見つかりません");
}

export function readLogFile(name: string, machine: string | null): string {
  return fs.readFileSync(resolveLogPath(name, machine), "utf-8");
}

function shellQuote(s: string): string {
  return `'${s.replace(/'/g, `'\\''`)}'`;
}

// Ported from plot_gui.py's run_plotjuggler(): the deb-packaged (apt)
// PlotJuggler isn't strict-confined like the snap build, so killPlotJuggler
// (pkill) can actually signal it afterwards. Runs through a login shell to
// pick up the ROS 2 environment, since `ros2` isn't on PATH otherwise.
export function openInPlotJuggler(name: string, machine: string | null): void {
  const filePath = resolveLogPath(name, machine);
  const rosCmd = `ros2 run plotjuggler plotjuggler -d ${shellQuote(filePath)} -l ${shellQuote(PROFILE_XML_PATH)}`;
  const child = spawn("bash", ["-lc", `source /opt/ros/jazzy/setup.bash && ${rosCmd}`], {
    detached: true,
    stdio: "ignore",
  });
  child.unref();
}

export function killPlotJuggler(): void {
  spawn("pkill", ["-f", "plotjuggler"], { stdio: "ignore" });
}

// その機体のログのフォルダ(機体の指定が無ければ共通)をファイルマネージャで開く
export function openLogsFolder(machine: string | null): string {
  const dir = logsDir(machine);
  fs.mkdirSync(dir, { recursive: true });
  const child = spawn("xdg-open", [dir], { detached: true, stdio: "ignore" });
  child.unref();
  return dir;
}
