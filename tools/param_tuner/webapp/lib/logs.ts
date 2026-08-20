import { spawn } from "node:child_process";
import fs from "node:fs";
import path from "node:path";

// webapp/ is the Next.js server cwd; tools/param_tuner/ is one level up.
const PARAM_TUNER_ROOT = path.join(process.cwd(), "..");
const LOGS_DIR = path.join(PARAM_TUNER_ROOT, "logs");
const PROFILE_XML_PATH = path.join(PARAM_TUNER_ROOT, "profile.xml");

export interface LogFileInfo {
  name: string;
  mtimeMs: number;
  size: number;
}

export function listLogFiles(): LogFileInfo[] {
  if (!fs.existsSync(LOGS_DIR)) return [];
  return fs
    .readdirSync(LOGS_DIR)
    .filter((f) => f.endsWith(".csv"))
    .map((name) => {
      const stat = fs.statSync(path.join(LOGS_DIR, name));
      return { name, mtimeMs: stat.mtimeMs, size: stat.size };
    })
    .sort((a, b) => b.mtimeMs - a.mtimeMs);
}

// Filename charset excludes "/" so this can't escape LOGS_DIR.
function resolveLogPath(name: string): string {
  if (!/^[\w.-]+\.csv$/.test(name)) throw new Error("不正なファイル名です");
  const filePath = path.join(LOGS_DIR, name);
  if (!fs.existsSync(filePath)) throw new Error("ファイルが見つかりません");
  return filePath;
}

export function readLogFile(name: string): string {
  return fs.readFileSync(resolveLogPath(name), "utf-8");
}

function shellQuote(s: string): string {
  return `'${s.replace(/'/g, `'\\''`)}'`;
}

// Ported from plot_gui.py's run_plotjuggler(): the deb-packaged (apt)
// PlotJuggler isn't strict-confined like the snap build, so killPlotJuggler
// (pkill) can actually signal it afterwards. Runs through a login shell to
// pick up the ROS 2 environment, since `ros2` isn't on PATH otherwise.
export function openInPlotJuggler(name: string): void {
  const filePath = resolveLogPath(name);
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

export function openLogsFolder(): void {
  fs.mkdirSync(LOGS_DIR, { recursive: true });
  const child = spawn("xdg-open", [LOGS_DIR], { detached: true, stdio: "ignore" });
  child.unref();
}
