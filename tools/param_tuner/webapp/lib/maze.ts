import fs from "node:fs";
import path from "node:path";
import { load as loadYaml } from "js-yaml";
import { formatMazeText, mazeSizeOf, parseMazeText, type Cell } from "./maze-shared";

// webapp/ is the Next.js server cwd; tools/param_tuner/ is one level up.
const PARAM_TUNER_ROOT = path.join(process.cwd(), "..");
const MAZE_LOGS_DIR = path.join(PARAM_TUNER_ROOT, "maze_logs");
const MAZE_DATA_DIR = path.join(PARAM_TUNER_ROOT, "maze_data");
const PROFILE_DIR = path.join(PARAM_TUNER_ROOT, "profile");
const SYSTEM_YAML = path.join(PROFILE_DIR, "system.yaml");

// log     = maze_logs/*.maze   機体から受信した迷路と、ここで保存した迷路(保存先はここ。ユーザー指定)。
//           受信した迷路(日時の名前)は記録なので読み取り専用、それ以外は上書き保存できる
// profile = profile/*.yaml のうち中身が迷路(.maze と同じカンマ区切り)のもの。VSCode 拡張で
//           使っていた maze.yaml や higashi2024.yaml など(その場で上書き保存できる)
// contest = maze_data/*.yaml|*.maze  過去の迷路(yaml は大会迷路の形式でゴール付き、読み取り専用)
export type MazeGroup = "log" | "profile" | "contest";

// 機体から受信した迷路の名前(serial-manager の nowStamp。古いものは YYYYMMDD_HHMM_SS)
export function isReceivedMazeName(name: string): boolean {
  return /^\d{8}_\d{4}_?\d{2}\.maze$/.test(name);
}

export interface MazeFileInfo {
  id: string; // `${group}/${name}`
  group: MazeGroup;
  name: string;
  mtimeMs: number;
}

export interface MazeContent {
  id: string;
  size: number;
  walls: number[];
  goals: Cell[] | null; // ファイル自身が持つゴール(大会迷路のみ)
  editable: boolean;
}

const NAME_RE = /^[\w.-]+$/;

function resolveMazePath(id: string): { group: MazeGroup; name: string; file: string } {
  const slash = id.indexOf("/");
  const group = id.slice(0, slash) as MazeGroup;
  const name = id.slice(slash + 1);
  if (slash < 0 || !NAME_RE.test(name)) throw new Error("不正なファイル名です");
  if (group === "log" && name.endsWith(".maze")) return { group, name, file: path.join(MAZE_LOGS_DIR, name) };
  if (group === "contest" && /\.(yaml|maze)$/.test(name)) return { group, name, file: path.join(MAZE_DATA_DIR, name) };
  if (group === "profile" && name.endsWith(".yaml")) return { group, name, file: path.join(PROFILE_DIR, name) };
  throw new Error("不明なファイルです");
}

function listDir(dir: string, group: MazeGroup, re: RegExp): MazeFileInfo[] {
  if (!fs.existsSync(dir)) return [];
  return fs
    .readdirSync(dir)
    .filter((name) => re.test(name))
    .map((name) => ({ id: `${group}/${name}`, group, name, mtimeMs: fs.statSync(path.join(dir, name)).mtimeMs }));
}

// 中身がカンマ区切りの迷路として読めるか(profile/ のパラメータ yaml と見分ける)。
function isMazeText(file: string): boolean {
  try {
    parseMazeText(fs.readFileSync(file, "utf-8"));
    return true;
  } catch {
    return false;
  }
}

export function listMazeFiles(): MazeFileInfo[] {
  const logs = listDir(MAZE_LOGS_DIR, "log", /\.maze$/).sort((a, b) => b.mtimeMs - a.mtimeMs);
  const profiles = listDir(PROFILE_DIR, "profile", /\.yaml$/)
    .filter((f) => isMazeText(path.join(PROFILE_DIR, f.name)))
    .sort((a, b) => a.name.localeCompare(b.name));
  const contests = listDir(MAZE_DATA_DIR, "contest", /\.(yaml|maze)$/).sort((a, b) => a.name.localeCompare(b.name));
  return [...logs, ...profiles, ...contests];
}

interface ContestYaml {
  maze_data?: { maze_size?: number; wall?: number[]; goal?: Cell[] };
}

export function readMaze(id: string): MazeContent {
  const { group, file } = resolveMazePath(id);
  if (!fs.existsSync(file)) throw new Error("ファイルが見つかりません");
  const text = fs.readFileSync(file, "utf-8");
  if (group === "contest" && file.endsWith(".yaml")) {
    const data = (loadYaml(text) as ContestYaml)?.maze_data;
    if (!Array.isArray(data?.wall)) throw new Error("maze_data.wall がありません");
    const walls = data.wall.map((w) => Number(w) & 0x0f);
    const size = mazeSizeOf(walls.length);
    if (data.maze_size !== undefined && data.maze_size !== size) {
      throw new Error(`maze_size=${data.maze_size} と壁の数 ${walls.length} が合いません`);
    }
    return { id, size, walls, goals: Array.isArray(data.goal) ? data.goal : null, editable: false };
  }
  const walls = parseMazeText(text);
  // 過去の迷路 (maze_data) の .maze と受信した迷路は読み取り専用(編集したら別名保存で maze_logs/ へ)
  const editable = group === "profile" || (group === "log" && !isReceivedMazeName(path.basename(file)));
  return { id, size: mazeSizeOf(walls.length), walls, goals: null, editable };
}

export function checkWalls(walls: unknown): number[] {
  if (!Array.isArray(walls) || !walls.every((w) => Number.isInteger(w) && w >= 0 && w <= 0x0f)) {
    throw new Error("壁データが不正です");
  }
  mazeSizeOf(walls.length);
  return walls as number[];
}

export function writeMaze(id: string, walls: unknown): void {
  const { group, name, file } = resolveMazePath(id);
  const editable = group === "profile" || (group === "log" && !isReceivedMazeName(name));
  if (!editable) throw new Error("このファイルは上書きできません。別名で保存してください");
  // profile/ はパラメータの yaml と同じ場所なので、今の中身が迷路のファイルだけ上書きする
  if (group === "profile" && !isMazeText(file)) throw new Error("迷路のファイルではないので上書きしません");
  const w = checkWalls(walls);
  fs.writeFileSync(file, formatMazeText(w, mazeSizeOf(w.length)), "utf-8");
}

// maze_logs/<name>.maze へ新規保存。既存ファイルは上書きしない。
export function saveMazeAs(name: string, walls: unknown): string {
  const base = name.trim().replace(/\.maze$/, "");
  if (!/^[\w-]+$/.test(base)) throw new Error("ファイル名は英数字・_・- だけにしてください");
  if (isReceivedMazeName(`${base}.maze`)) throw new Error("受信した迷路と同じ日時の名前は使えません(読み取り専用になるため)");
  const file = path.join(MAZE_LOGS_DIR, `${base}.maze`);
  if (fs.existsSync(file)) throw new Error(`${base}.maze は既にあります。名前を変えてください`);
  const w = checkWalls(walls);
  fs.mkdirSync(MAZE_LOGS_DIR, { recursive: true });
  fs.writeFileSync(file, formatMazeText(w, mazeSizeOf(w.length)), "utf-8");
  return `log/${base}.maze`;
}

// system.yaml は読むだけ(書き換えは test-templates.ts の行置換で行う決まり)。
export function readSystemMaze(): { goals: Cell[] | null; mazeSize: number | null } {
  try {
    const sys = loadYaml(fs.readFileSync(SYSTEM_YAML, "utf-8")) as { goals?: Cell[]; maze_size?: number };
    return {
      goals: Array.isArray(sys?.goals) ? sys.goals : null,
      mazeSize: typeof sys?.maze_size === "number" ? sys.maze_size : null,
    };
  } catch {
    return { goals: null, mazeSize: null };
  }
}
