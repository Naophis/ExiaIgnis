import fs from "node:fs";
import path from "node:path";
import { load as loadYaml } from "js-yaml";
import { formatMazeText, mazeSizeOf, parseMazeText, type Cell } from "./maze-shared";

// webapp/ is the Next.js server cwd; tools/param_tuner/ is one level up.
const PARAM_TUNER_ROOT = path.join(process.cwd(), "..");
const MAZE_LOGS_DIR = path.join(PARAM_TUNER_ROOT, "maze_logs");
const MAZE_DATA_DIR = path.join(PARAM_TUNER_ROOT, "maze_data");
const PROFILE_DIR = path.join(PARAM_TUNER_ROOT, "profile");
// 編集用の .maze の置き場所。プロファイルパネルの一覧に出て、そこからも送れる
// (「全て送信」には含まれない)。
const EDIT_DIR = path.join(PROFILE_DIR, "hf");
// VSCode 拡張で使っていた迷路(中身は .maze と同じカンマ区切り)。
const LEGACY_EDIT_FILE = "maze.yaml";
const SYSTEM_YAML = path.join(PROFILE_DIR, "system.yaml");

// log     = maze_logs/*.maze   機体から受信した迷路(読み取り専用)
// edit    = profile/hf/*.maze  編集用(上書き保存できる)+ profile/maze.yaml
// contest = maze_data/*.yaml   大会迷路(ゴール付き、読み取り専用)
export type MazeGroup = "log" | "edit" | "contest";

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
  if (group === "contest" && name.endsWith(".yaml")) return { group, name, file: path.join(MAZE_DATA_DIR, name) };
  if (group === "edit" && name === LEGACY_EDIT_FILE) return { group, name, file: path.join(PROFILE_DIR, name) };
  if (group === "edit" && name.endsWith(".maze")) return { group, name, file: path.join(EDIT_DIR, name) };
  throw new Error("不明なファイルです");
}

function listDir(dir: string, group: MazeGroup, re: RegExp): MazeFileInfo[] {
  if (!fs.existsSync(dir)) return [];
  return fs
    .readdirSync(dir)
    .filter((name) => re.test(name))
    .map((name) => ({ id: `${group}/${name}`, group, name, mtimeMs: fs.statSync(path.join(dir, name)).mtimeMs }));
}

export function listMazeFiles(): MazeFileInfo[] {
  const logs = listDir(MAZE_LOGS_DIR, "log", /\.maze$/).sort((a, b) => b.mtimeMs - a.mtimeMs);
  const edits = listDir(EDIT_DIR, "edit", /\.maze$/);
  if (fs.existsSync(path.join(PROFILE_DIR, LEGACY_EDIT_FILE))) {
    const mtimeMs = fs.statSync(path.join(PROFILE_DIR, LEGACY_EDIT_FILE)).mtimeMs;
    edits.push({ id: `edit/${LEGACY_EDIT_FILE}`, group: "edit", name: LEGACY_EDIT_FILE, mtimeMs });
  }
  edits.sort((a, b) => a.name.localeCompare(b.name));
  const contests = listDir(MAZE_DATA_DIR, "contest", /\.yaml$/).sort((a, b) => a.name.localeCompare(b.name));
  return [...logs, ...edits, ...contests];
}

interface ContestYaml {
  maze_data?: { maze_size?: number; wall?: number[]; goal?: Cell[] };
}

export function readMaze(id: string): MazeContent {
  const { group, file } = resolveMazePath(id);
  if (!fs.existsSync(file)) throw new Error("ファイルが見つかりません");
  const text = fs.readFileSync(file, "utf-8");
  if (group === "contest") {
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
  return { id, size: mazeSizeOf(walls.length), walls, goals: null, editable: group === "edit" };
}

export function checkWalls(walls: unknown): number[] {
  if (!Array.isArray(walls) || !walls.every((w) => Number.isInteger(w) && w >= 0 && w <= 0x0f)) {
    throw new Error("壁データが不正です");
  }
  mazeSizeOf(walls.length);
  return walls as number[];
}

export function writeMaze(id: string, walls: unknown): void {
  const { group, file } = resolveMazePath(id);
  if (group !== "edit") throw new Error("このファイルは上書きできません。別名で保存してください");
  const w = checkWalls(walls);
  fs.writeFileSync(file, formatMazeText(w, mazeSizeOf(w.length)), "utf-8");
}

// profile/hf/<name>.maze へ新規保存。既存ファイルは上書きしない。
export function saveMazeAs(name: string, walls: unknown): string {
  const base = name.trim().replace(/\.maze$/, "");
  if (!/^[\w-]+$/.test(base)) throw new Error("ファイル名は英数字・_・- だけにしてください");
  const file = path.join(EDIT_DIR, `${base}.maze`);
  if (fs.existsSync(file)) throw new Error(`${base}.maze は既にあります。名前を変えてください`);
  const w = checkWalls(walls);
  fs.mkdirSync(EDIT_DIR, { recursive: true });
  fs.writeFileSync(file, formatMazeText(w, mazeSizeOf(w.length)), "utf-8");
  return `edit/${base}.maze`;
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
