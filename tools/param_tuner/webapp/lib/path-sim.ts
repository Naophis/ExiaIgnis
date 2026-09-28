import { spawn } from "node:child_process";
import fs from "node:fs";
import path from "node:path";
import { load as loadYaml } from "js-yaml";
import { mazeSizeOf, type Cell } from "./maze-shared";

// tools/path_sim(MainTask::path_run() の経路生成のホスト版)を呼ぶ。
// 経路生成はファームのソースそのもの。実行前に毎回 `make -s` を通すので、
// ファームのソースを変えると次の計算から反映される(初回ビルドは十数秒)。

// webapp/ is the Next.js server cwd; tools/param_tuner/ is one level up.
const PARAM_TUNER_ROOT = path.join(process.cwd(), "..");
const PROFILE_DIR = path.join(PARAM_TUNER_ROOT, "profile");
const MODE = "hf";
const PATH_SIM_DIR = path.join(PARAM_TUNER_ROOT, "..", "path_sim");
const PATH_SIM_BIN = path.join(PATH_SIM_DIR, "build", "path_sim");
const BUILD_TIMEOUT_MS = 180_000;
const RUN_TIMEOUT_MS = 20_000;

export type PathDirection = "right" | "left";

export interface ExecOption {
  index: number; // run_prf の exec_prof の番号(機体のモード番号 − 2)
  fast: number;
  normal: number;
  slow: number;
  vMax: number | null; // vel_prof[normal].fast.v(直線の最高速)
  turnFile: string | null; // fast プロファイルの large のファイル(速度の目安)
}

export interface PathSegment {
  str_time: number;
  turn_time: number;
  total_time: number;
  v_start: number;
  v_max: number;
  v_end: number;
  dist: number;
}

export interface PathCandidate {
  type: number; // lgc->set_param_num()
  result: boolean;
  time: number;
  path_s: number[];
  path_t: number[];
}

// load_slalom_param() が読んだターンのパラメータ(ang は度、time は読込時に計算された値)。
export interface TurnParam {
  type: string; // "large" など(turn_name_list の名前)
  file?: string; // profiles の list のファイル名
  v?: number;
  end_v?: number;
  ang?: number;
  rad?: number;
  rad2?: number;
  pow_n?: number;
  time?: number;
  time2?: number;
  front_l?: number;
  front_r?: number;
  back_l?: number;
  back_r?: number;
}

export type TurnParamSet = "fast" | "normal" | "slow";

export interface PathSimResult {
  ok: boolean;
  error?: string;
  exec?: { index: number; fast: number; normal: number; slow: number };
  direction?: PathDirection;
  suction?: number;
  cell_size?: number;
  start_offset?: number;
  selected_type?: number; // -1 = 候補がすべて失敗して単純な経路 / left
  candidates?: PathCandidate[];
  path_s?: number[];
  path_t?: number[];
  segments?: PathSegment[];
  goal_time?: number;
  turn_params?: Record<TurnParamSet, TurnParam[]>;
  straight_params?: Record<string, { v_max: number; accl: number; decel: number }>;
  log: string; // ファームの printf(実機のコンソールと同じ)
}

// 機体へ送るときと同じ名前・同じ変換(yaml → JSON)。serial-manager の sendFile 参照。
function profileFiles(): Record<string, string> {
  const files: Record<string, string> = {};
  for (const f of ["system.yaml", "hardware.yaml"]) {
    files[f.replace("yaml", "txt")] = JSON.stringify(loadYaml(fs.readFileSync(path.join(PROFILE_DIR, f), "utf-8")));
  }
  const modeDir = path.join(PROFILE_DIR, MODE);
  for (const f of fs.readdirSync(modeDir).filter((n) => n.endsWith(".yaml"))) {
    files[f.replace("yaml", MODE)] = JSON.stringify(loadYaml(fs.readFileSync(path.join(modeDir, f), "utf-8")));
  }
  return files;
}

function readModeYaml<T>(file: string): T | null {
  try {
    return loadYaml(fs.readFileSync(path.join(PROFILE_DIR, MODE, file), "utf-8")) as T;
  } catch {
    return null;
  }
}

export function readExecOptions(): ExecOption[] {
  const run = readModeYaml<{ exec_prof?: { fast?: number; normal?: number; slow?: number }[] }>("run_prf.yaml");
  const vel = readModeYaml<{ v_prof?: { fast?: { v?: number } }[] }>("vel_prof.yaml");
  const prof = readModeYaml<{ list?: string[]; profile_idx?: { large?: number }[] }>("profiles.yaml");
  return (run?.exec_prof ?? []).map((e, index) => {
    const fast = e.fast ?? 0;
    const normal = e.normal ?? 0;
    const large = prof?.profile_idx?.[fast]?.large;
    return {
      index,
      fast,
      normal,
      slow: e.slow ?? 0,
      vMax: vel?.v_prof?.[normal]?.fast?.v ?? null,
      turnFile: large !== undefined ? (prof?.list?.[large]?.replace(/\.hf$/, "") ?? null) : null,
    };
  });
}

function runProcess(
  cmd: string,
  args: string[],
  opts: { cwd?: string; input?: string; timeoutMs: number },
): Promise<{ code: number | null; stdout: string; stderr: string }> {
  return new Promise((resolve, reject) => {
    const child = spawn(cmd, args, { cwd: opts.cwd, stdio: ["pipe", "pipe", "pipe"] });
    let stdout = "";
    let stderr = "";
    const timer = setTimeout(() => {
      child.kill("SIGKILL");
      reject(new Error(`${path.basename(cmd)} がタイムアウトしました (${opts.timeoutMs / 1000}s)`));
    }, opts.timeoutMs);
    child.stdout.on("data", (b: Buffer) => (stdout += b.toString()));
    child.stderr.on("data", (b: Buffer) => (stderr += b.toString()));
    child.on("error", (err) => {
      clearTimeout(timer);
      reject(err);
    });
    child.on("close", (code) => {
      clearTimeout(timer);
      resolve({ code, stdout, stderr });
    });
    child.stdin.end(opts.input ?? "");
  });
}

async function ensureBuilt(): Promise<void> {
  const { code, stdout, stderr } = await runProcess("make", ["-s"], { cwd: PATH_SIM_DIR, timeoutMs: BUILD_TIMEOUT_MS });
  if (code !== 0) {
    const tail = (stderr || stdout).trim().split("\n").slice(-12).join("\n");
    throw new Error(`path_sim のビルドに失敗しました:\n${tail}`);
  }
}

// make と実行を 1 本ずつにする(同時に make が走ると .o の書き込みがぶつかる)。
let queue: Promise<unknown> = Promise.resolve();

export function runPathSim(req: {
  walls: number[]; // .maze の並び(idx = x * size + y)
  goals: Cell[] | null;
  exec: number;
  direction: PathDirection;
}): Promise<PathSimResult> {
  const job = async (): Promise<PathSimResult> => {
    const size = mazeSizeOf(req.walls.length);
    // ファームの並び map[x + y * size]。シミュレータでは全マス既知(踏破済み)として扱う。
    const map = new Array<number>(size * size);
    for (let x = 0; x < size; x++) {
      for (let y = 0; y < size; y++) map[x + y * size] = (req.walls[x * size + y] & 0x0f) | 0xf0;
    }
    const input = JSON.stringify({
      files: profileFiles(),
      map,
      exec: req.exec,
      direction: req.direction,
      ...(req.goals ? { goals: req.goals } : {}),
    });
    await ensureBuilt();
    const { stdout, stderr } = await runProcess(PATH_SIM_BIN, [], { input, timeoutMs: RUN_TIMEOUT_MS });
    let parsed: Omit<PathSimResult, "log">;
    try {
      parsed = JSON.parse(stdout);
    } catch {
      const tail = stderr.trim().split("\n").slice(-12).join("\n");
      throw new Error(`path_sim の出力を読めません(異常終了?):\n${tail}`);
    }
    return { ...parsed, log: stderr };
  };
  const run = queue.then(job, job);
  queue = run.catch(() => undefined);
  return run;
}
