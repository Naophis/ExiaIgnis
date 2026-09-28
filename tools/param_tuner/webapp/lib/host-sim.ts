import { spawn } from "node:child_process";
import fs from "node:fs";
import path from "node:path";
import { load as loadYaml } from "js-yaml";
import { mazeSizeOf } from "./maze-shared";

// tools/path_sim のホスト用コマンド(path_sim / search_sim)を呼ぶ共通部分。
// どちらもファームのソースそのものをビルドしたもので、実行前に毎回 `make -s` を
// 通すので、ファームのソースを変えると次の計算から反映される(初回ビルドは十数秒)。

// webapp/ is the Next.js server cwd; tools/param_tuner/ is one level up.
const PARAM_TUNER_ROOT = path.join(process.cwd(), "..");
export const PROFILE_DIR = path.join(PARAM_TUNER_ROOT, "profile");
export const MODE = "hf";
const PATH_SIM_DIR = path.join(PARAM_TUNER_ROOT, "..", "path_sim");
const BUILD_TIMEOUT_MS = 180_000;
const RUN_TIMEOUT_MS = 20_000;

// 機体へ送るときと同じ名前・同じ変換(yaml → JSON)。serial-manager の sendFile 参照。
export function profileFiles(): Record<string, string> {
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

// .maze の並び(idx = x * size + y)→ ファームの並び map[x + y * size]。
export function toFirmwareMap(walls: number[], orBits: number): number[] {
  const size = mazeSizeOf(walls.length);
  const map = new Array<number>(size * size);
  for (let x = 0; x < size; x++) {
    for (let y = 0; y < size; y++) map[x + y * size] = (walls[x * size + y] & 0x0f) | orBits;
  }
  return map;
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
    throw new Error(`tools/path_sim のビルドに失敗しました:\n${tail}`);
  }
}

// make と実行を 1 本ずつにする(同時に make が走ると .o の書き込みがぶつかる)。
let queue: Promise<unknown> = Promise.resolve();

// ビルドを確かめてから bin に input(JSON)を渡し、stdout の JSON と stderr(ファームの printf)を返す。
export function runHostSim<T>(bin: "path_sim" | "search_sim", input: unknown): Promise<T & { log: string }> {
  const job = async () => {
    await ensureBuilt();
    const { stdout, stderr } = await runProcess(path.join(PATH_SIM_DIR, "build", bin), [], {
      input: JSON.stringify(input),
      timeoutMs: RUN_TIMEOUT_MS,
    });
    let parsed: T;
    try {
      parsed = JSON.parse(stdout);
    } catch {
      const tail = stderr.trim().split("\n").slice(-12).join("\n");
      throw new Error(`${bin} の出力を読めません(異常終了?):\n${tail}`);
    }
    return { ...parsed, log: stderr };
  };
  const run = queue.then(job, job);
  queue = run.catch(() => undefined);
  return run;
}
