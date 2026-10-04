import fs from "node:fs";
import path from "node:path";
import { load as loadYaml } from "js-yaml";
import { MODE, profileFiles, runHostSim, toFirmwareMap } from "./host-sim";
import { profileDir } from "./machines";
import type { Cell } from "./maze-shared";

// tools/path_sim の path_sim(MainTask::path_run() の経路生成のホスト版)を呼ぶ。

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
  // 経路を作った方法。time = タイム最小の経路探索 / patterns = 重みパターン 1〜5 の比較(予備)/
  // simple = path_create の経路そのまま
  method_used?: "time" | "patterns" | "simple";
  // タイム最小の経路探索(TimePathPlanner)の結果。result が ok 以外なら予備へ戻っている
  planner?: {
    result: string;
    time: number;
    ms: number; // 計算時間(PC 上)
    nodes: number;
    node_cap: number;
    heap_max: number;
    edges: number;
    seg_cached: number;
    mem_bytes: number;
  };
  selected_type?: number; // -1 = 候補がすべて失敗して単純な経路 / left / タイム最小の経路探索
  candidates?: PathCandidate[]; // 重みパターンの比較をしたときだけ
  path_s?: number[];
  path_t?: number[];
  segments?: PathSegment[];
  goal_time?: number;
  turn_params?: Record<TurnParamSet, TurnParam[]>;
  straight_params?: Record<string, { v_max: number; accl: number; decel: number }>;
  log: string; // ファームの printf(実機のコンソールと同じ)
}

function readModeYaml<T>(machine: string, file: string): T | null {
  try {
    return loadYaml(fs.readFileSync(path.join(profileDir(machine), MODE, file), "utf-8")) as T;
  } catch {
    return null;
  }
}

export function readExecOptions(machine: string): ExecOption[] {
  const run = readModeYaml<{ exec_prof?: { fast?: number; normal?: number; slow?: number }[] }>(machine, "run_prf.yaml");
  const vel = readModeYaml<{ v_prof?: { fast?: { v?: number } }[] }>(machine, "vel_prof.yaml");
  const prof = readModeYaml<{ list?: string[]; profile_idx?: { large?: number }[] }>(machine, "profiles.yaml");
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

export function runPathSim(req: {
  machine: string; // どの機体の走行パラメータで計算するか
  walls: number[]; // .maze の並び(idx = x * size + y)
  goals: Cell[] | null;
  exec: number;
  direction: PathDirection;
}): Promise<PathSimResult> {
  // シミュレータでは全マス既知(踏破済み)として扱う。
  return runHostSim<Omit<PathSimResult, "log">>("path_sim", {
    files: profileFiles(req.machine),
    map: toFirmwareMap(req.walls, 0xf0),
    exec: req.exec,
    direction: req.direction,
    ...(req.goals ? { goals: req.goals } : {}),
  });
}
