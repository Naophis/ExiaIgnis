import { profileFiles, runHostSim, toFirmwareMap } from "./host-sim";
import type { Cell } from "./maze-shared";

// tools/path_sim の search_sim(SearchController::exec() の探索のホスト版)を呼ぶ。
// 足立法と迷路ロジックはファームのソースそのもの。探索ループの写しと簡略化の内容は
// tools/path_sim/search_main.cpp の冒頭を参照。

// 1 ステップ = 足立法の判断 1 回。位置はファームの座標(区画)、向きは Direction の値
// (N=1 / E=2 / W=4 / S=8)。
export interface SearchStep {
  f: [number, number, number]; // 判断した区画(入口の境界にいる)と向き
  to: [number, number, number]; // 行き先
  m: "S" | "F" | "D" | "R" | "L" | "B"; // 探索直進 / 既知の直進 / 既知→ターン / 右 / 左 / 後退
  t0: number; // 判断した時刻 s
  t1: number; // 次の判断の時刻 s
  g: boolean; // ゴール到達後か
  sg: number; // 残りのサブゴール数
  c: [number, number][]; // 前の判断からの地図の変化(ファームの並びの idx, 値)
}

export interface SearchSimResult {
  ok: boolean;
  error?: string;
  maze_size?: number;
  end_reason?: "home" | "timeup" | "back4" | "none" | "step_limit";
  total_time?: number; // 最後の停止まで s
  goal_time?: number; // ゴール区画に最初に入った時刻 s(-1 = 届かず)
  goal_by?: "enter" | "known" | "none"; // known = 入らずに到達扱い(残り 1 区画の 4 辺が既知)
  finish_time?: number;
  search_timer?: number; // seach_timer s
  subgoal_mode?: number; // offset.yaml の search_subgoal_mode(0 = 重みパターン 1 を毎回 / 1 = 重みパターン 2 と 4)
  params?: {
    search_v: number;
    search_accl: number;
    fast_v: number;
    turn_v: number;
    turn_time: number;
    w_max: number;
    alpha: number;
    turn_file?: string;
  };
  steps?: SearchStep[];
  log: string;
}

export function runSearchSim(req: { walls: number[]; goals: Cell[] | null }): Promise<SearchSimResult> {
  return runHostSim<Omit<SearchSimResult, "log">>("search_sim", {
    files: profileFiles(),
    truth: toFirmwareMap(req.walls, 0),
    ...(req.goals ? { goals: req.goals } : {}),
  });
}
