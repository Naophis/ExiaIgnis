// 詳細ログ解析ページ(/logs)用の定数。motion_state の名前/色と、走行チューニングで
// よく一緒に見る列の組み合わせ(プリセット)。
// motion_state の番号は include/enums.hpp の MotionType と一致させること。

export const MOTION_STATE_NAME: Record<number, string> = {
  0: "NONE",
  1: "STRAIGHT",
  2: "PIVOT",
  3: "SLA_FRONT_STR",
  4: "SLALOM",
  5: "BACK_STRAIGHT",
  6: "WALL_OFF",
  7: "READY",
  8: "PIVOT_PRE",
  9: "PIVOT_PRE2",
  10: "PIVOT_AFTER",
  11: "FRONT_CTRL",
  12: "PIVOT_OFFSET",
  13: "WALL_OFF_DIA",
  14: "SLA_BACK_STR",
  15: "SYS_ID_PARA",
  16: "SYS_ID_ROLL",
  17: "SENSING_DUMP",
};

// グラフ背景の帯。旋回(SLALOM)と前後の繋ぎ(SLA_FRONT_STR/SLA_BACK_STR/WALL_OFF)が
// 目で分かれば十分なので、直進や停止は透明にして帯だらけにしない。
export const MOTION_STATE_BAND: Record<number, string> = {
  2: "rgba(244,114,182,0.10)", // PIVOT
  3: "rgba(56,189,248,0.10)", // SLA_FRONT_STR
  4: "rgba(168,85,247,0.16)", // SLALOM
  6: "rgba(34,197,94,0.10)", // WALL_OFF
  13: "rgba(34,197,94,0.16)", // WALL_OFF_DIA
  14: "rgba(56,189,248,0.16)", // SLA_BACK_STR
};

export function motionStateLabel(state: number): string {
  return MOTION_STATE_NAME[state] ?? `state ${state}`;
}

// グラフ1枚あたりの系列色(列の並び順に割り当てる)
export const SERIES_COLORS = [
  "#61afef",
  "#e06c75",
  "#98c379",
  "#d19a66",
  "#c678dd",
  "#56b6c2",
  "#e5c07b",
  "#abb2bf",
];

export const DEFAULT_CHART_HEIGHT = 160;

export interface ChartSpec {
  id: string;
  title: string;
  columns: string[];
  /** グラフの高さ [px]。ドラッグで変えられ、localStorage に保存される。 */
  height?: number;
}

// 初期表示。ターン調整で実際に並べて見る組み合わせ
// (目標と実測を同じグラフに置き、指令 duty は別グラフにする)。
export const DEFAULT_CHARTS: Omit<ChartSpec, "id">[] = [
  { title: "角速度", columns: ["ideal_w", "w_lp"] },
  { title: "角度", columns: ["ideal_ang", "ang", "kim_theta"] },
  { title: "壁PD", columns: ["duty_sen", "s_pid_p"] },
  { title: "速度", columns: ["ideal_v", "v_c", "v_l", "v_r"] },
  { title: "duty", columns: ["duty_l", "duty_r"] },
];

// カーソル位置の読み取り行に常に出す列
export const CURSOR_READOUT_COLUMNS = [
  "motion_state",
  "ideal_v",
  "v_c",
  "ideal_w",
  "w_lp",
  "ideal_ang",
  "ang",
  "kim_theta",
  "left45_d",
  "right45_d",
  "duty_sen",
  "duty_l",
  "duty_r",
];

// 全 NaN の列(そのファームでは出ていない列)を除いた、選べる列の一覧
export function usableColumns(rows: Record<string, number>[]): string[] {
  if (rows.length === 0) return [];
  const keys = Object.keys(rows[0]);
  const step = Math.max(1, Math.floor(rows.length / 200));
  return keys.filter((k) => {
    if (k === "index") return false; // x 軸そのもの
    for (let i = 0; i < rows.length; i += step) {
      if (Number.isFinite(rows[i][k])) return true;
    }
    return false;
  });
}

export interface StateBlock {
  x0: number;
  x1: number;
  state: number;
}

// motion_state の連続区間。x は CSV の index 列。
export function motionStateBlocks(rows: Record<string, number>[]): StateBlock[] {
  const out: StateBlock[] = [];
  let s = 0;
  for (let i = 1; i <= rows.length; i++) {
    if (i === rows.length || rows[i].motion_state !== rows[s].motion_state) {
      out.push({ x0: rows[s].index, x1: rows[i - 1].index, state: rows[s].motion_state });
      s = i;
    }
  }
  return out;
}
