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
  /**
   * 表示する列。`name` か `name*係数`(PlotJuggler の Scale 変換と同じ)。
   * 係数付きは桁が違う列を同じグラフに重ねるため(例: motion_state*100)。
   */
  columns: string[];
  /** グラフの高さ [px]。ドラッグで変えられ、localStorage に保存される。 */
  height?: number;
}

/** `alpha*0.01` のような列指定を分解する。係数なしなら scale=1。 */
export function parseColumnSpec(spec: string): { column: string; scale: number } {
  const i = spec.lastIndexOf("*");
  if (i > 0) {
    const scale = parseFloat(spec.slice(i + 1));
    if (Number.isFinite(scale)) return { column: spec.slice(0, i), scale };
  }
  return { column: spec, scale: 1 };
}

/** 列チップに出す表示名 */
export function columnSpecLabel(spec: string): string {
  const { column, scale } = parseColumnSpec(spec);
  return scale === 1 ? column : `${column}×${scale}`;
}

export interface ChartView {
  key: string;
  title: string;
  charts: Omit<ChartSpec, "id">[];
}

// 観点ごとのグラフ構成。PlotJuggler のレイアウト(tools/param_tuner/profile.xml)の
// タブとプロットをそのまま写したもの。profile.xml を編集したらこちらも合わせる
// (xy プロットと hf のスロット配列は軌跡プロット側の担当なので省いている)。
export const CHART_VIEWS: ChartView[] = [
  {
    key: "turn",
    title: "旋回",
    charts: [
      { title: "角速度", columns: ["ideal_w", "w_lp"] },
      { title: "角度", columns: ["ideal_ang", "ang", "kim_theta"] },
      { title: "壁PD", columns: ["duty_sen", "s_pid_p"] },
      { title: "速度", columns: ["ideal_v", "v_c", "v_l", "v_r"] },
      { title: "duty", columns: ["duty_l", "duty_r"] },
    ],
  },
  {
    key: "overview",
    title: "概観",
    charts: [
      { title: "velocity", columns: ["v_c", "ideal_v", "v_c2"] },
      { title: "distance", columns: ["dist_kf", "ideal_dist", "dist"] },
      { title: "angular_velocity", columns: ["w_lp", "ideal_w", "alpha*0.01"] },
      { title: "angle", columns: ["ang_kf", "ideal_ang", "ang"] },
      { title: "duty", columns: ["duty_l", "duty_r", "duty_sen", "duty_suction*0.01"] },
      { title: "battery", columns: ["battery", "battery_raw"] },
      { title: "accel_raw", columns: ["accl", "accl_x"] },
      { title: "motion_state", columns: ["motion_state"] },
    ],
  },
  {
    key: "ctrl",
    title: "制御",
    charts: [
      { title: "v_ref", columns: ["v_c", "ideal_v", "v_c2", "v_kf_l", "v_kf_r"] },
      { title: "m_pid", columns: ["m_pid_p", "m_pid_i", "m_pid_i2", "m_pid_d"] },
      { title: "m_pid_v", columns: ["m_pid_p_v", "m_pid_i_v", "m_pid_i2_v", "m_pid_d_v"] },
      { title: "w_ref", columns: ["w_lp", "ideal_w", "duty_roll_before"] },
      { title: "ang_ref", columns: ["ideal_ang", "ang_kf", "motion_state", "ang_kf_sum"] },
      { title: "g_pid", columns: ["g_pid_p", "g_pid_i", "g_pid_i2", "g_pid_d"] },
      { title: "ang_pid", columns: ["ang_pid_p", "ang_pid_i*0.01", "ang_pid_d", "ang_i_bias", "ang_i_bias_val"] },
      {
        title: "g_pid_v",
        columns: ["g_pid_p_v", "g_pid_i_v", "g_pid_i2_v", "g_pid_d_v", "duty_roll", "sat_roll_dir", "mpc_d_estimated"],
      },
      { title: "ang_pid_v", columns: ["ang_pid_p_v", "ang_pid_i_v", "ang_pid_d_v"] },
      { title: "s_pid", columns: ["s_pid_p", "s_pid_i", "s_pid_i2", "s_pid_d"] },
      { title: "s_pid_v", columns: ["s_pid_p_v", "s_pid_i_v", "s_pid_i2_v", "s_pid_d_v", "duty_sen"] },
    ],
  },
  {
    key: "feedforward",
    title: "FFトルク",
    charts: [
      { title: "ff_duty", columns: ["ff_duty_front", "ff_duty_roll", "ff_duty_rpm_l", "ff_duty_rpm_r"] },
      { title: "ff_torque", columns: ["ff_front_torque", "ff_roll_torque"] },
      { title: "friction_torque", columns: ["ff_friction_torque_l", "ff_friction_torque_r"] },
      { title: "v_kf_wheel", columns: ["v_kf_l", "v_kf_r", "v_c"] },
      { title: "mpc_d_estimated", columns: ["mpc_d_estimated"] },
      { title: "sat_roll_dir", columns: ["sat_roll_dir", "motion_state"] },
    ],
  },
  {
    key: "position",
    title: "位置",
    charts: [
      { title: "x", columns: ["x"] },
      { title: "y", columns: ["y"] },
      { title: "v_ref", columns: ["ideal_v", "v_c"] },
      { title: "dist_ref", columns: ["dist_kf", "ideal_dist"] },
      { title: "w_ref", columns: ["ideal_w", "w_lp"] },
      { title: "ang_ref", columns: ["ang_kf", "ideal_ang"] },
      { title: "side45_d", columns: ["left45_d", "right45_d"] },
      { title: "side90_d", columns: ["left90_d", "right90_d"] },
    ],
  },
  {
    key: "kanayama",
    title: "Kanayama",
    charts: [
      { title: "v", columns: ["v_c", "v_c2", "ideal_v", "knym_v"] },
      { title: "w", columns: ["ideal_w", "w_lp", "knym_w"] },
      { title: "theta", columns: ["odm_theta", "kim_theta", "ang_kf"] },
      { title: "odm_x/kim_x", columns: ["odm_x", "kim_x"] },
      { title: "odm_y/kim_y", columns: ["odm_y", "kim_y"] },
      { title: "xy", columns: ["x", "y"] },
    ],
  },
  {
    key: "side_sensors",
    title: "横センサー",
    charts: [
      { title: "left45_raw", columns: ["left45", "left45_2", "left45_3"] },
      { title: "right45_raw", columns: ["right45", "right45_2", "right45_3"] },
      { title: "left45_d", columns: ["left45_d", "left45_2_d", "left45_3_d"] },
      { title: "right45_d", columns: ["right45_d", "right45_2_d", "right45_3_d"] },
      { title: "sen_dist_left45", columns: ["sen_dist_l45", "sen_dist_l45_2", "sen_dist_l45_3"] },
      { title: "sen_dist_right45", columns: ["sen_dist_r45", "sen_dist_r45_2", "sen_dist_r45_3"] },
      { title: "left45_d_diff", columns: ["left45_d_diff", "left45_2_d_diff", "left45_3_d_diff"] },
      { title: "right45_d_diff", columns: ["right45_d_diff", "right45_2_d_diff", "right45_3_d_diff"] },
    ],
  },
  {
    key: "front_sensors",
    title: "前センサー",
    charts: [
      { title: "raw", columns: ["front", "left90", "right90"] },
      { title: "front_d", columns: ["front_d", "front_far_d"] },
      { title: "side90_d", columns: ["left90_d", "right90_d"] },
      { title: "side90_far_d", columns: ["left90_far_d", "right90_far_d"] },
      { title: "side90_d_diff", columns: ["left90_d_diff", "right90_d_diff"] },
      { title: "dist_mod90", columns: ["dist_mod90", "motion_state"] },
    ],
  },
  {
    key: "wall_off",
    title: "壁切れ",
    charts: [
      { title: "left45_d_diff", columns: ["left45_d_diff", "left45_2_d_diff", "left45_3_d_diff"] },
      { title: "right45_d_diff", columns: ["right45_d_diff", "right45_2_d_diff", "right45_3_d_diff"] },
      { title: "front", columns: ["front_d", "front_far_d"] },
      { title: "left45_all", columns: ["left45_d", "left45_2_d", "left45_3_d", "sen_dist_l45"] },
      { title: "right45_all", columns: ["right45_d", "right45_2_d", "right45_3_d", "sen_dist_r45"] },
      { title: "w_ref", columns: ["ideal_w", "w_lp"] },
      { title: "duty_sen", columns: ["duty_sen"] },
      { title: "motion_state", columns: ["motion_state", "continuous_turn"] },
    ],
  },
  {
    key: "wall_off_hf",
    title: "壁切れhf",
    charts: [
      { title: "side45 (1kHz)", columns: ["left45_d", "right45_d", "sen_dist_l45", "sen_dist_r45"] },
      { title: "hf_edge_rel", columns: ["hf_edge_rel"] },
      { title: "hf_cnt / hf_side", columns: ["hf_cnt", "hf_side"] },
      { title: "motion_state", columns: ["motion_state", "continuous_turn"] },
      { title: "dist / ideal_dist", columns: ["dist", "ideal_dist"] },
    ],
  },
  {
    key: "imu_accel",
    title: "IMU加速度",
    charts: [
      { title: "accel_raw_xyz", columns: ["accel_x", "accel_y", "accel_z"] },
      { title: "accel_corr_xyz", columns: ["accel_x_corr", "accel_y_corr", "accel_z_corr"] },
      { title: "accel_x_cmp", columns: ["accel_x", "accel_x_corr"] },
      { title: "accel_y_cmp", columns: ["accel_y", "accel_y_corr"] },
      { title: "accel_z_cmp", columns: ["accel_z", "accel_z_corr"] },
      { title: "motion_ref", columns: ["motion_state", "w_lp"] },
    ],
  },
  {
    key: "encoder",
    title: "エンコーダ",
    charts: [
      { title: "enc_raw", columns: ["v_l_enc", "v_r_enc"] },
      { title: "enc_sin", columns: ["v_l_enc_sin", "v_r_enc_sin"] },
      { title: "v_kf_wheel", columns: ["v_kf_l", "v_kf_r"] },
      { title: "v_lr_raw", columns: ["v_l", "v_r"] },
      { title: "v_ref", columns: ["v_c", "ideal_v"] },
    ],
  },
  {
    key: "calc_time",
    title: "計算時間",
    charts: [
      {
        title: "planning",
        columns: [
          "pln_calc_time",
          "pln_calc_time2",
          "pln_time_diff",
          "pln_t_copy",
          "pln_t_ctl",
          "pln_t_ego",
          "pln_t_kanayama",
          "pln_t_sensor",
          "pln_t_trj",
          "motion_state*100",
        ],
      },
      { title: "sensing", columns: ["sen_calc_time", "sen_calc_time2", "motion_state*100"] },
      { title: "dither", columns: ["dither_consumed", "dither_lead", "dither_late", "dither_backlog"] },
    ],
  },
  {
    key: "debug_misc",
    title: "その他",
    charts: [
      { title: "dbg_off", columns: ["dbg_off_ang", "dbg_off_wgain", "dbg_off_kny"] },
      { title: "continuous_turn", columns: ["continuous_turn", "motion_state"] },
      { title: "ang_sum", columns: ["ang_kf_sum", "img_ang_sum"] },
      { title: "wall_fit", columns: ["wfit_beta", "wfit_sig"] },
    ],
  },
];

export const DEFAULT_VIEW_KEY = "turn";

export function viewByKey(key: string): ChartView {
  return CHART_VIEWS.find((v) => v.key === key) ?? CHART_VIEWS[0];
}

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
