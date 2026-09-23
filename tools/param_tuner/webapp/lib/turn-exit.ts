// tools/param_tuner/turn_exit_check.py の移植(ブラウザ/Node 共用の純粋関数)。
// 旋回(motion_state=SLALOM)ごとに、旋回中の追従状態と旋回後の出口残差
// (横ずれ・ヨー)を出す。数式・しきい値・列名は .py と揃えてあるので、
// 変更するときは両方を同時に直すこと(.py の docstring が仕様の正本)。
//
// 「ターン直後の duty_sen が大きい」を旋回の良否に使うと、片壁×2 と
// ヨーの見かけ横ずれ(LAT_K mm/deg)で実際のずれを 3〜4 倍に誇張するため、
// 壁読み値から出口残差を直接出す。行順は CSV ファイル順(parseCsv の生出力)。

export const SLALOM_STATE = 4;
export const DEFAULT_POST_TICKS = 40;
// [mm/deg] ヨー1°あたりの45°センサーの見かけ横ずれ(hardware.yaml start_align.lat_k 実測)
export const DEFAULT_LAT_K = 0.96;
const DT = 0.001;
const G = 9806; // [mm/s^2]
const RAD2DEG = 180 / Math.PI;

export interface TurnExitOptions {
  log?: string;
  postTicks?: number;
  latK?: number;
}

export interface TurnExitRow {
  log: string;
  idx: number; // SLALOM 開始行
  endIdx: number; // SLALOM 最終行
  exitIdx: number; // 旋回後の最初の行
  kind: string;
  dir: "L" | "R";
  v: number;
  angDeg: number;
  // 斜めへ抜ける旋回(dia45/dia135/dia90)。出口で45°センサーが柱を見るので
  // off 系(off0/off/yaw/offC/wide)は NaN にする
  exitDiag: boolean;
  wmax: number; // |ideal_w| 最大 [rad/s]
  latg: number; // v*wmax/g [G]
  wOver: number; // プラトー域の |w_lp|-|ideal_w| 最大
  wUnder: number; // 同 最小
  vcMin: number; // 旋回中 v_c 最小 / ideal_v
  vIn: number; // 旋回中 内輪速度最小 / ideal_v (0.1未満は内輪停止=接触)
  sat: number; // |duty|>99 の tick 数
  lag: number; // 旋回最終 tick の ideal_ang-ang [deg]
  yaw0: number; // 旋回後 2tick 目の kim_theta [deg]
  off0: number; // 最初に両壁が見えた tick の (l45-r45)/2 [mm]、+は右
  off: number; // 旋回後 25〜50mm の (l45-r45)/2 平均 [mm]
  yaw: number; // 同区間の kim_theta 平均 [deg]
  offC: number; // off - latK*yaw
  wide: number; // offC を旋回の外側正に直した値 [mm]
  dsen40: number; // 旋回後 postTicks の |duty_sen| 最大 [deg]
  ey40: number; // 同 |s_pid_p| 最大 [mm]
}

export const TURN_EXIT_REQUIRED_COLUMNS = [
  "motion_state",
  "ideal_v",
  "v_c",
  "v_l",
  "v_r",
  "ideal_w",
  "w_lp",
  "ideal_ang",
  "ang",
  "kim_theta",
  "left45_d",
  "right45_d",
  "duty_l",
  "duty_r",
  "duty_sen",
  "s_pid_p",
] as const;

export function missingTurnExitColumns(rows: Record<string, number>[]): string[] {
  if (rows.length === 0) return [...TURN_EXIT_REQUIRED_COLUMNS];
  return TURN_EXIT_REQUIRED_COLUMNS.filter((c) => !(c in rows[0]));
}

// 旋回角 θ に対する出口横ずれ(外側正)の幾何感度 [mm/mm]:
//   Δlat ≈ Δrad*(1-cosθ) + Δfront*sinθ  (back は出口位置を横に動かさない)
export function turnAngleOf(kind: string): number | null {
  if (kind.startsWith("dia45")) return 45;
  if (kind.startsWith("dia135")) return 135;
  if (kind === "dia90" || kind === "large90" || kind === "normal90") return 90;
  if (kind === "orval180") return 180;
  const m = /^turn(\d+)$/.exec(kind);
  return m ? parseInt(m[1], 10) : null;
}

export function turnSensitivity(kind: string): { rad: number; front: number } | null {
  const th = turnAngleOf(kind);
  if (th === null) return null;
  const r = (th * Math.PI) / 180;
  return { rad: 1 - Math.cos(r), front: Math.sin(r) };
}

function classify(angDeg: number, diag: boolean, v: number): [string, boolean] {
  const a = Math.round(Math.abs(angDeg));
  if (a === 45) return [diag ? "dia45_2" : "dia45", !diag];
  if (a === 135) return [diag ? "dia135_2" : "dia135", !diag];
  if (a === 90) return [diag ? "dia90" : v < 600 ? "normal90" : "large90", diag];
  if (a === 180) return ["orval180", diag];
  return [`turn${a}`, diag];
}

function segments(states: number[]): Array<[number, number, number]> {
  const out: Array<[number, number, number]> = [];
  let s = 0;
  const n = states.length;
  for (let i = 1; i <= n; i++) {
    if (i === n || states[i] !== states[s]) {
      out.push([s, i, states[s]]);
      s = i;
    }
  }
  return out;
}

function bothWalls(r: Record<string, number>): boolean {
  return r.left45_d > 1 && r.left45_d < 90 && r.right45_d > 1 && r.right45_d < 90;
}

export function analyzeTurnExits(rows: Record<string, number>[], opts: TurnExitOptions = {}): TurnExitRow[] {
  const postTicks = opts.postTicks ?? DEFAULT_POST_TICKS;
  const latK = opts.latK ?? DEFAULT_LAT_K;
  const log = opts.log ?? "";
  const n = rows.length;
  if (n === 0 || missingTurnExitColumns(rows).length > 0) return [];
  const states = rows.map((r) => r.motion_state);
  const out: TurnExitRow[] = [];
  // 走行開始は直線。45°/135° の旋回を通るたびに斜め⇄直線がトグルする
  let diag = false;

  for (const [s, e, st] of segments(states)) {
    if (st !== SLALOM_STATE || e - s < 5) continue;
    const seg = rows.slice(s, e);
    let wSum = 0;
    let wmax = 0;
    for (const r of seg) {
      wSum += r.ideal_w;
      wmax = Math.max(wmax, Math.abs(r.ideal_w));
    }
    const angDeg = wSum * DT * RAD2DEG;
    const v = seg[0].ideal_v;
    const [kind, diagAfter] = classify(angDeg, diag, v);
    diag = diagAfter;
    const exitDiag = diagAfter;
    const left = angDeg > 0;

    let wOver = -Infinity;
    let wUnder = Infinity;
    let vcMin = Infinity;
    let vIn = Infinity;
    let sat = 0;
    for (const r of seg) {
      if (Math.abs(r.ideal_w) > 0.5 * wmax) {
        const dw = Math.abs(r.w_lp) - Math.abs(r.ideal_w);
        wOver = Math.max(wOver, dw);
        wUnder = Math.min(wUnder, dw);
      }
      vcMin = Math.min(vcMin, r.v_c);
      vIn = Math.min(vIn, left ? r.v_l : r.v_r);
      if (Math.abs(r.duty_l) > 99 || Math.abs(r.duty_r) > 99) sat++;
    }
    const last = rows[e - 1];
    const lag = last.ideal_ang - last.ang;

    // 旋回後の窓: 次の SLALOM に入ったら打ち切り
    let postEnd = Math.min(e + Math.max(postTicks, 60), n);
    for (let i = e; i < postEnd; i++) {
      if (rows[i].motion_state === SLALOM_STATE) {
        postEnd = i;
        break;
      }
    }
    const post = rows.slice(e, postEnd);
    const yaw0 = post.length > 1 ? post[1].kim_theta : NaN;

    let off0 = NaN;
    if (!exitDiag) {
      const fb = post.find(bothWalls);
      if (fb) off0 = (fb.left45_d - fb.right45_d) / 2;
    }

    let off = NaN;
    let yaw = NaN;
    let offC = NaN;
    let wide = NaN;
    if (!exitDiag) {
      let travel = 0;
      let cnt = 0;
      let offSum = 0;
      let yawSum = 0;
      for (const r of post) {
        travel += r.ideal_v * DT;
        if (travel >= 25 && travel <= 50 && bothWalls(r) && r.left45_d < 60 && r.right45_d < 60) {
          cnt++;
          offSum += (r.left45_d - r.right45_d) / 2;
          yawSum += r.kim_theta;
        }
      }
      if (cnt >= 3) {
        off = offSum / cnt;
        yaw = yawSum / cnt;
        offC = off - latK * yaw;
        wide = left ? offC : -offC;
      }
    }

    let dsen40 = 0;
    let ey40 = 0;
    for (const r of post.slice(0, postTicks)) {
      dsen40 = Math.max(dsen40, Math.abs(r.duty_sen) * RAD2DEG);
      ey40 = Math.max(ey40, Math.abs(r.s_pid_p));
    }

    out.push({
      log,
      idx: s,
      endIdx: e - 1,
      exitIdx: e,
      kind,
      dir: left ? "L" : "R",
      v,
      angDeg,
      exitDiag,
      wmax,
      latg: (v * wmax) / G,
      wOver: Number.isFinite(wOver) ? wOver : NaN,
      wUnder: Number.isFinite(wUnder) ? wUnder : NaN,
      vcMin: vcMin / v,
      vIn: vIn / v,
      sat,
      lag,
      yaw0,
      off0,
      off,
      yaw,
      offC,
      wide,
      dsen40,
      ey40,
    });
  }
  return out;
}

// ---- 表示ヘルパー(パネル・詳細ページ・マーカーのラベルで共用) ----

/** NaN を "–" にする数値整形 */
export const fmtNum = (x: number, digits = 1): string => (Number.isFinite(x) ? x.toFixed(digits) : "–");

/** テーブル行の選択キー。idx だけだと別ログの同じ行番号に当たりうるのでログ名を含める。 */
export const turnKey = (t: TurnExitRow): string => `${t.log}|${t.idx}`;

/** 軌跡プロットの出口マーカーに出す1行 */
export function turnExitLabel(t: TurnExitRow): string {
  return `turn-exit idx=${t.idx} ${t.kind} ${t.dir} v=${t.v} wide=${fmtNum(t.wide)} yaw0=${fmtNum(t.yaw0)}° sat=${t.sat} v_in=${fmtNum(t.vIn, 2)}`;
}

/** 行を選んだときに読み取り欄へ出す全項目 */
export function formatTurnExit(t: TurnExitRow): string {
  return [
    `${t.kind} ${t.dir} v=${t.v} idx=${t.idx}-${t.endIdx} exit=${t.exitIdx}`,
    `wmax=${fmtNum(t.wmax)} (${fmtNum(t.latg)}G)`,
    `w+${fmtNum(t.wOver)}/${fmtNum(t.wUnder)}`,
    `vc_min=${fmtNum(t.vcMin, 2)} v_in=${fmtNum(t.vIn, 2)} sat=${t.sat}`,
    `lag=${fmtNum(t.lag)}° yaw0=${fmtNum(t.yaw0)}°`,
    `off0=${fmtNum(t.off0)} off=${fmtNum(t.off)} yaw=${fmtNum(t.yaw)}° off_c=${fmtNum(t.offC)} wide=${fmtNum(t.wide)}`,
    `dsen40=${fmtNum(t.dsen40, 0)}° ey40=${fmtNum(t.ey40)}`,
  ].join(" | ");
}

export interface Stat {
  n: number;
  mean: number;
  std: number; // 母標準偏差(ddof=0、.py と同じ)
}

export interface TurnExitSummaryRow {
  kind: string;
  dir: "L" | "R";
  v: number;
  n: number;
  wide: Stat;
  off0: Stat;
  yaw0: Stat;
  lag: Stat;
  sat: Stat;
  vcMin: Stat;
  vIn: Stat;
  dsen40: Stat;
}

function stat(values: number[]): Stat {
  const xs = values.filter((x) => Number.isFinite(x));
  if (xs.length === 0) return { n: 0, mean: NaN, std: NaN };
  const mean = xs.reduce((a, b) => a + b, 0) / xs.length;
  const varSum = xs.reduce((a, b) => a + (b - mean) * (b - mean), 0);
  return { n: xs.length, mean, std: Math.sqrt(varSum / xs.length) };
}

export function summarizeTurnExits(rows: TurnExitRow[]): TurnExitSummaryRow[] {
  const groups = new Map<string, TurnExitRow[]>();
  for (const r of rows) {
    const key = `${r.kind}|${r.dir}|${r.v}`;
    const g = groups.get(key);
    if (g) g.push(r);
    else groups.set(key, [r]);
  }
  const out: TurnExitSummaryRow[] = [];
  for (const g of groups.values()) {
    const pick = (f: (r: TurnExitRow) => number) => stat(g.map(f));
    out.push({
      kind: g[0].kind,
      dir: g[0].dir,
      v: g[0].v,
      n: g.length,
      wide: pick((r) => r.wide),
      off0: pick((r) => r.off0),
      yaw0: pick((r) => r.yaw0),
      lag: pick((r) => r.lag),
      sat: pick((r) => r.sat),
      vcMin: pick((r) => r.vcMin),
      vIn: pick((r) => r.vIn),
      dsen40: pick((r) => r.dsen40),
    });
  }
  return out.sort((a, b) => a.kind.localeCompare(b.kind) || a.dir.localeCompare(b.dir) || a.v - b.v);
}
