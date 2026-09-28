// センサー距離キャリブレーション(旧 csv/sensor.sh + merge.sh + pyplot.py)の
// 純粋関数部分。ブラウザ/Node 共用。
//
// 入力はテストモード15(main_task_test_misc.cpp の dump2())が 100ms ごとに出す
// 9列の生値 "L90, L45_3, L45_2, L45, F, R45, R45_2, R45_3, R90"。
// 距離換算はファームの SensorProcessor::calc_sensor_val と同じ
//   dist = a / ln(raw) - b
// で、u = 1/ln(raw) と置けば a, b について線形なので最小二乗は閉形式で解ける
// (pyplot.py の curve_fit と同じ最適解になる)。

export const CHANNELS = ["L90", "L45_3", "L45_2", "L45", "F", "R45", "R45_2", "R45_3", "R90"] as const;
export type Channel = (typeof CHANNELS)[number];

// 壁の置き方。l/r/f は旧 csv の l_/r_/f_ プレフィックスと同じ。
// s は左右の壁を同時に置いた測定で、旧手順では同じ記録を l_ と r_ の両方に
// 保存していた(csv/ の l_*.csv と r_*.csv は中身が同一)。1回の記録で
// L45 系と R45 系の両方に使う。
export type CalibGroup = "s" | "l" | "r" | "f";
export const GROUP_LABEL: Record<CalibGroup, string> = { s: "横壁(左右)", l: "左壁", r: "右壁", f: "前壁" };
export const GROUP_ORDER: CalibGroup[] = ["s", "l", "r", "f"];

// 行の置き方がそのフィット対象(TargetDef.group は l/r/f)に使えるか。
export function rowServes(rowGroup: CalibGroup, targetGroup: CalibGroup): boolean {
  return rowGroup === targetGroup || (rowGroup === "s" && (targetGroup === "l" || targetGroup === "r"));
}

export interface CalibRow {
  id: string;
  group: CalibGroup;
  dist: number; // mm。スイープ行では開始位置 D0
  use: boolean; // フィットに使うか
  samples: number[][]; // 各要素は CHANNELS 順の9値
  // 前壁スイープ(テストモード28)の行だけが持つ。samples と同じ長さで、
  // 各サンプルの開始位置からの後退距離 [mm]。距離は dist + offsets[i]。
  // D0 を後から直せるよう、絶対距離ではなく相対で持つ。
  offsets?: number[];
  source?: string; // 取り込んだログ名
  pose?: SweepPose; // スイープ中の姿勢(取り込み時にログから計算)
}

// d0 を渡すとスイープ行の開始位置をそれで置き換える(アンカーから求めた値)。
export function sampleDist(r: CalibRow, i: number, d0?: number): number {
  return r.offsets ? (d0 ?? r.dist) + r.offsets[i] : r.dist;
}

// スイープ中の姿勢。測りたい範囲(終点から 100mm 手前まで)だけを見る。
// 45 度センサーの読みの傾きから「横壁に対する傾き」を出す案は、読みが向きの
// 変化でも約 1mm/度動くので当てにならず、やめた(実ログで 3.2 度と出たが
// ジャイロでは 1.5 度以内だった)。
export interface SweepPose {
  vMax: number; // 最高速度 [mm/s]
  headingDriftDeg: number; // 向きの変化幅(ジャイロ) [deg]
  // 横ずれ [mm]、左寄りが正。(右の読み − 左の読み)/2 の平均。45 度センサーが
  // 前壁を見始める手前(終点から 45mm 以上手前)だけで、両側の壁が見えているとき。
  latOffsetMm: number | null;
  wallCtrl: boolean; // 壁制御が働いていたか(duty_sen が出ていたか)
}

// sensor.yaml の gain セクションに書くキー。F(前壁制御ゲイン)は距離換算では
// ないので対象外(pyplot.py は F 列もフィットしていたが書き写し先が無い)。
export type TargetKey =
  | "L45_3"
  | "L45_2"
  | "L45"
  | "R45"
  | "R45_2"
  | "R45_3"
  | "L90_near"
  | "R90_near"
  | "L90_mid"
  | "R90_mid"
  | "L90_far"
  | "R90_far";

export interface TargetDef {
  key: TargetKey;
  group: CalibGroup;
  channel: Channel;
  refDist: number; // 「現在値との差」を評価する基準距離
  // 使う距離範囲の既定値。90度センサーは測定範囲が広く 1 組の a, b では
  // 表しきれないので near/mid/far で範囲を分ける(旧手順では near/mid が
  // 42〜96、far が 84〜138 のデータ。csv/1st/near と csv/1st/far を手で
  // 入れ替えて回していた)。ファームは 3 組とも常に計算し、どれを使うかは
  // 使う側のコードが決める(距離による自動切り替えは無い)。
  range: [number, number];
}

export const TARGETS: TargetDef[] = [
  { key: "L45_3", group: "l", channel: "L45_3", refDist: 45, range: [0, 999] },
  { key: "L45_2", group: "l", channel: "L45_2", refDist: 45, range: [0, 999] },
  { key: "L45", group: "l", channel: "L45", refDist: 45, range: [0, 999] },
  { key: "R45", group: "r", channel: "R45", refDist: 45, range: [0, 999] },
  { key: "R45_2", group: "r", channel: "R45_2", refDist: 45, range: [0, 999] },
  { key: "R45_3", group: "r", channel: "R45_3", refDist: 45, range: [0, 999] },
  { key: "L90_near", group: "f", channel: "L90", refDist: 45, range: [42, 96] },
  { key: "R90_near", group: "f", channel: "R90", refDist: 45, range: [42, 96] },
  { key: "L90_mid", group: "f", channel: "L90", refDist: 90, range: [42, 96] },
  { key: "R90_mid", group: "f", channel: "R90", refDist: 90, range: [42, 96] },
  { key: "L90_far", group: "f", channel: "L90", refDist: 132, range: [84, 138] },
  { key: "R90_far", group: "f", channel: "R90", refDist: 132, range: [84, 138] },
];

// L90_near ⇔ R90_near のように、左右で同じ用途のキー(範囲を連動させる相手)。
export function pairedKey(k: TargetKey): TargetKey | null {
  const m = /^([LR])(.*)$/.exec(k);
  if (!m) return null;
  const other = `${m[1] === "L" ? "R" : "L"}${m[2]}` as TargetKey;
  return TARGETS.some((t) => t.key === other) ? other : null;
}

export const TARGET_KEYS = TARGETS.map((t) => t.key);

// csv/sensor.sh のコメントにある測定位置。
export const PRESET_POSITIONS: { group: CalibGroup; dist: number }[] = [
  { group: "s", dist: 33 },
  { group: "s", dist: 39 },
  { group: "s", dist: 45 },
  { group: "s", dist: 51 },
  { group: "f", dist: 42 },
  { group: "f", dist: 48 },
  { group: "f", dist: 54 },
  { group: "f", dist: 84 },
  { group: "f", dist: 90 },
  { group: "f", dist: 96 },
  { group: "f", dist: 126 },
  { group: "f", dist: 132 },
  { group: "f", dist: 138 },
];

export type Gain = [number, number]; // [a, b]

// 機体の生値の行を CHANNELS 順の 9 値にする。
//  - テストモード15(dump2): "123, 456, ..."(9 個の整数)
//  - テストモード14(dump1): "sensor: 123, 456, ..."(F を除く 8 個)。F はファームと
//    同じ (L90+R90)/2 で補う。モード14 は実行中も yaml を受信して即反映するので、
//    保存+送信のあと再起動せずに読みを確かめられる。
const DUMP2_RE = /^\s*-?\d+(\s*,\s*-?\d+){8}\s*$/;
const DUMP1_RE = /^\s*sensor:\s*(-?\d+(?:\s*,\s*-?\d+){7})\s*$/;
export function parseDump2Line(line: string): number[] | null {
  if (DUMP2_RE.test(line)) return line.split(",").map((s) => parseInt(s.trim(), 10));
  const m = DUMP1_RE.exec(line);
  if (!m) return null;
  const v = m[1].split(",").map((s) => parseInt(s.trim(), 10));
  return [v[0], v[1], v[2], v[3], Math.trunc((v[0] + v[7]) / 2), v[4], v[5], v[6], v[7]];
}

// テストモード28(MainTask::test_front_sensor_sweep)が出す進行状況の行。
// ファーム側の文言を変えたらここも合わせること。
// 走行はケーブルなしで行うので、実際に画面へ届くのは ready / wired / done /
// dumped くらい(ほかは、つないだまま手順を進めたときにだけ見える)。
//  ready   待機中(生値も出ている)。1 秒ごと。cancelled も同じ扱い
//  wired   ボタンが押されたが、ケーブルがつながっているので走らない
//  place   ボタンが押された。スタート位置に置かれるのを待っている
//  near    前壁が走行距離より近く見えるので走らない(直線の長さが足りない)
//  hand    開始手順に入った。前に手をかざすと走る
//  running 走行中
//  done    ログを送り始める(ケーブルがつながった)
//  dumped  ログを送り終えた
export type SweepFwState = "ready" | "wired" | "place" | "near" | "hand" | "running" | "done" | "dumped";
const SWEEP_STATE_PREFIX: [string, SweepFwState][] = [
  ["sweep: ready", "ready"],
  ["sweep: cancelled", "ready"],
  ["sweep: unplug", "wired"],
  ["sweep: place", "place"],
  ["sweep: front wall too close", "near"],
  ["sweep: wave", "hand"],
  ["sweep: running", "running"],
  ["sweep: done", "done"],
  ["sweep: sending", "done"],
  ["sweep: dumped", "dumped"],
];
export interface SweepFwLine {
  state: SweepFwState;
  d0?: number; // 開始位置の前壁距離(迷路の寸法から。ready / done の行に付く)
  dist?: number; // 走行距離
}
export function parseSweepStateLine(line: string): SweepFwLine | null {
  const t = line.trimStart();
  if (!t.startsWith("sweep:")) return null;
  for (const [prefix, state] of SWEEP_STATE_PREFIX) {
    if (!t.startsWith(prefix)) continue;
    const num = (key: string) => {
      const m = new RegExp(`\\b${key}=(-?[\\d.]+)`).exec(t);
      return m ? Number(m[1]) : undefined;
    };
    return { state, d0: num("d0"), dist: num("dist") ?? num("run") };
  }
  return null;
}

// スイープ開始位置の前壁距離の既定値。ファームが "sweep: ready (… d0=…)" で
// 知らせる値を使うので、これは過去ログを手で取り込むときの初期値。
// 探索の走り出しと同じ 15(offset_start_dist_search) + 90×2 区画 = 195mm 走って
// 区画中央(前壁まで 42mm)で止まるので 195 + 42。
export const SWEEP_DEFAULT_D0 = 237;
// スイープで近づきたい距離(near の下端 42 の少し手前で止める)
export const SWEEP_TARGET_END = 45;

export function channelIndex(ch: Channel): number {
  return CHANNELS.indexOf(ch);
}

export function gainToDist(raw: number, g: Gain): number {
  if (!(raw > 1)) return NaN;
  return g[0] / Math.log(raw) - g[1];
}

// ファームの adjust_b_to_target と同じ逆算: 距離 d を読む生値。
export function distToRaw(d: number, g: Gain): number {
  const denom = d + g[1];
  if (denom <= 0) return NaN;
  return Math.exp(g[0] / denom);
}

export interface FitPoint {
  rowId: string;
  raw: number;
  dist: number;
}

// d0Of: スイープ行の開始位置(そのチャンネル用)。undefined なら行の手入力値。
// skipStatic: 静止点をフィットに使わない(スイープのアンカーとしてだけ使う)。
export function targetPoints(
  t: TargetDef,
  rows: CalibRow[],
  range: [number, number],
  d0Of?: (r: CalibRow, ch: Channel) => number | undefined,
  skipStatic = false,
): FitPoint[] {
  const ci = channelIndex(t.channel);
  const pts: FitPoint[] = [];
  for (const r of rows) {
    if (!r.use || !rowServes(r.group, t.group)) continue;
    if (skipStatic && !r.offsets) continue;
    const d0 = r.offsets ? d0Of?.(r, t.channel) : undefined;
    r.samples.forEach((s, i) => {
      const d = sampleDist(r, i, d0);
      if (d < range[0] || d > range[1]) return;
      const raw = s[ci];
      if (raw > 1) pts.push({ rowId: r.id, raw, dist: d });
    });
  }
  return pts;
}

export interface FitResult {
  gain: Gain;
  n: number; // サンプル数
  nDist: number; // 距離の種類数
  rms: number; // mm
}

// fixedA を渡すと a を固定して b だけ合わせる(センサー付け直し後の簡易補正)。
export function fitGain(pts: FitPoint[], fixedA?: number): FitResult | null {
  const nDist = new Set(pts.map((p) => p.dist)).size;
  if (pts.length === 0) return null;
  const u = pts.map((p) => 1 / Math.log(p.raw));
  const y = pts.map((p) => p.dist);
  let a: number;
  let b: number;
  if (fixedA !== undefined) {
    if (nDist < 1) return null;
    a = fixedA;
    b = u.reduce((s, ui, i) => s + (a * ui - y[i]), 0) / pts.length;
  } else {
    if (nDist < 2) return null;
    const n = pts.length;
    const mu = u.reduce((s, v) => s + v, 0) / n;
    const my = y.reduce((s, v) => s + v, 0) / n;
    let sxx = 0;
    let sxy = 0;
    for (let i = 0; i < n; i++) {
      sxx += (u[i] - mu) ** 2;
      sxy += (u[i] - mu) * (y[i] - my);
    }
    if (sxx <= 0) return null;
    a = sxy / sxx;
    b = a * mu - my; // y = a*u - b
  }
  let ss = 0;
  for (let i = 0; i < pts.length; i++) ss += (a * u[i] - b - y[i]) ** 2;
  return { gain: [a, b], n: pts.length, nDist, rms: Math.sqrt(ss / pts.length) };
}

// ---- スイープの開始位置をアンカー(静止点)から求める ----
//
// スイープが測るのは走行距離に対する生値の変化で、絶対距離は開始位置 D0 次第。
// 既知距離 d_s に置いた静止点(平均生値 r_s)について、スイープ曲線が r_s を
// 読む位置 off(r_s) を求めれば D0 = d_s − off(r_s)。これをセンサーごとに行う。
//  - 開始位置のずれはそのまま D0 に出るので、置き方の精度が要らない。
//  - 傾きがあると左右のセンサーの壁距離が一定量ずつずれるが、L90/R90 を別々に
//    合わせるので吸収される(静止点はスペーサーで面を合わせて置く前提)。
//  - 換算式(a/ln−b)を仮定しない。off(r_s) は r_s 付近の局所直線で読む。
// アンカーが複数あれば中央値を採る(置き損じた 1 点に引きずられない)。
export interface SweepAnchor {
  rowId: string;
  dist: number; // 静止点の距離
  d0: number; // この静止点から求めた開始位置
}
export interface SweepD0 {
  d0: number; // 中央値
  spread: number; // アンカー間のばらつき(最大−最小)
  anchors: SweepAnchor[];
}
const ANCHOR_WINDOW_MM = 4;
const ANCHOR_MIN_SAMPLES = 8;

function median(xs: number[]): number {
  const a = [...xs].sort((x, y) => x - y);
  const m = a.length >> 1;
  return a.length % 2 ? a[m] : (a[m - 1] + a[m]) / 2;
}

export function estimateSweepD0(sweep: CalibRow, rows: CalibRow[], ch: Channel): SweepD0 | null {
  if (!sweep.offsets) return null;
  const ci = channelIndex(ch);
  const us: number[] = [];
  const offs: number[] = [];
  sweep.samples.forEach((smp, i) => {
    const raw = smp[ci];
    if (raw > 1.5) {
      us.push(1 / Math.log(raw));
      offs.push(sweep.offsets![i]);
    }
  });
  if (us.length < ANCHOR_MIN_SAMPLES) return null;
  // ノイズで端が広がるので、外挿を避ける範囲は両端 1% を落として決める
  const sorted = [...us].sort((a, b) => a - b);
  const uLo = sorted[Math.floor(sorted.length * 0.01)];
  const uHi = sorted[Math.ceil(sorted.length * 0.99) - 1];

  const anchors: SweepAnchor[] = [];
  for (const r of rows) {
    if (r.offsets || !r.use || r.group !== "f" || !r.samples.length) continue;
    const raws = r.samples.map((smp) => smp[ci]).filter((v) => v > 1.5);
    if (!raws.length) continue;
    const uS = 1 / Math.log(mean(raws));
    if (uS < uLo || uS > uHi) continue; // スイープが通っていない距離
    // u が近い順に 15 点とり、その走行距離の中央値を窓の中心にする
    const near = us
      .map((u, i) => [Math.abs(u - uS), i] as const)
      .sort((a, b) => a[0] - b[0])
      .slice(0, 15)
      .map(([, i]) => offs[i]);
    const center = median(near);
    let n = 0;
    let su = 0;
    let so = 0;
    for (let i = 0; i < us.length; i++) {
      if (Math.abs(offs[i] - center) > ANCHOR_WINDOW_MM) continue;
      n++;
      su += us[i];
      so += offs[i];
    }
    if (n < ANCHOR_MIN_SAMPLES) continue;
    const mu = su / n;
    const mo = so / n;
    let sxx = 0;
    let sxy = 0;
    for (let i = 0; i < us.length; i++) {
      if (Math.abs(offs[i] - center) > ANCHOR_WINDOW_MM) continue;
      sxx += (us[i] - mu) ** 2;
      sxy += (us[i] - mu) * (offs[i] - mo);
    }
    if (sxx <= 0) continue;
    const offS = mo + (sxy / sxx) * (uS - mu);
    anchors.push({ rowId: r.id, dist: r.dist, d0: r.dist - offS });
  }
  if (!anchors.length) return null;
  const d0s = anchors.map((a) => a.d0);
  return { d0: median(d0s), spread: Math.max(...d0s) - Math.min(...d0s), anchors };
}

export function mean(xs: number[]): number {
  return xs.length ? xs.reduce((s, v) => s + v, 0) / xs.length : NaN;
}

export function std(xs: number[]): number {
  if (xs.length < 2) return 0;
  const m = mean(xs);
  return Math.sqrt(xs.reduce((s, v) => s + (v - m) ** 2, 0) / (xs.length - 1));
}

// その壁の置き方で意味のあるチャンネル(安定度チェック用)。
export function groupChannels(g: CalibGroup): Channel[] {
  if (g === "s") return ["L45_3", "L45_2", "L45", "R45", "R45_2", "R45_3"];
  if (g === "l") return ["L45_3", "L45_2", "L45"];
  if (g === "r") return ["R45", "R45_2", "R45_3"];
  return ["L90", "R90"];
}

// 旧 csv と同じ形式: ヘッダー + "dist,v1, v2, ..."。dists を渡すと行ごとに
// 距離を変えて書く(スイープ行。旧 pyplot.py も行ごとの dist を読むのでそのまま使える)。
export const CSV_HEADER = `dist,${CHANNELS.join(",")}`;
export function rowToCsv(dist: number, samples: number[][], dists?: number[]): string {
  const d = (i: number) => (dists ? +dists[i].toFixed(2) : dist);
  return [CSV_HEADER, ...samples.map((s, i) => `${d(i)},${s.join(", ")}`)].join("\n") + "\n";
}

export function parseCsv(text: string): { dist: number; dists: number[]; samples: number[][] } | null {
  const dists: number[] = [];
  const samples: number[][] = [];
  for (const line of text.split(/\r?\n/)) {
    const cols = line.split(",").map((s) => s.trim());
    if (cols.length !== 10) continue;
    const nums = cols.map(Number);
    if (nums.some((v) => !Number.isFinite(v))) continue; // ヘッダーやゴミ行
    dists.push(nums[0]);
    samples.push(nums.slice(1));
  }
  return samples.length ? { dist: dists[dists.length - 1], dists, samples } : null;
}

// テストモード28(前壁スイープ)のログ CSV から動いている区間を取り出す。
// 前進(sensor_sweep_dir=1、背面を壁に当てて前壁へ近づく)は STRAIGHT(1)、
// 後退(-1)は BACK_STRAIGHT(5)の行を使う。dist 列はモーション受信で 0 に戻る
// 積算距離(前進で正、後退で負)なので、どちらも offset = −dist とすれば
// 「前壁距離 = D0 + offset」になる(D0 = 開始位置の前壁距離)。
// 開始前の静止区間(ログ開始〜受信)は D0 に点が偏るので使わない。
// 返り値 null = 使える区間が無い。looksLikeSweep = 1 方向の低速・短距離の
// 動きだけで、前壁に近づいて前センサーが大きく読んでいるログ(直進テストや
// 探索のログを自動取り込みで拾わないための判定)。過去ログ 1927 本で、拾うのは
// 実機のスイープ 2 本(20260928_2229xx)だけ。
export const MOTION_STRAIGHT = 1;
export const MOTION_BACK_STRAIGHT = 5;
const MOTION_IDLE = new Set([0, 7, 17]); // NONE / READY / SENSING_DUMP
const SWEEP_MAX_V = 3000; // mm/s(速度は system.yaml の test.v_max で決まる)
const SWEEP_MAX_SPAN = 400; // mm(4 区画ぶんまで)
const SWEEP_MIN_SPAN = 30; // mm
const SWEEP_MIN_SAMPLES = 150;
const SWEEP_RESET_MM = 5; // dist がこれ以上戻ったら、次の go_straight に入った
const SWEEP_MAX_HEADING_DEG = 5; // これ以上向きが変わるログは直進ではない
const SWEEP_MIN_PEAK_RAW = 200;
const SWEEP_MIN_RISE = 3; // 両端の生値の比 // 前壁に近づいていれば L90/R90 の生値はこれより大きくなる
const LOG_COLUMN_OF: Record<Channel, string> = {
  L90: "left90",
  L45_3: "left45_3",
  L45_2: "left45_2",
  L45: "left45",
  F: "front",
  R45: "right45",
  R45_2: "right45_2",
  R45_3: "right45_3",
  R90: "right90",
};
export interface SweepLog {
  offsets: number[];
  samples: number[][];
  looksLikeSweep: boolean;
  // looksLikeSweep が false の理由(画面に出す。黙って無視すると、走らせたのに
  // 何も起きないように見える)
  reject: string | null;
  forward: boolean;
  pose: SweepPose;
}

// 45 度センサーが前壁を見始める手前までを姿勢の推定に使う。実ログ
// (20260927_095027.csv)では前壁まで約 74mm から読みが縮み始めるので、終点
// (前壁まで 45mm 前後)から 45mm 以上手前の区間だけを見る。
const POSE_ZONE_FROM_END_MM = 45;
// 測りたい範囲(前壁から 〜138mm)の外は見ない。走り出しは壁制御が姿勢を
// 直している途中なので、ここを混ぜると傾きが大きく出る。
const POSE_ZONE_MAX_FROM_END_MM = 100;
const POSE_WALL_MIN = 20;
const POSE_WALL_MAX = 60;

export function parseSweepLog(text: string): SweepLog | null {
  const lines = text.split(/\r?\n/);
  const header = lines[0]?.split(",") ?? [];
  const col = (name: string) => header.indexOf(name);
  const iState = col("motion_state");
  const iDist = col("dist");
  const iV = col("ideal_v");
  const iVc = col("v_c");
  const iIndex = col("index");
  const iAng = col("ang");
  const iL45d = col("left45_d");
  const iR45d = col("right45_d");
  const iDutySen = col("duty_sen");
  const iCh = CHANNELS.map((c) => col(LOG_COLUMN_OF[c]));
  if (iState < 0 || iDist < 0 || iCh[0] < 0 || iCh[8] < 0) return null;
  // 先に出てきた方の向きを採用する
  let moving = -1;
  const offsets: number[] = [];
  const samples: number[][] = [];
  let pure = true;
  let maxV = 0;
  const angs: number[] = [];
  const l45d: number[] = [];
  const r45d: number[] = [];
  let wallCtrl = false;
  // 走行は go_straight を 2 本つないでいる(壁制御あり → なし)。dist は 2 本目の
  // 頭で 0 に戻るので、戻った所で直前までの距離を足してつなぎ直す。戻る前後の
  // 行の間に進んだぶんは速度 × 時間で補う(400mm/s で 0.4mm)。
  let base = 0;
  let prevDist: number | null = null;
  let prevIndex = 0;
  for (let k = 1; k < lines.length; k++) {
    const cols = lines[k].split(",");
    if (cols.length < header.length) continue;
    const state = Number(cols[iState]);
    if (moving < 0 && (state === MOTION_STRAIGHT || state === MOTION_BACK_STRAIGHT)) moving = state;
    if (state !== moving) {
      if (!MOTION_IDLE.has(state)) pure = false;
      continue;
    }
    const dist = Number(cols[iDist]);
    if (!Number.isFinite(dist)) continue;
    const index = iIndex >= 0 ? Number(cols[iIndex]) || 0 : k;
    if (prevDist !== null && Math.abs(dist) < Math.abs(prevDist) - SWEEP_RESET_MM) {
      const dtMs = Math.min(Math.max(index - prevIndex, 1), 5);
      const step = iVc >= 0 ? ((Number(cols[iVc]) || 0) * dtMs) / 1000 : 0;
      base += prevDist + step - dist;
    }
    prevDist = dist;
    prevIndex = index;
    const off = -(base + dist);
    if (iV >= 0) maxV = Math.max(maxV, Math.abs(Number(cols[iV]) || 0));
    offsets.push(off);
    samples.push(iCh.map((i) => (i >= 0 ? Number(cols[i]) || 0 : 0)));
    angs.push(iAng >= 0 ? Number(cols[iAng]) || 0 : 0);
    l45d.push(iL45d >= 0 ? Number(cols[iL45d]) || 0 : 0);
    r45d.push(iR45d >= 0 ? Number(cols[iR45d]) || 0 : 0);
    if (iDutySen >= 0 && Math.abs(Number(cols[iDutySen]) || 0) > 1e-4) wallCtrl = true;
  }
  if (!samples.length) return null;
  let lo = Infinity;
  let hi = -Infinity;
  for (const o of offsets) {
    if (o < lo) lo = o;
    if (o > hi) hi = o;
  }
  let peak = 0;
  let sane = true; // 12bit ADC の範囲を外れる値があれば壊れたログ
  for (const s of samples) {
    peak = Math.max(peak, Math.min(s[0], s[8])); // L90 と R90 の両方
    if (s[0] > 4095 || s[8] > 4095 || s[0] < 0 || s[8] < 0) sane = false;
  }
  // 前壁に近づいて(または離れて)いれば、両端で生値が大きく違う
  const endMean = (ci: number, from: number) => {
    let sum = 0;
    for (let i = from; i < from + 20; i++) sum += samples[i][ci];
    return sum / 20;
  };
  const rises =
    samples.length >= 40 &&
    [0, 8].every((ci) => {
      const a = endMean(ci, 0);
      const b = endMean(ci, samples.length - 20);
      return Math.max(a, b) >= SWEEP_MIN_RISE * Math.max(1, Math.min(a, b));
    });
  let angLo = Infinity;
  let angHi = -Infinity;
  for (const a of angs) {
    if (a < angLo) angLo = a;
    if (a > angHi) angHi = a;
  }
  const span = hi - lo;
  const reject = !pure
    ? "直進以外の動きが入っています"
    : !sane
      ? "前センサーの値が壊れています(ADC の範囲外)"
      : span < SWEEP_MIN_SPAN || span > SWEEP_MAX_SPAN
        ? `走行距離が ${span.toFixed(0)}mm です(${SWEEP_MIN_SPAN}〜${SWEEP_MAX_SPAN}mm を想定)`
        : samples.length < SWEEP_MIN_SAMPLES
          ? `点数が ${samples.length} しかありません`
          : angHi - angLo > SWEEP_MAX_HEADING_DEG
            ? `走行中に向きが ${(angHi - angLo).toFixed(1)}° 変わっています`
            : maxV > SWEEP_MAX_V
              ? `速度が ${maxV.toFixed(0)}mm/s です`
              : peak < SWEEP_MIN_PEAK_RAW || !rises
                ? `前壁へ近づいても前センサー(L90/R90)の生値が増えていません(最大 ${peak})。前壁が無い、またはセンサーが消えています`
                : null;
  const looksLikeSweep = reject === null;
  const forward = moving === MOTION_STRAIGHT;
  // 姿勢: 測りたい範囲だけを見る(終点 = offset 最小 = 前壁に最も近い点)
  let zLo = Infinity;
  let zHi = -Infinity;
  let latSum = 0;
  let latN = 0;
  for (let i = 0; i < offsets.length; i++) {
    const fromEnd = offsets[i] - lo;
    if (fromEnd > POSE_ZONE_MAX_FROM_END_MM) continue;
    if (angs[i] < zLo) zLo = angs[i];
    if (angs[i] > zHi) zHi = angs[i];
    if (fromEnd < POSE_ZONE_FROM_END_MM) continue;
    const l = l45d[i];
    const r = r45d[i];
    if (l < POSE_WALL_MIN || l > POSE_WALL_MAX || r < POSE_WALL_MIN || r > POSE_WALL_MAX) continue;
    latSum += (r - l) / 2;
    latN++;
  }
  const pose: SweepPose = {
    vMax: maxV,
    headingDriftDeg: zHi >= zLo ? zHi - zLo : 0,
    latOffsetMm: latN >= 30 ? latSum / latN : null,
    wallCtrl,
  };
  return { offsets, samples, looksLikeSweep, reject, forward, pose };
}
