// Client-side ports of analyze_sensor_drop.py / analyze_motion_state_transitions.py.
// Both scripts scan a raw (unsorted, as-logged) CSV row sequence for
// motion_state transitions and report index/value info; here we additionally
// resolve each event to the row's (x, y) so it can be drawn as a marker on
// the trajectory plot. Row order must match the scripts' (CSV file order,
// not timestamp-sorted) - use the raw parseCsv() output, not trajectory.ts's
// sorted rows.

import { projectSensorPoint, type TrajectoryPoint } from "@/lib/trajectory";

// Client-side port of tools/param_tuner/wall_off_edge_check.py, extended to
// place markers on the spatial trajectory plot and value-vs-index chart
// (the .py only prints numbers). See that script's docstring for the
// rationale (WallOffController's current detection is a level threshold on
// an absolute sensor value, which lags by however far the baseline sits
// below the threshold - worse the closer the robot is hugging the wall).

export interface AnalysisEvent {
  x: number;
  y: number;
  // "sensor": (x,y) is already the projected wall-contact point (world coords,
  // POS_OFFSET_X baked in, like TrajectoryData.leftWallPoints/rightWallPoints).
  // "robot": (x,y) is the raw logged position, needs the same offset the
  // trajectory plot applies to robot-position points at draw time.
  anchored: "robot" | "sensor";
  kind:
    | "drop"
    | "rise"
    | "state-start"
    | "state-end"
    // The 45deg reading the state-start/state-end row was referencing,
    // projected onto its wall-contact point (so the plot shows *where* the
    // robot thought the wall was at the moment the state changed, not just
    // where the robot itself was).
    | "state-start-sensor"
    | "state-end-sensor"
    | "trough"
    | "trough-rise"
    // wall_off_edge_check.py 相当: WallOffController の壁切れ判定を
    // レベル判定(現行)/気づいた点(Layer1)/逆算エッジ(Layer2)の3段で可視化する。
    // 現行方式がどれだけ遅れて発火しているかを、走行位置として直接比較できる。
    | "wall-off-anchor"
    | "wall-off-anchor-sensor"
    | "wall-off-actual"
    | "wall-off-actual-sensor"
    | "wall-off-arm"
    | "wall-off-edge"
    | "wall-off-edge-sensor";
  label: string;
  // Companion anchor for a leader line: raw logged robot (x, y) of the same
  // row, so a sensor-anchored marker can be drawn tied to the robot marker it
  // belongs to. Needs the same POS_OFFSET_X a "robot"-anchored point gets.
  linkX?: number;
  linkY?: number;
  // Set only for column-specific events (drop/rise/trough/trough-rise), so a
  // value-vs-index time-series chart can place the same marker on its own
  // axes instead of the spatial (x, y) ones above.
  column?: string;
  seriesIndex?: number;
  seriesValue?: number;
}

// Maps a raw CSV row (by object identity) to the TrajectoryPoint built from
// it, so a drop/rise event tied to left45_d/right45_d can be re-anchored to
// the sensor's projected wall position instead of the robot's own (x, y).
// trajectory.ts's buildTrajectoryData() reuses row objects verbatim as
// TrajectoryPoint.raw, so identity lookup works as long as both are derived
// from the same parseCsv() output.
export type RowPointMap = Map<Record<string, number>, TrajectoryPoint>;

function sensorSideSign(column: string): 1 | -1 | null {
  if (column === "left45_d") return 1;
  if (column === "right45_d") return -1;
  return null;
}

function resolvePos(
  row: Record<string, number>,
  column: string,
  pointByRow?: RowPointMap
): { x: number; y: number; anchored: "robot" | "sensor" } {
  const sideSign = sensorSideSign(column);
  const tp = pointByRow?.get(row);
  if (sideSign !== null && tp) {
    const wp = projectSensorPoint(tp, column as "left45_d" | "right45_d", sideSign);
    if (wp) return { x: wp.x, y: wp.y, anchored: "sensor" };
  }
  return { x: row.x, y: row.y, anchored: "robot" };
}

function asNum(row: Record<string, number>, key: string): number | undefined {
  const v = row[key];
  return v !== undefined && Number.isFinite(v) ? v : undefined;
}

function hasXY(row: Record<string, number>): boolean {
  return asNum(row, "x") !== undefined && asNum(row, "y") !== undefined;
}

function fmt(row: Record<string, number>, key: string): string {
  const v = asNum(row, key);
  return v !== undefined ? v.toFixed(1) : "?";
}

function findMotionStateStarts(rows: Record<string, number>[], motionState: number): number[] {
  const starts: number[] = [];
  let prev: number | undefined;
  for (let i = 0; i < rows.length; i++) {
    const ms = asNum(rows[i], "motion_state");
    if (ms === motionState && prev !== motionState) starts.push(i);
    prev = ms;
  }
  return starts;
}

export interface SensorDropOptions {
  motionState: number;
  low: number;
  high: number;
  columns: string[];
  pointByRow?: RowPointMap;
}

// Port of analyze_sensor_drop.py: main.
export function computeSensorDropEvents(rows: Record<string, number>[], opts: SensorDropOptions): AnalysisEvent[] {
  const starts = findMotionStateStarts(rows, opts.motionState);
  const events: AnalysisEvent[] = [];

  for (const col of opts.columns) {
    for (const s of starts) {
      const startIndex = fmt(rows[s], "index");

      let dropI: number | null = null;
      for (let i = s; i < rows.length; i++) {
        const v = asNum(rows[i], col);
        if (v !== undefined && v <= opts.low) {
          dropI = i;
          break;
        }
      }
      if (dropI === null) continue;
      const dropRow = rows[dropI];
      if (hasXY(dropRow)) {
        const pos = resolvePos(dropRow, col, opts.pointByRow);
        events.push({
          ...pos,
          kind: "drop",
          label: `${col} <=${opts.low} start=${startIndex} idx=${fmt(dropRow, "index")} val=${fmt(dropRow, col)}`,
          column: col,
          seriesIndex: asNum(dropRow, "index"),
          seriesValue: asNum(dropRow, col),
        });
      }

      let riseI: number | null = null;
      for (let i = dropI + 1; i < rows.length; i++) {
        const v = asNum(rows[i], col);
        if (v !== undefined && v >= opts.high) {
          riseI = i;
          break;
        }
      }
      if (riseI === null) continue;
      const riseRow = rows[riseI];
      if (hasXY(riseRow)) {
        const diff = asNum(riseRow, "index") !== undefined && asNum(dropRow, "index") !== undefined
          ? (riseRow.index - dropRow.index).toFixed(0)
          : "?";
        const pos = resolvePos(riseRow, col, opts.pointByRow);
        events.push({
          ...pos,
          kind: "rise",
          label: `${col} >=${opts.high} start=${startIndex} idx=${fmt(riseRow, "index")} val=${fmt(riseRow, col)} diff=${diff}`,
          column: col,
          seriesIndex: asNum(riseRow, "index"),
          seriesValue: asNum(riseRow, col),
        });
      }
    }
  }
  return events;
}

// Port of analyze_sensor_trough.py.

function asSensorNum(row: Record<string, number>, key: string): number | undefined {
  const v = asNum(row, key);
  return v !== undefined && v > 0 ? v : undefined;
}

function findMotionStateEnds(rows: Record<string, number>[], motionState: number): number[] {
  const ends: number[] = [];
  let prev: number | undefined;
  for (let i = 0; i < rows.length; i++) {
    const ms = asNum(rows[i], "motion_state");
    if (prev === motionState && ms !== undefined && ms !== motionState) ends.push(i);
    prev = ms;
  }
  return ends;
}

interface TroughResult {
  troughRow: number;
  riseRow: number | null;
}

// Rejects single-sample outlier spikes before zigzag detection runs.
// left90_d/right90_d read larger absolute distances than left45_d/right45_d,
// so their single-frame read noise is also larger in absolute mm - large
// enough to fool the eps-hysteresis zigzag below into treating an isolated
// glitchy sample as a genuine reversal. A median-of-`window` filter removes
// those without blurring out a real, sustained trend reversal.
function medianFilter(values: number[], window: number): number[] {
  if (window <= 1) return values.slice();
  const half = Math.floor(window / 2);
  const out: number[] = [];
  for (let k = 0; k < values.length; k++) {
    const seg = values.slice(Math.max(0, k - half), Math.min(values.length, k + half + 1)).slice().sort((a, b) => a - b);
    out.push(seg[Math.floor(seg.length / 2)]);
  }
  return out;
}

// Zigzag trough/rise detector, matching analyze_sensor_trough.py::find_troughs:
// hunts the column's running minimum ("trough" mode) until it climbs back up
// by more than `eps`, then switches to hunting the running maximum ("peak"
// mode) until it falls by more than `eps`, and so on - repeating within the
// [start, window-exit) span instead of stopping at the first cycle. Reversal
// decisions run on the median-filtered series; returned indices point at the
// original (unfiltered) rows.
function findTroughs(
  rows: Record<string, number>[],
  col: string,
  start: number,
  eps: number,
  inStates?: Set<number>,
  medianWindow = 3
): TroughResult[] {
  const n = rows.length;
  const inWindow = (i: number) => (inStates ? inStates.has(asNum(rows[i], "motion_state") ?? NaN) : true);

  const idxs: number[] = [];
  const raws: number[] = [];
  let i = start;
  while (i < n && inWindow(i)) {
    const v = asSensorNum(rows[i], col);
    if (v !== undefined) {
      idxs.push(i);
      raws.push(v);
    }
    i++;
  }
  if (raws.length === 0) return [];

  const smoothed = medianFilter(raws, medianWindow);

  const results: TroughResult[] = [];
  let mode: "trough" | "peak" = "trough";
  let cycleStartV = smoothed[0];
  let extremeK = 0;
  let extremeV = smoothed[0];
  for (let k = 1; k < smoothed.length; k++) {
    const v = smoothed[k];
    if (mode === "trough") {
      if (v < extremeV) {
        extremeK = k;
        extremeV = v;
      } else if (v > extremeV + eps) {
        if (extremeV <= cycleStartV - eps) results.push({ troughRow: idxs[extremeK], riseRow: idxs[k] });
        mode = "peak";
        cycleStartV = extremeV;
        extremeK = k;
        extremeV = v;
      }
    } else {
      if (v > extremeV) {
        extremeK = k;
        extremeV = v;
      } else if (v < extremeV - eps) {
        mode = "trough";
        cycleStartV = extremeV;
        extremeK = k;
        extremeV = v;
      }
    }
  }
  if (mode === "trough" && extremeV <= cycleStartV - eps) results.push({ troughRow: idxs[extremeK], riseRow: null });

  return results;
}

export interface SensorTroughOptions {
  motionState: number;
  states: number[];
  eps: number;
  medianWindow?: number;
  columns: string[];
  pointByRow?: RowPointMap;
}

export function computeSensorTroughEvents(
  rows: Record<string, number>[],
  opts: SensorTroughOptions
): AnalysisEvent[] {
  const ends = findMotionStateEnds(rows, opts.motionState);
  const inStates = new Set(opts.states);
  const events: AnalysisEvent[] = [];

  for (const col of opts.columns) {
    for (const e of ends) {
      const endIndex = fmt(rows[e], "index");
      const troughs = findTroughs(rows, col, e, opts.eps, inStates, opts.medianWindow);
      troughs.forEach((t, n) => {
        const troughRow = rows[t.troughRow];
        if (hasXY(troughRow)) {
          const pos = resolvePos(troughRow, col, opts.pointByRow);
          events.push({
            ...pos,
            kind: "trough",
            label: `${col} [${n + 1}] 極小 end=${endIndex} idx=${fmt(troughRow, "index")} val=${fmt(troughRow, col)}`,
            column: col,
            seriesIndex: asNum(troughRow, "index"),
            seriesValue: asNum(troughRow, col),
          });
        }
        if (t.riseRow === null) return;
        const riseRow = rows[t.riseRow];
        if (hasXY(riseRow)) {
          const diff =
            asNum(riseRow, "index") !== undefined && asNum(troughRow, "index") !== undefined
              ? (riseRow.index - troughRow.index).toFixed(0)
              : "?";
          const pos = resolvePos(riseRow, col, opts.pointByRow);
          events.push({
            ...pos,
            kind: "trough-rise",
            label: `${col} [${n + 1}] 上昇開始 end=${endIndex} idx=${fmt(riseRow, "index")} val=${fmt(riseRow, col)} diff=${diff}`,
            column: col,
            seriesIndex: asNum(riseRow, "index"),
            seriesValue: asNum(riseRow, col),
          });
        }
      });
    }
  }
  return events;
}

function findBlocks(rows: Record<string, number>[], state: number): Array<[number, number, number | null]> {
  const blocks: Array<[number, number, number | null]> = [];
  let blockStart: number | null = null;
  let prev: number | undefined;
  for (let i = 0; i < rows.length; i++) {
    const ms = asNum(rows[i], "motion_state");
    const inState = ms === state;
    const wasIn = prev === state;
    if (inState && !wasIn) blockStart = i;
    if (wasIn && !inState) blocks.push([blockStart as number, i - 1, ms ?? null]);
    prev = ms;
  }
  if (blockStart !== null && prev === state) blocks.push([blockStart, rows.length - 1, null]);
  return blocks;
}

export interface MotionTransitionOptions {
  states: number[];
  columns: string[];
  // Needed to project the row's 45deg readings onto their wall-contact
  // points; without it only the robot-position markers are emitted.
  pointByRow?: RowPointMap;
}

// The transition columns that projectSensorPoint() knows the geometry for.
// left45_2_d / left45_3_d etc. are alternate LED patterns of the same
// physical sensor and have no separate projection, so they stay text-only.
const PROJECTABLE_45 = ["left45_d", "right45_d"] as const;

// Port of analyze_motion_state_transitions.py: main.
export function computeMotionTransitionEvents(
  rows: Record<string, number>[],
  opts: MotionTransitionOptions
): AnalysisEvent[] {
  const events: AnalysisEvent[] = [];

  const projectable = (row: Record<string, number>, col: string) =>
    opts.pointByRow ? resolvePos(row, col, opts.pointByRow).anchored === "sensor" : false;

  const describe = (row: Record<string, number>) =>
    opts.columns
      .filter((c) => asNum(row, c) !== undefined)
      // Flag the readings that are outside projectSensorPoint()'s valid band
      // so a missing wall marker reads as "out of range", not "lost".
      .map((c) => {
        const outOfBand =
          (PROJECTABLE_45 as readonly string[]).includes(c) && !projectable(row, c) ? "(投影外)" : "";
        return `${c}=${fmt(row, c)}${outOfBand}`;
      })
      .join(", ");

  // Emits the robot-position marker plus, for each projectable 45deg column,
  // a wall-contact marker tied back to it by a leader line.
  const pushEvent = (row: Record<string, number>, kind: "state-start" | "state-end", label: string) => {
    events.push({ x: row.x, y: row.y, anchored: "robot", kind, label });
    if (!opts.pointByRow) return;
    for (const col of PROJECTABLE_45) {
      if (!opts.columns.includes(col)) continue;
      const pos = resolvePos(row, col, opts.pointByRow);
      if (pos.anchored !== "sensor") continue;
      events.push({
        ...pos,
        kind: kind === "state-start" ? "state-start-sensor" : "state-end-sensor",
        label: `${label.split(" | ")[0]} ${col}=${fmt(row, col)} → 壁(${pos.x.toFixed(1)}, ${pos.y.toFixed(1)})`,
        linkX: row.x,
        linkY: row.y,
        column: col,
        seriesIndex: asNum(row, "index"),
        seriesValue: asNum(row, col),
      });
    }
  };

  for (const state of opts.states) {
    for (const [bStart, bEnd, nextMs] of findBlocks(rows, state)) {
      const startRow = rows[bStart];
      if (hasXY(startRow)) {
        pushEvent(
          startRow,
          "state-start",
          `state=${state} 開始 idx=${fmt(startRow, "index")} | ${describe(startRow)}`
        );
      }
      if (nextMs !== null && bEnd + 1 < rows.length) {
        const endRow = rows[bEnd + 1];
        if (hasXY(endRow)) {
          pushEvent(
            endRow,
            "state-end",
            `state=${state} 終了(->${nextMs}) idx=${fmt(endRow, "index")} | ${describe(endRow)}`
          );
        }
      }
    }
  }
  return events;
}

// --- wall_off_edge_check.py port -------------------------------------------
//
// 2026-09-05 の実測(n=4反復x2側)で判明した重要な注意: Layer1(arm)/Layer2
// (edge、直線外挿)は個々のログではr2=0.93〜0.99と当てはまり良く見えても、
// 複数試行間のばらつきは現行方式(actual)よりむしろ大きい(std比1.6〜3.7倍)。
// x_edge=(baseline-切片)/傾き の計算が傾きの推定誤差で割るため、傾きが
// センサー読みの立ち上がり形状(姿勢/進入角で変わる)の試行間ばらつきをその
// まま位置誤差に増幅してしまうのが原因と見られる。つまりこの2つは「現行
// より真の切れ目に近い推定値」ではなく、あくまで参考情報 - ラベルはそう
// 読めるようにしてある。詳しい実測結果は wall_off_edge_check.py の docstring
// 参照。

// もう一つのパターン: WALL_OFF開始時点でまだ壁が見えていない(遠い距離から
// 始まる)区間もある。この場合センサ値は遠→接近→最も近づく(トラフ)→
// また離れる、という形になり、区間先頭をベースラインにすると意味を成さない。
// findBaselineAnchor()でセグメント内の(平滑化後の)最小点を探し、そこを
// 「壁に最も寄った瞬間」の基準点として全ての計算(baseline/arm/fitの探索
// 開始点)をそこからにする。開始時点で既に可視なパターンでは、この最小点は
// ほぼ区間先頭に一致するので同じロジックで両パターンをカバーできる。

export interface WallOffEdgeOptions {
  // e.g. [6, 13] to overlay the straight (WALL_OFF) and diagonal
  // (WALL_OFF_DIA) wall-off segments in the same pass.
  motionStates: number[];
  baselineN?: number; // baseline = median of the N samples starting at the anchor (trough)
  armDelta?: number; // [mm] "noticed" once the sensor rises this far above baseline
  fitLo?: number; // [mm] fit window lower bound, relative to baseline - state 6 (straight)
  fitHi?: number; // [mm] fit window upper bound, relative to baseline - state 6 (straight)
  // WALL_OFF_DIA (state 13) runs at ~5x the speed over a much longer segment
  // (diagonal-cell pitch, not a single cell), so the same rise covers a much
  // wider mm-of-value band - fitLo/fitHi above are tuned from state-6 data
  // and are almost always too narrow for it (real deltas at trigger observed
  // 6.8-29.3mm across 6 real DIA logs, vs ~1-6mm for state 6). Defaults to a
  // separate, wider window unless overridden.
  fitLoDia?: number;
  fitHiDia?: number;
  minFitN?: number;
  medianWindow?: number; // smoothing window used only to locate the anchor (trough)
  existTh?: number; // [mm] a side's baseline must be below this to count as "a wall was seen"
  minDelta?: number; // [mm] a side's rise must exceed this to count as a real event, not noise
  pointByRow?: RowPointMap;
}

const WALL_OFF_DEFAULTS = {
  baselineN: 5,
  armDelta: 1.0,
  fitLo: 1.0,
  fitHi: 5.0,
  fitLoDia: 2.0,
  fitHiDia: 20.0,
  minFitN: 3,
  medianWindow: 3,
  existTh: 70.0,
  minDelta: 1.0,
};

function median(values: number[]): number {
  const s = values.slice().sort((a, b) => a - b);
  const mid = Math.floor(s.length / 2);
  return s.length % 2 === 1 ? s[mid] : (s[mid - 1] + s[mid]) / 2;
}

// Same slope/intercept/r2 as wall_off_edge_check.py's reg().
function linreg(x: number[], y: number[]): { slope: number; intercept: number; r2: number } {
  const n = x.length;
  const mx = x.reduce((a, b) => a + b, 0) / n;
  const my = y.reduce((a, b) => a + b, 0) / n;
  let sxy = 0;
  let sxx = 0;
  let syy = 0;
  for (let i = 0; i < n; i++) {
    const dx = x[i] - mx;
    const dy = y[i] - my;
    sxy += dx * dy;
    sxx += dx * dx;
    syy += dy * dy;
  }
  const slope = sxx !== 0 ? sxy / sxx : 0;
  const intercept = my - slope * mx;
  const r2 = sxx > 0 && syy > 0 ? (sxy * sxy) / (sxx * syy) : NaN;
  return { slope, intercept, r2 };
}

// Earliest index of the (smoothed) minimum - "the moment the robot got
// closest to the wall". In a segment that's visible from the start, this is
// ~index 0 (the value only ever rises); in one that isn't, the value falls
// from a large "far" reading to a trough before rising, and this finds that
// trough. Matches wall_off_edge_check.py's find_anchor().
function findBaselineAnchor(ys: number[], medianWindow: number): number {
  const smoothed = medianFilter(ys, medianWindow);
  let idx = 0;
  let minV = smoothed[0];
  for (let i = 1; i < smoothed.length; i++) {
    if (smoothed[i] < minV) {
      minV = smoothed[i];
      idx = i;
    }
  }
  return idx;
}

interface SideBaseline {
  anchorI: number;
  baseline: number;
  delta: number; // last sample minus baseline - the size of the recede signal
}

function analyzeSideBaseline(ys: number[], baselineN: number, medianWindow: number): SideBaseline {
  const anchorI = findBaselineAnchor(ys, medianWindow);
  const windowEnd = Math.min(anchorI + baselineN, ys.length);
  const baseline = median(ys.slice(anchorI, windowEnd));
  return { anchorI, baseline, delta: ys[ys.length - 1] - baseline };
}

// The side with the larger recede signal (each measured from its own
// anchor/baseline, so this works the same whether the wall was visible from
// the start or approached from far away). Matches wall_off_edge_check.py's
// pick_side().
//
// Returns side: null when NEITHER side ever actually saw a wall (baseline
// stayed at/near sensor_range_max, i.e. "far" the whole segment) or the
// rise is too small to be anything but noise. Earlier this function always
// picked a side regardless, so a WALL_OFF segment with no real wall event at
// all (motion_state exits via some unrelated path - front correction, a
// distance timeout, ...) still got confident-looking anchor/actual/edge
// markers built from pure noise. Reported from the Param Console GUI
// 2026-09-05.
function pickWallOffSide(
  segRows: Record<string, number>[],
  baselineN: number,
  medianWindow: number,
  existTh: number,
  minDelta: number
): { side: "left" | "right" | null; left: SideBaseline; right: SideBaseline } {
  const left = analyzeSideBaseline(
    segRows.map((r) => r.left45_d),
    baselineN,
    medianWindow
  );
  const right = analyzeSideBaseline(
    segRows.map((r) => r.right45_d),
    baselineN,
    medianWindow
  );
  const valid = (s: SideBaseline) => s.baseline < existTh && s.delta > minDelta;
  const leftOk = valid(left);
  const rightOk = valid(right);
  let side: "left" | "right" | null;
  if (!leftOk && !rightOk) side = null;
  else if (leftOk && rightOk) side = right.delta > left.delta ? "right" : "left";
  else side = rightOk ? "right" : "left";
  return { side, left, right };
}

// Interpolates a synthetic TrajectoryPoint at `target` (in the segment's
// "dist" units) from the two bracketing rows' already-computed trajectory
// points, so the model's back-projected edge - which generally falls between
// logged samples, not on one - can still be drawn at a specific (x, y).
function interpolateAt(
  segRows: Record<string, number>[],
  xs: number[],
  target: number,
  pointByRow: RowPointMap
): { x: number; y: number; angleCorrected: number; index?: number; extrapolated: boolean } | null {
  const n = xs.length;
  if (n < 2) return null;
  let i = 0;
  while (i < n - 2 && xs[i + 1] < target) i++;
  const p0 = pointByRow.get(segRows[i]);
  const p1 = pointByRow.get(segRows[i + 1]);
  if (!p0 || !p1) return null;
  const span = xs[i + 1] - xs[i];
  const t = span !== 0 ? (target - xs[i]) / span : 0;
  const lerp = (a: number, b: number) => a + (b - a) * t;
  const idx0 = asNum(segRows[i], "index");
  const idx1 = asNum(segRows[i + 1], "index");
  return {
    x: lerp(p0.x, p1.x),
    y: lerp(p0.y, p1.y),
    angleCorrected: lerp(p0.angleCorrected, p1.angleCorrected),
    index: idx0 !== undefined && idx1 !== undefined ? lerp(idx0, idx1) : undefined,
    extrapolated: t < 0 || t > 1,
  };
}

// Port of wall_off_edge_check.py: main. Per WALL_OFF/WALL_OFF_DIA segment
// (whichever `motionStates` includes, e.g. [6, 13] for both at once), emits
// up to four markers per side:
//   - wall-off-anchor:  where the sensor got closest to the wall (the
//                       "found it" moment - only interesting when it's not
//                       right at the segment start, i.e. the not-yet-visible
//                       pattern).
//   - wall-off-actual:  where the current code actually fired.
//   - wall-off-arm:     where a naive "> baseline + armDelta" check would
//                       first notice the rise (Layer1, reference only - see
//                       the file-level comment on why this isn't necessarily
//                       better than wall-off-actual).
//   - wall-off-edge:    back-projected from a straight line fit through the
//                       rise (Layer2, reference only, same caveat).
export function computeWallOffEdgeEvents(rows: Record<string, number>[], opts: WallOffEdgeOptions): AnalysisEvent[] {
  const baselineN = opts.baselineN ?? WALL_OFF_DEFAULTS.baselineN;
  const armDelta = opts.armDelta ?? WALL_OFF_DEFAULTS.armDelta;
  const fitLoStraight = opts.fitLo ?? WALL_OFF_DEFAULTS.fitLo;
  const fitHiStraight = opts.fitHi ?? WALL_OFF_DEFAULTS.fitHi;
  const fitLoDia = opts.fitLoDia ?? WALL_OFF_DEFAULTS.fitLoDia;
  const fitHiDia = opts.fitHiDia ?? WALL_OFF_DEFAULTS.fitHiDia;
  const minFitN = opts.minFitN ?? WALL_OFF_DEFAULTS.minFitN;
  const medianWindow = opts.medianWindow ?? WALL_OFF_DEFAULTS.medianWindow;
  const existTh = opts.existTh ?? WALL_OFF_DEFAULTS.existTh;
  const minDelta = opts.minDelta ?? WALL_OFF_DEFAULTS.minDelta;
  const pointByRow = opts.pointByRow;

  const events: AnalysisEvent[] = [];
  const blocks = opts.motionStates.flatMap((state) =>
    findBlocks(rows, state).map(([bStart, bEnd]) => ({ bStart, bEnd, state }))
  );
  for (const { bStart, bEnd, state } of blocks) {
    const isDia = state === 13;
    const fitLo = isDia ? fitLoDia : fitLoStraight;
    const fitHi = isDia ? fitHiDia : fitHiStraight;
    const segRows = rows.slice(bStart, bEnd + 1);
    if (!("dist" in segRows[0]) || !("left45_d" in segRows[0]) || !("right45_d" in segRows[0])) continue;

    const picked = pickWallOffSide(segRows, baselineN, medianWindow, existTh, minDelta);
    const side = picked.side;
    // Neither side saw a real wall event in this segment (e.g. the robot
    // exited WALL_OFF via front correction or a distance timeout, not a
    // 45deg kireme) - nothing meaningful to draw.
    if (side === null) continue;
    const info = side === "left" ? picked.left : picked.right;
    const { anchorI, baseline } = info;
    // The not-yet-visible pattern often fires within a handful of samples of
    // the trough - only require enough room for an "actual" marker, not a
    // full baseline+fit window (that just means Layer1/Layer2 won't compute,
    // handled below).
    if (segRows.length - anchorI < 2) continue;

    const col = side === "left" ? "left45_d" : "right45_d";
    const sideSign: 1 | -1 = side === "left" ? 1 : -1;
    const xs = segRows.map((r) => r.dist);
    const ys = segRows.map((r) => r[col]);

    if (anchorI > 0 && hasXY(segRows[anchorI])) {
      const anchorRow = segRows[anchorI];
      events.push({
        x: anchorRow.x,
        y: anchorRow.y,
        anchored: "robot",
        kind: "wall-off-anchor",
        label: `壁切れ[${side}] 壁に最接近(見えていないパターンの基準点) idx=${fmt(anchorRow, "index")} 走行=${xs[anchorI].toFixed(1)}mm baseline=${baseline.toFixed(2)}mm`,
        column: col,
        seriesIndex: asNum(anchorRow, "index"),
        seriesValue: ys[anchorI],
      });
      const anchorSensorPos = resolvePos(anchorRow, col, pointByRow);
      if (anchorSensorPos.anchored === "sensor") {
        events.push({
          ...anchorSensorPos,
          kind: "wall-off-anchor-sensor",
          label: `壁切れ[${side}] 最接近時点の壁位置`,
          linkX: anchorRow.x,
          linkY: anchorRow.y,
        });
      }
    }

    const lastI = segRows.length - 1;
    const actualRow = segRows[lastI];
    if (hasXY(actualRow)) {
      events.push({
        x: actualRow.x,
        y: actualRow.y,
        anchored: "robot",
        kind: "wall-off-actual",
        label:
          `壁切れ[${side}] 現行検出 idx=${fmt(actualRow, "index")} 走行=${xs[lastI].toFixed(1)}mm ` +
          `(baseline=${baseline.toFixed(2)}mm +${(ys[lastI] - baseline).toFixed(2)}mm、` +
          `アンカーから${(xs[lastI] - xs[anchorI]).toFixed(2)}mm)`,
        column: col,
        seriesIndex: asNum(actualRow, "index"),
        seriesValue: ys[lastI],
      });
      const sensorPos = resolvePos(actualRow, col, pointByRow);
      if (sensorPos.anchored === "sensor") {
        events.push({
          ...sensorPos,
          kind: "wall-off-actual-sensor",
          label: `壁切れ[${side}] 現行検出時点の壁位置`,
          linkX: actualRow.x,
          linkY: actualRow.y,
        });
      }
    }

    let armI = -1;
    for (let i = anchorI; i < ys.length; i++) {
      if (ys[i] - baseline > armDelta) {
        armI = i;
        break;
      }
    }
    if (armI >= 0 && hasXY(segRows[armI])) {
      const armRow = segRows[armI];
      events.push({
        x: armRow.x,
        y: armRow.y,
        anchored: "robot",
        kind: "wall-off-arm",
        label: `壁切れ[${side}] 気づいた点(baseline+${armDelta}mm、参考値・実測ではactualよりばらつきが大きいことがある) idx=${fmt(armRow, "index")} 走行=${xs[armI].toFixed(1)}mm`,
        column: col,
        seriesIndex: asNum(armRow, "index"),
        seriesValue: ys[armI],
      });
    }

    const fitIdx: number[] = [];
    for (let i = anchorI; i < ys.length; i++) {
      if (ys[i] - baseline > fitLo && ys[i] - baseline < fitHi) fitIdx.push(i);
    }
    if (fitIdx.length < minFitN || !pointByRow) continue;
    const { slope, intercept, r2 } = linreg(
      fitIdx.map((i) => xs[i]),
      fitIdx.map((i) => ys[i])
    );
    if (slope === 0) continue;
    const xEdge = (baseline - intercept) / slope;
    const interp = interpolateAt(segRows, xs, xEdge, pointByRow);
    if (!interp) continue;

    events.push({
      x: interp.x,
      y: interp.y,
      anchored: "robot",
      kind: "wall-off-edge",
      label:
        `壁切れ[${side}] 逆算エッジ(参考値・実測ではactualよりばらつきが大きいことがある) 走行=${xEdge.toFixed(1)}mm slope=${slope.toFixed(3)}mm/mm r2=${r2.toFixed(3)} ` +
        `見かけの検出遅れ=${(xs[lastI] - xEdge).toFixed(1)}mm` +
        (interp.extrapolated ? " (区間外挿・要注意)" : ""),
      column: col,
      seriesIndex: interp.index,
      seriesValue: baseline,
    });
    const wp = projectSensorPoint(
      { x: interp.x, y: interp.y, angleCorrected: interp.angleCorrected, raw: { [col]: baseline } },
      col as "left45_d" | "right45_d",
      sideSign
    );
    if (wp) {
      events.push({
        x: wp.x,
        y: wp.y,
        anchored: "sensor",
        kind: "wall-off-edge-sensor",
        label: `壁切れ[${side}] 逆算エッジの壁位置`,
        linkX: interp.x,
        linkY: interp.y,
      });
    }
  }
  return events;
}
