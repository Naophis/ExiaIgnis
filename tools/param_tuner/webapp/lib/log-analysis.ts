// Client-side ports of analyze_sensor_drop.py / analyze_motion_state_transitions.py.
// Both scripts scan a raw (unsorted, as-logged) CSV row sequence for
// motion_state transitions and report index/value info; here we additionally
// resolve each event to the row's (x, y) so it can be drawn as a marker on
// the trajectory plot. Row order must match the scripts' (CSV file order,
// not timestamp-sorted) - use the raw parseCsv() output, not trajectory.ts's
// sorted rows.

import { projectSensorPoint, type TrajectoryPoint } from "@/lib/trajectory";

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
    | "trough-rise";
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
