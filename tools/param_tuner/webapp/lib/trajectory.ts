// Pure, client-safe port of plot_gui.py's trajectory computation (CSV load,
// timestamp sort, cumulative-angle reconstruction, per-state grouping, and
// 45deg wall-sensor projection). No Node dependencies - this runs in the
// browser so the canvas renderer can redraw without a server round-trip.

const SEN_MIN = 20;
const SEN_MAX = 80.5;
const SENSOR_X_OFFSET = 25.0; // mm forward from robot center
const SENSOR_ANGLE_RAD = (58 * Math.PI) / 180; // sensor mounting angle
const DEVIATION_DEG_TH = 25; // deg
// Applied to trajectory points (and baked into wall-sensor projections) when
// plotting, but deliberately NOT applied to the wall-grid extent below -
// this mirrors plot_gui.py's own plot_file()/`_plot_wall_sensor`, which is
// inconsistent about it in exactly this way.
const POS_OFFSET_X = 45 - 9;

export interface TrajectoryPoint {
  x: number;
  y: number;
  timestamp: number;
  angleCorrected: number; // radians
  raw: Record<string, number>;
}

export interface TrajectoryGroup {
  state: number;
  color: string;
  points: TrajectoryPoint[];
}

export interface WallPoint {
  x: number;
  y: number;
}

export interface GridLine {
  x1: number;
  y1: number;
  x2: number;
  y2: number;
}

export interface TrajectoryData {
  groups: TrajectoryGroup[];
  leftWallPoints: WallPoint[];
  rightWallPoints: WallPoint[];
  gridLines: GridLine[];
  allPoints: TrajectoryPoint[];
  worldBounds: { xMin: number; xMax: number; yMin: number; yMax: number };
}

export function parseCsv(text: string): Record<string, number>[] {
  const lines = text.split(/\r?\n/).filter((l) => l.trim().length > 0);
  if (lines.length < 2) return [];
  const headers = lines[0].split(",").map((h) => h.trim());
  const rows: Record<string, number>[] = [];
  for (let i = 1; i < lines.length; i++) {
    const cells = lines[i].split(",");
    const row: Record<string, number> = {};
    for (let j = 0; j < headers.length; j++) {
      row[headers[j]] = parseFloat(cells[j]);
    }
    rows.push(row);
  }
  return rows;
}

function stableSortByTimestamp(rows: Record<string, number>[]): Record<string, number>[] {
  const hasIndex = rows.length > 0 && "index" in rows[0];
  return rows
    .map((r, i) => ({ r, i }))
    .sort((a, b) => {
      const dt = a.r.timestamp - b.r.timestamp;
      if (dt !== 0) return dt;
      if (hasIndex) return a.r.index - b.r.index;
      return a.i - b.i; // stable fallback
    })
    .map((x) => x.r);
}

// ang_kf_sum is already cumulative across motions; ang_kf is not, so it's
// reconstructed by carrying the previous cumulative value forward across
// timestamp boundaries, unless ideal_ang shows a >25deg jump (a real reset).
function computeAngleCorrected(rows: Record<string, number>[]): number[] {
  const n = rows.length;
  const out = new Array<number>(n).fill(0);
  if (n === 0) return out;

  if ("ang_kf_sum" in rows[0]) {
    for (let i = 0; i < n; i++) out[i] = (rows[i].ang_kf_sum * Math.PI) / 180;
    return out;
  }
  if (!("ang_kf" in rows[0])) return out;

  const hasIdeal = "ideal_ang" in rows[0];
  let angleOffset = 0;
  let prevTimestamp: number | null = null;
  for (let i = 0; i < n; i++) {
    const ts = rows[i].timestamp;
    const angKfDeg = rows[i].ang_kf;
    if (prevTimestamp !== null && ts !== prevTimestamp) {
      if (hasIdeal) {
        const deviation = Math.abs(angKfDeg - rows[i].ideal_ang);
        angleOffset = deviation > DEVIATION_DEG_TH ? 0 : out[i - 1];
      } else {
        angleOffset = out[i - 1];
      }
    }
    out[i] = (angKfDeg * Math.PI) / 180 + angleOffset;
    prevTimestamp = ts;
  }
  return out;
}

// Teal -> gold sequential ramp (matches the app's theme) standing in for
// matplotlib's viridis; only needs to read as "earlier -> later" visually.
function sequentialColor(t: number): string {
  const stops = [
    { r: 0x0a, g: 0x3b, b: 0x38 },
    { r: 0x2d, g: 0xd4, b: 0xbf },
    { r: 0xe8, g: 0xc5, b: 0x4a },
  ];
  const scaled = Math.min(Math.max(t, 0), 1) * (stops.length - 1);
  const i = Math.min(Math.floor(scaled), stops.length - 2);
  const f = scaled - i;
  const a = stops[i];
  const b = stops[i + 1];
  const lerp = (x: number, y: number) => Math.round(x + (y - x) * f);
  return `rgb(${lerp(a.r, b.r)},${lerp(a.g, b.g)},${lerp(a.b, b.b)})`;
}

function projectWallSensor(
  points: TrajectoryPoint[],
  column: "left45_d" | "right45_d",
  sideSign: 1 | -1,
  out: WallPoint[]
) {
  for (const p of points) {
    const sensorD = p.raw[column];
    if (sensorD === undefined || Number.isNaN(sensorD)) continue;
    if (!(sensorD >= SEN_MIN && sensorD < SEN_MAX)) continue;
    const angle = p.angleCorrected;
    const sensorY = sensorD;
    const sensorX = sensorY / Math.tan(SENSOR_ANGLE_RAD);
    const mountX = p.x + SENSOR_X_OFFSET * Math.cos(angle);
    const mountY = p.y + SENSOR_X_OFFSET * Math.sin(angle);
    out.push({
      x: mountX + sensorX * Math.cos(angle) - sideSign * sensorY * Math.sin(angle) + POS_OFFSET_X,
      y: mountY + sensorX * Math.sin(angle) + sideSign * sensorY * Math.cos(angle),
    });
  }
}

function buildGridLines(xMax: number, yMin: number, yMax: number, yMaxAbs: number): GridLine[] {
  const lines: GridLine[] = [];
  const flip = yMaxAbs > 100 && yMin < 0;
  for (let i = 0; i < xMax + 90; i += 90) {
    lines.push(flip ? { x1: i, y1: 45, x2: i, y2: yMin - 45 } : { x1: i, y1: 45, x2: i, y2: yMax + 45 });
  }
  const horizExtent = Math.max(-yMin, yMax) + 90;
  for (let i = 0; i < horizExtent; i += 90) {
    lines.push(
      flip ? { x1: 0, y1: -i + 45, x2: xMax + 90, y2: -i + 45 } : { x1: 0, y1: i + 45, x2: xMax + 90, y2: i + 45 }
    );
  }
  return lines;
}

export function buildTrajectoryData(rawRows: Record<string, number>[]): TrajectoryData | null {
  if (rawRows.length === 0 || !("x" in rawRows[0]) || !("y" in rawRows[0])) return null;

  const sorted = stableSortByTimestamp(rawRows);
  const angleCorrected = computeAngleCorrected(sorted);

  let xMin = Infinity;
  let xMax = -Infinity;
  let yMin = Infinity;
  let yMax = -Infinity;
  let yMaxAbs = 0;
  for (const r of sorted) {
    if (r.x < xMin) xMin = r.x;
    if (r.x > xMax) xMax = r.x;
    if (r.y < yMin) yMin = r.y;
    if (r.y > yMax) yMax = r.y;
    if (Math.abs(r.y) > yMaxAbs) yMaxAbs = Math.abs(r.y);
  }

  // filtered_data = data[data['timestamp'].diff().fillna(0) >= 0]
  const stateOrder: number[] = [];
  const stateMap = new Map<number, TrajectoryPoint[]>();
  for (let i = 0; i < sorted.length; i++) {
    const diff = i === 0 ? 0 : sorted[i].timestamp - sorted[i - 1].timestamp;
    if (diff < 0) continue;
    const r = sorted[i];
    const ts = r.timestamp;
    if (!stateMap.has(ts)) {
      stateMap.set(ts, []);
      stateOrder.push(ts);
    }
    stateMap.get(ts)!.push({ x: r.x, y: r.y, timestamp: ts, angleCorrected: angleCorrected[i], raw: r });
  }

  const numStates = stateOrder.length;
  const groups: TrajectoryGroup[] = stateOrder.map((state, i) => ({
    state,
    color: sequentialColor(numStates > 1 ? i / (numStates - 1) : 0.5),
    points: stateMap.get(state)!,
  }));

  const leftWallPoints: WallPoint[] = [];
  const rightWallPoints: WallPoint[] = [];
  const allPoints: TrajectoryPoint[] = [];
  let worldXMin = xMin + POS_OFFSET_X;
  let worldXMax = xMax + POS_OFFSET_X;
  let worldYMin = yMin;
  let worldYMax = yMax;
  const grow = (x: number, y: number) => {
    if (x < worldXMin) worldXMin = x;
    if (x > worldXMax) worldXMax = x;
    if (y < worldYMin) worldYMin = y;
    if (y > worldYMax) worldYMax = y;
  };

  for (const group of groups) {
    projectWallSensor(group.points, "left45_d", 1, leftWallPoints);
    projectWallSensor(group.points, "right45_d", -1, rightWallPoints);
    for (const p of group.points) allPoints.push(p);
  }
  for (const w of leftWallPoints) grow(w.x, w.y);
  for (const w of rightWallPoints) grow(w.x, w.y);
  worldXMin = Math.min(worldXMin, xMin);
  worldXMax = Math.max(worldXMax, xMax);

  const gridLines = buildGridLines(xMax, yMin, yMax, yMaxAbs);
  for (const l of gridLines) {
    grow(l.x1, l.y1);
    grow(l.x2, l.y2);
  }

  return {
    groups,
    leftWallPoints,
    rightWallPoints,
    gridLines,
    allPoints,
    worldBounds: { xMin: worldXMin, xMax: worldXMax, yMin: worldYMin, yMax: worldYMax },
  };
}

export const TRAJECTORY_POS_OFFSET_X = POS_OFFSET_X;
