// Pure, client-safe port of plot_gui.py's trajectory computation (CSV load,
// timestamp sort, cumulative-angle reconstruction, per-state grouping, and
// 45deg wall-sensor projection). No Node dependencies - this runs in the
// browser so the canvas renderer can redraw without a server round-trip.

const SEN_MIN = 20;
const SEN_MAX = 80.5;
const SENSOR_X_OFFSET = 25.0; // mm forward from robot center
const SENSOR_ANGLE_RAD = (58 * Math.PI) / 180; // sensor mounting angle
const DEVIATION_DEG_TH = 25; // deg
// Applied to trajectory points, wall-sensor projections, and the maze-cell
// grid lines below so all three line up in the same world-space.
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
  // Half-cell (45mm) auxiliary line rather than a real 90mm cell boundary.
  // Drawn dotted so it reads as a guide, not a wall.
  minor?: boolean;
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

// Projects a single trajectory point's 45deg wall-sensor reading onto its
// world-space wall-contact position (same math as the bulk `_plot_wall_sensor`
// projection below, factored out so callers with a single point - e.g. an
// analysis-event marker - can reuse it instead of duplicating the geometry).
export function projectSensorPoint(
  p: Pick<TrajectoryPoint, "x" | "y" | "angleCorrected" | "raw">,
  column: "left45_d" | "right45_d",
  sideSign: 1 | -1
): WallPoint | null {
  const sensorD = p.raw[column];
  if (sensorD === undefined || Number.isNaN(sensorD)) return null;
  if (!(sensorD >= SEN_MIN && sensorD < SEN_MAX)) return null;
  const angle = p.angleCorrected;
  const sensorY = sensorD;
  const sensorX = sensorY / Math.tan(SENSOR_ANGLE_RAD);
  const mountX = p.x + SENSOR_X_OFFSET * Math.cos(angle);
  const mountY = p.y + SENSOR_X_OFFSET * Math.sin(angle);
  return {
    x: mountX + sensorX * Math.cos(angle) - sideSign * sensorY * Math.sin(angle) + POS_OFFSET_X,
    y: mountY + sensorX * Math.sin(angle) + sideSign * sensorY * Math.cos(angle),
  };
}

function projectWallSensor(
  points: TrajectoryPoint[],
  column: "left45_d" | "right45_d",
  sideSign: 1 | -1,
  out: WallPoint[]
) {
  for (const p of points) {
    const wp = projectSensorPoint(p, column, sideSign);
    if (wp) out.push(wp);
  }
}

const CELL_SIZE = 90; // mm, one maze cell
const HALF_CELL = CELL_SIZE / 2; // 45mm auxiliary pitch (cell center lines)
const CELL_Y_OFFSET = 45; // matches the historical plot_gui.py row-boundary offset

// Draws maze-cell (90mm) outlines only for cells the trajectory actually
// passes through, instead of a blanket rectangle over the whole bounding
// box - a full-extent grid ends up drawing lines far from any real data
// whenever worldBounds gets pulled wide by an outlier point (wall-sensor
// projection, a stray return-run excursion, etc).
// `points` must already be in the same offset world-space the trajectory
// dots are drawn in (x + POS_OFFSET_X, y) so cell boundaries line up with
// what's on screen.
function buildGridLines(points: { x: number; y: number }[]): GridLine[] {
  const cells = new Set<string>();
  for (const p of points) {
    const cx = Math.floor(p.x / CELL_SIZE);
    const cy = Math.floor((p.y - CELL_Y_OFFSET) / CELL_SIZE);
    cells.add(`${cx},${cy}`);
  }

  const lines: GridLine[] = [];
  const seen = new Set<string>();
  const addLine = (x1: number, y1: number, x2: number, y2: number, minor = false) => {
    const key = `${x1},${y1},${x2},${y2}`;
    if (seen.has(key)) return;
    seen.add(key);
    lines.push(minor ? { x1, y1, x2, y2, minor } : { x1, y1, x2, y2 });
  };

  for (const key of cells) {
    const [cx, cy] = key.split(",").map(Number);
    const x0 = cx * CELL_SIZE;
    const x1 = x0 + CELL_SIZE;
    const y0 = cy * CELL_SIZE + CELL_Y_OFFSET;
    const y1 = y0 + CELL_SIZE;
    addLine(x0, y0, x0, y1);
    addLine(x1, y0, x1, y1);
    addLine(x0, y0, x1, y0);
    addLine(x0, y1, x1, y1);
    // 45mm auxiliary guides through the cell's edge midpoints (the half-cell
    // landmarks the search/slalom offsets are all specified against):
    //   - the upright cross, i.e. the cell center lines
    //   - the inscribed diamond, i.e. the 45deg diagonal lattice a diagonal
    //     run actually travels along
    const mx = x0 + HALF_CELL;
    const my = y0 + HALF_CELL;
    addLine(mx, y0, mx, y1, true);
    addLine(x0, my, x1, my, true);
    addLine(mx, y0, x1, my, true);
    addLine(x1, my, mx, y1, true);
    addLine(mx, y1, x0, my, true);
    addLine(x0, my, mx, y0, true);
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
  for (const r of sorted) {
    if (r.x < xMin) xMin = r.x;
    if (r.x > xMax) xMax = r.x;
    if (r.y < yMin) yMin = r.y;
    if (r.y > yMax) yMax = r.y;
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

  const gridLines = buildGridLines(allPoints.map((p) => ({ x: p.x + POS_OFFSET_X, y: p.y })));
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
