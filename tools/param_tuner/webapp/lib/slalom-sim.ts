// Pure, client-safe TS port of tools/slalom/{slalom.py,plot.py}'s trajectory
// simulation (the tool historically launched as a standalone Tkinter app to
// derive front/back straight-line offsets and the trajectory plot for each
// turn type stored in profile/hf/t_<v>.yaml). No Node dependencies - runs in
// the browser next to the YAML editor.
//
// Ported pieces, 1:1 with the Python originals unless noted:
//  - Slalom.calc_neipire / safe_integrand / calc_base_time / calc (Euler
//    variant only - the RK4 toggles in main.py's GUI default off and this
//    panel doesn't expose them)
//  - Slalom.calc_offset_dist's per-type branches (Plot.exe's hf_cl=0 default
//    table for rad/pow_n/ang/end_pos/start_ang)
//
// slalom.py's calc_slip (the K/K_y tire-slip dynamics model) IS ported here
// (see simulateSlipStep below) to draw a second, slip-affected trajectory
// alongside the idealized one - useful as a visual "how far will the real
// robot drift" check. Note this is purely a diagnostic overlay: front/back/
// time (and everything applySimResultToYaml writes back to the yaml) always
// come from the non-slip path, matching what the firmware actually uses -
// the real robot doesn't know its own slip, so there's nothing to feed back.

export type TurnType =
  | "normal"
  | "large"
  | "orval"
  | "dia45"
  | "dia135"
  | "dia45_2"
  | "dia135_2"
  | "dia90";

export const TURN_TYPES: TurnType[] = [
  "normal",
  "large",
  "orval",
  "dia45",
  "dia135",
  "dia45_2",
  "dia135_2",
  "dia90",
];

export const TURN_TYPE_LABELS: Record<TurnType, string> = {
  normal: "normal",
  large: "large",
  orval: "orval",
  dia45: "dia45",
  dia135: "dia135",
  dia45_2: "dia45_2",
  dia135_2: "dia135_2",
  dia90: "dia90",
};

// front/back is genuinely unimplemented for orval upstream (plot.py's
// calc_offset_dist has an empty `pass` branch for it) - the trajectory is
// still meaningful, but offsets always come out as 0/0. Surface that instead
// of pretending it's a real result.
export const OFFSET_UNSUPPORTED: ReadonlySet<TurnType> = new Set(["orval"]);

interface Point {
  x: number;
  y: number;
}

// plot.py's Plot.exe default table, hf_cl=0 (the only cell size this robot
// currently uses). v is not part of the original table - it's the argument
// callers pass in (tgt_v); here it seeds from the t_<v>.yaml filename.
export const TURN_DEFAULTS: Record<
  TurnType,
  { rad: number; n: number; ang: number; startAngle: number; endPos: Point }
> = {
  normal: { rad: 27, n: 2, ang: 90, startAngle: 0, endPos: { x: 45, y: 45 } },
  large: { rad: 60.5, n: 4, ang: 90, startAngle: 0, endPos: { x: 90, y: 90 } },
  orval: { rad: 52.25, n: 4, ang: 180, startAngle: 0, endPos: { x: 0, y: 180 } },
  dia45: { rad: 54.0, n: 6, ang: 45, startAngle: 0, endPos: { x: 90, y: 45 } },
  dia135: { rad: 45.0, n: 4, ang: 135, startAngle: 0, endPos: { x: 45, y: 90 } },
  dia45_2: { rad: 52, n: 4, ang: 45, startAngle: 45, endPos: { x: 90, y: 45 } },
  dia135_2: { rad: 45.0, n: 4, ang: 135, startAngle: 45, endPos: { x: -45, y: 90 } },
  dia90: { rad: 42.0, n: 4, ang: 90, startAngle: 0, endPos: { x: 90 / Math.sqrt(2), y: 90 / Math.sqrt(2) } },
};

// main2.py's hardcoded offset block (never exposed in the Tkinter GUI
// either - the "prev"/"after"/"prev_dia"/"after_dia" straight-line lead-in
// used when solving for where the turn must start/end within the cell).
const OFFSET = { prev: 7, after: 7, prevDia: 7, afterDia: 7 };

const DT = 0.001;
const CELL_SIZE = 90;
const HALF_CELL_SIZE = 45;
const SLIP_MASS = 0.015; // slalom.py's module-level `m` (kg-scale tuning constant, not the robot's real mass)

// hardware.yaml's slip_param_k2 / slip_param_K at the time of writing - just
// the Tkinter GUI's starting point, editable in the panel like there too.
export const DEFAULT_SLIP_K = 150;
export const DEFAULT_SLIP_KY = 0.5;

function safeIntegrand(x: number, n: number): number {
  if (Math.abs(x) >= 1) return 0;
  return Math.E * Math.exp(-1 / (1 - Math.pow(x, n)));
}

// slalom.py computes Et via np.trapz over np.linspace(0, 1, int(1/dx)) with
// dx = 0.0001/64 (640000 points). Et depends only on n, so callers should
// memoize this (see etCache below) rather than recompute per keystroke.
function computeEt(n: number): number {
  const dx = 0.0001 / 64;
  const points = Math.floor(1 / dx);
  const step = 1 / (points - 1);
  let sum = 0;
  let prevY = safeIntegrand(0, n);
  for (let i = 1; i < points; i++) {
    const y = safeIntegrand(i * step, n);
    sum += ((prevY + y) / 2) * step;
    prevY = y;
  }
  return sum;
}

const etCache = new Map<number, number>();
function getEt(n: number): number {
  const cached = etCache.get(n);
  if (cached !== undefined) return cached;
  const et = computeEt(n);
  etCache.set(n, et);
  return et;
}

// slalom.py's calc_neipire: the (super-Gaussian-shaped) jerk-limited angular
// acceleration profile, normalized so integrating it over [0, base_time*2]
// sweeps through `ang` radians of heading change.
function calcNeipire(t: number, s: number, n: number): number {
  if (t <= 0) return 0;
  const tt = t / s;
  if (tt >= 2) return 0;
  const z = Math.pow(tt - 1, n);
  if (!Number.isFinite(z)) return 0;
  const denom = 1 - z;
  if (denom === 0) return 0;
  const p = Math.exp(-1 / denom);
  const powNm1 = Math.pow(tt - 1, n - 1);
  if (!Number.isFinite(p) || !Number.isFinite(powNm1)) return 0;
  const res = ((-n * p) / (denom * denom)) * powNm1 / s * Math.E;
  return Number.isFinite(res) ? res : 0;
}

function calcDist(p0: Point, p1: Point): number {
  const d2 = (p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2;
  return d2 > 0 ? Math.sqrt(d2) : 0;
}

// Hard cap on integration steps so a pathological input (e.g. v near 0)
// can't hang the browser tab. Realistic speeds/radii stay well under this.
const MAX_STEPS = 500_000;

export interface SlalomSimParams {
  type: TurnType;
  v: number; // mm/s
  rad: number; // mm
  n: number; // pow_n
  ang: number; // deg, total turn angle
  K: number; // slip_param_k2 - beta (slip angle) relaxation gain
  Ky: number; // slip_param_K - lateral force gain
}

export interface SlalomSimResult {
  path: Point[]; // idealized (non-slip) trajectory, offset by turnOffset already applied
  slipPath: Point[]; // same frame/offset as `path`, but integrated with tire-slip dynamics - overlay only
  prevPath: [Point, Point];
  afterPath: [Point, Point]; // shifted to align with the trajectory's own origin, like plot.py's after_path2
  time: number; // seconds, informational only - not written to the yaml
  front: number; // mm
  back: number; // mm
  offsetSupported: boolean;
  maxAccG: number;
  et: number;
  steps: number;
}

export function simulateSlalom(params: SlalomSimParams): SlalomSimResult {
  const { type, v, rad, n, ang, K, Ky } = params;
  const defaults = TURN_DEFAULTS[type];
  const angRad = (ang * Math.PI) / 180;
  const et = getEt(n);

  const baseAlpha = v / rad;
  const baseTime = (rad * angRad) / (2 * v * et);
  const limitTimeCount = (baseTime * 2) / DT;
  const steps = Math.min(Math.max(Math.floor(limitTimeCount), 0), MAX_STEPS);

  const startTheta = defaults.startAngle * (Math.PI / 180);

  // Idealized (non-slip) state.
  let theta = startTheta;
  let w = 0;
  let x = 0;
  let y = 0;
  let maxAccY = 0;
  const path: Point[] = [{ x, y }];

  // Tire-slip state (slalom.py's calc_slip/step_euler_slip). Driven by the
  // exact same w(t) sequence as the idealized state above - alpha(t) has no
  // feedback from slip dynamics, only x/y/theta/vx/vy/beta do.
  let slipTheta = startTheta;
  let slipX = 0;
  let slipY = 0;
  let vx = v / 1000;
  let vy = 0;
  let beta = 0;
  const slipPath: Point[] = [{ x: slipX, y: slipY }];

  for (let i = 1; i <= steps; i++) {
    const t = DT * (i - 1);
    const alpha = baseAlpha * calcNeipire(t, baseTime, n);
    const wPrev = w; // slalom.py's step_euler_slip reads state["w"] *before* it's overwritten with w_next
    const wNext = w + alpha * DT;

    theta = theta + wNext * DT;
    x = x + v * Math.cos(theta) * DT;
    y = y + v * Math.sin(theta) * DT;
    w = wNext;
    path.push({ x, y });
    maxAccY = Math.max(maxAccY, Math.abs((v * w) / 1000));

    // step_euler_slip: Fx is always forced to 0 upstream (the PI-control
    // term it would otherwise use is dead code there), so only the lateral
    // slip force Fy = -Ky * beta drives ax/ay besides the w x v coupling.
    const fy = -Ky * beta;
    const ax = wPrev * vy;
    const ay = fy / SLIP_MASS - wPrev * vx;
    const nextVx = vx + ax * DT;
    const nextVy = vy + ay * DT;
    const speed = Math.max(Math.hypot(nextVx, nextVy), 1e-6);
    slipX = slipX + speed * 1000 * Math.cos(slipTheta) * DT;
    slipY = slipY + speed * 1000 * Math.sin(slipTheta) * DT;
    const nextBeta = (beta / DT - wNext) / (1 / DT + K / speed);
    slipTheta = slipTheta + wNext * DT + (nextBeta - beta);
    vx = nextVx;
    vy = nextVy;
    beta = nextBeta;
    slipPath.push({ x: slipX, y: slipY });
  }

  const endPos = defaults.endPos;
  const endX = x;
  const endY = y;

  let prevPath: [Point, Point] = [{ x: 0, y: 0 }, { x: 0, y: 0 }];
  let afterPath: [Point, Point] = [{ x: 0, y: 0 }, { x: 0, y: 0 }];
  const offsetSupported = !OFFSET_UNSUPPORTED.has(type);

  if (offsetSupported) {
    const a = Math.sin(angRad);
    const b = Math.cos(angRad);

    switch (type) {
      case "normal": {
        const endOffset = endPos.y - endY;
        const startOffset = endPos.x - endX;
        prevPath = [{ x: 0, y: 0 }, { x: startOffset, y: 0 }];
        afterPath = [{ x: endX, y: endY }, { x: endX + endOffset * b, y: endY + endOffset * a }];
        break;
      }
      case "large": {
        const endOffset = endPos.y - endY - OFFSET.after;
        const startOffset = endPos.x - endX + OFFSET.prev;
        prevPath = [{ x: -OFFSET.prev, y: 0 }, { x: startOffset - OFFSET.prev, y: 0 }];
        afterPath = [{ x: endX, y: endY }, { x: endX + endOffset * b, y: endY + endOffset * a }];
        break;
      }
      case "dia45":
      case "dia135": {
        const endOffset = (endPos.y - endY) / a;
        const startOffset = endPos.x - endX + OFFSET.prev - endOffset * b;
        prevPath = [{ x: -OFFSET.prev, y: 0 }, { x: startOffset - OFFSET.prev, y: 0 }];
        afterPath = [{ x: endX, y: endY }, { x: endX + endOffset * b, y: endY + endOffset * a }];
        break;
      }
      case "dia45_2": {
        const startOffset = (HALF_CELL_SIZE - endX) / a + OFFSET.prevDia;
        const endOffset = CELL_SIZE - endY - (startOffset - OFFSET.prevDia) * a;
        prevPath = [
          { x: -OFFSET.prevDia * a, y: -OFFSET.prevDia * a },
          { x: (startOffset - OFFSET.prevDia) * b, y: (startOffset - OFFSET.prevDia) * a },
        ];
        afterPath = [{ x: endX, y: endY }, { x: endX, y: endY + endOffset }];
        break;
      }
      case "dia135_2": {
        const startOffset = (CELL_SIZE - endY) / b + OFFSET.prev;
        const endOffset = Math.abs(HALF_CELL_SIZE + endX + Math.abs((startOffset - OFFSET.prev) * a));
        prevPath = [
          { x: -OFFSET.prevDia * a, y: -OFFSET.prevDia * a },
          { x: Math.abs((startOffset - OFFSET.prevDia) * a), y: Math.abs((startOffset - OFFSET.prevDia) * b) },
        ];
        afterPath = [{ x: endX, y: endY }, { x: endX - endOffset, y: endY }];
        break;
      }
      case "dia90": {
        const cellDiag = CELL_SIZE / Math.SQRT2;
        const endOffset = cellDiag - endY;
        const startOffset = cellDiag - endX + OFFSET.prevDia;
        prevPath = [{ x: -OFFSET.prevDia, y: 0 }, { x: startOffset - OFFSET.prevDia, y: 0 }];
        afterPath = [{ x: endX, y: endY }, { x: endX + endOffset * b, y: endY + endOffset * a }];
        break;
      }
    }
  }

  const turnOffset = prevPath[1];
  const offsetPath = path.map((p) => ({ x: p.x + turnOffset.x, y: p.y + turnOffset.y }));
  // Drawn in the same cell-aligned frame as the idealized path (rather than
  // solving its own offset like plotorval.py's independent res2 does), so
  // the overlay directly shows how far slip drags the robot off the
  // idealized line the firmware actually plans for.
  const offsetSlipPath = slipPath.map((p) => ({ x: p.x + turnOffset.x, y: p.y + turnOffset.y }));
  const afterPath2: [Point, Point] = [
    { x: afterPath[0].x + turnOffset.x, y: afterPath[0].y + turnOffset.y },
    { x: afterPath[1].x + turnOffset.x, y: afterPath[1].y + turnOffset.y },
  ];

  return {
    path: offsetPath,
    slipPath: offsetSlipPath,
    prevPath,
    afterPath: afterPath2,
    time: baseTime,
    front: calcDist(prevPath[0], prevPath[1]),
    back: calcDist(afterPath[0], afterPath[1]),
    offsetSupported,
    maxAccG: maxAccY / 9.8,
    et,
    steps,
  };
}
