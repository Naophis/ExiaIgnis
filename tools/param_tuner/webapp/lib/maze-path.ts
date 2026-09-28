// path_s / path_t(diagonalPath() 後の最短経路)→ 迷路上の軌跡(区画単位、y は北向き)。
// ブラウザ/Node 共用、import なし。
//
// 基準点の約束(path_create / convert_large_path / diagonalPath から導いたもの):
// - 1 ステップ = 半区画グリッドの 1 目。直進中は軸方向に 0.5 区画、斜め中は (±0.5, ±0.5)。
// - path_s[i] は、直前のターンの出口の基準点から次のターンの入口の基準点までのステップ数。
//   実際の直線は s − 2 ステップ(ターンが前後 1 ステップずつ使う。calc_goal_time の
//   (0.5 * s − 1) * cell と同じ)。
// - 最初の基準点はスタート区画の南端 (0.5, 0)(path_create は path_s = 3 で (0, 1) の中心から
//   数え始める)。機体は区画中心 (0.5, 0.5) から走り出す。
// - ターンごとの入口 → 出口の基準点のずれ:
//     Normal(1/2)   なし(区画中心)
//     Large(5/6)    step(入る向き) + step(出る向き)
//     Orval(3/4)    2 × step(横)。横 = 入る向きを曲がる側へ 90° 回した向き
//     Dia45(7/8)    なし(直線と斜めの交点)
//     Dia135(9/10)  直進→斜め: step(入る向き) + step(90° 曲がった時点の軸方向)
//                   斜め→直進: step(90° 曲がった時点の軸方向) + step(出る向き)
//     Dia90(11/12)  なし(2 本の斜め線の交点。間の直進は斜めへの出入りの補正で 0 になる)
// - 弧は「入口の基準点 − 1 ステップ」から「出口の基準点 + 1 ステップ」。
// - Finish(255)は最後の出口の基準点 + (s − 2) ステップ(= ゴール区画の中心)で止まる。
// 奇数の path_t が右、偶数が左(TrajectoryCreator::get_turn_dir)。

export type Pt = [number, number];

// 向きは 8 方位を時計回りに 0..7(N, NE, E, SE, S, SW, W, NW)。
const STEP: Pt[] = [
  [0, 0.5],
  [0.5, 0.5],
  [0.5, 0],
  [0.5, -0.5],
  [0, -0.5],
  [-0.5, -0.5],
  [-0.5, 0],
  [-0.5, 0.5],
];
const rot = (h: number, k: number) => (((h + k) % 8) + 8) % 8;
const isDiag = (h: number) => h % 2 === 1;
const add = (p: Pt, q: Pt, k = 1): Pt => [p[0] + q[0] * k, p[1] + q[1] * k];

export interface PathTurn {
  index: number; // path_s / path_t の添字
  code: number; // path_t
  name: string;
  right: boolean;
  at: Pt; // 入口と出口の基準点の中点(ラベル位置)
  arcFrom: Pt;
  arcTo: Pt;
}

export interface PathGeometry {
  // 直線(line)と弧(3 次ベジエ)の並び。区画単位、y は北向き。seg は path_s / path_t の添字
  // (区間 i = i 番目の直線 + i 番目のターン。calc_goal_time の区間と同じ)。
  pieces: (
    | { kind: "line"; seg: number; from: Pt; to: Pt }
    | { kind: "curve"; seg: number; from: Pt; c1: Pt; c2: Pt; to: Pt }
  )[];
  turns: PathTurn[];
  start: Pt;
  end: Pt | null; // Finish まで辿れたときのゴール位置
}

export function turnName(code: number, diag: boolean): string {
  if (code === 1 || code === 2) return "Normal";
  if (code === 3 || code === 4) return "Orval";
  if (code === 5 || code === 6) return "Large";
  if (code === 7 || code === 8) return diag ? "Dia45_2" : "Dia45";
  if (code === 9 || code === 10) return diag ? "Dia135_2" : "Dia135";
  if (code === 11 || code === 12) return "Dia90";
  if (code === 255) return "Finish";
  return `?${code}`;
}

// 向き h で p を通る直線と、向き g で q を通る直線の交点(平行なら null)。
function intersect(p: Pt, h: number, q: Pt, g: number): Pt | null {
  const [a, b] = STEP[h];
  const [c, d] = STEP[g];
  const det = a * -d - -c * b;
  if (Math.abs(det) < 1e-9) return null;
  const rx = q[0] - p[0];
  const ry = q[1] - p[1];
  const t = (rx * -d - -c * ry) / det;
  return add(p, STEP[h], t);
}

function arc(from: Pt, hIn: number, to: Pt, hOut: number): { c1: Pt; c2: Pt } {
  const q = intersect(from, hIn, to, hOut);
  if (q) {
    // 2 次ベジエ(制御点 = 2 本の接線の交点)を 3 次に上げたもの。
    return { c1: add(from, [q[0] - from[0], q[1] - from[1]], 2 / 3), c2: add(to, [q[0] - to[0], q[1] - to[1]], 2 / 3) };
  }
  // 180°(Orval): 接線が平行なので、進行方向へ膨らませる。
  const w = Math.hypot(to[0] - from[0], to[1] - from[1]) * 1.4;
  const u = STEP[hIn];
  const n = Math.hypot(u[0], u[1]);
  const dir: Pt = [u[0] / n, u[1] / n];
  return { c1: add(from, dir, w), c2: add(to, dir, w) };
}

export function buildPathGeometry(pathS: number[], pathT: number[]): PathGeometry {
  const pieces: PathGeometry["pieces"] = [];
  const turns: PathTurn[] = [];
  const start: Pt = [0.5, 0.5];
  let ref: Pt = [0.5, 0]; // 直前の出口の基準点
  let h = 0; // N
  let pos: Pt = start; // 描いた線の先端
  let end: Pt | null = null;

  for (let i = 0; i < pathS.length && i < pathT.length; i++) {
    const s = pathS[i];
    const code = pathT[i];
    const pin = add(ref, STEP[h], s);
    if (code === 255) {
      end = add(ref, STEP[h], s - 2);
      pieces.push({ kind: "line", seg: i, from: pos, to: end });
      break;
    }
    if (code < 1 || code > 12) break;
    const right = code % 2 === 1;
    const sg = right ? 1 : -1;
    const diag = isDiag(h);
    let hOut = h;
    let pout: Pt = pin;
    if (code <= 2) {
      hOut = rot(h, 2 * sg);
    } else if (code <= 4) {
      hOut = rot(h, 4);
      pout = add(pin, STEP[rot(h, 2 * sg)], 2);
    } else if (code <= 6) {
      hOut = rot(h, 2 * sg);
      pout = add(add(pin, STEP[h]), STEP[hOut]);
    } else if (code <= 8) {
      hOut = rot(h, sg);
    } else if (code <= 10) {
      hOut = rot(h, 3 * sg);
      const mid = diag ? rot(h, sg) : rot(h, 2 * sg);
      pout = diag ? add(add(pin, STEP[mid]), STEP[hOut]) : add(add(pin, STEP[h]), STEP[mid]);
    } else {
      hOut = rot(h, 2 * sg);
    }
    const arcFrom = add(pin, STEP[h], -1);
    const arcTo = add(pout, STEP[hOut]);
    pieces.push({ kind: "line", seg: i, from: pos, to: arcFrom });
    const { c1, c2 } = arc(arcFrom, h, arcTo, hOut);
    pieces.push({ kind: "curve", seg: i, from: arcFrom, c1, c2, to: arcTo });
    turns.push({
      index: i,
      code,
      name: turnName(code, diag),
      right,
      at: [(pin[0] + pout[0]) / 2, (pin[1] + pout[1]) / 2],
      arcFrom,
      arcTo,
    });
    pos = arcTo;
    ref = pout;
    h = hOut;
  }
  return { pieces, turns, start, end };
}

// 軌跡を折れ線にする(描画の当たり判定・検証用)。
export function samplePath(g: PathGeometry, perCurve = 16): Pt[] {
  const out: Pt[] = [];
  for (const p of g.pieces) {
    if (p.kind === "line") {
      out.push(p.from, p.to);
      continue;
    }
    for (let k = 0; k <= perCurve; k++) {
      const t = k / perCurve;
      const u = 1 - t;
      const b = [u * u * u, 3 * u * u * t, 3 * u * t * t, t * t * t];
      out.push([
        b[0] * p.from[0] + b[1] * p.c1[0] + b[2] * p.c2[0] + b[3] * p.to[0],
        b[0] * p.from[1] + b[1] * p.c1[1] + b[2] * p.c2[1] + b[3] * p.to[1],
      ]);
    }
  }
  return out;
}

// SVG の path d。toScreen で区画座標(y 北向き)を画面座標へ。seg を渡すとその区間だけ。
export function pathD(g: PathGeometry, toScreen: (p: Pt) => Pt, seg?: number): string {
  let d = "";
  let last: Pt | null = null;
  const f = (p: Pt) => {
    const [x, y] = toScreen(p);
    return `${+x.toFixed(4)} ${+y.toFixed(4)}`;
  };
  for (const p of g.pieces) {
    if (seg !== undefined && p.seg !== seg) continue;
    if (!last || last[0] !== p.from[0] || last[1] !== p.from[1]) d += `M${f(p.from)}`;
    d += p.kind === "line" ? `L${f(p.to)}` : `C${f(p.c1)} ${f(p.c2)} ${f(p.to)}`;
    last = p.to;
  }
  return d;
}
