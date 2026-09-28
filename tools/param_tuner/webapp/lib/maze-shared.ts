// 迷路ファイルの形式(ブラウザ/Node 共用)。sample/mm_maze_viewer(VSCode 拡張)の
// Maze クラスの移植。
//
// .maze / maze_logs/ / maze_data/*.yaml の wall 配列はどれも同じ並び:
//   idx = x * size + y   (テキストでは 1 行 = 1 列 x、行の中は y = 0 から北へ)
// 各要素の下位 4bit が壁(N=1, E=2, W=4, S=8)。ファームの map[x + y * size] とは
// 転置の関係で、送受信時に serial-manager の swapMazeTriangle が入れ替える。

export const WALL_N = 0x01;
export const WALL_E = 0x02;
export const WALL_W = 0x04;
export const WALL_S = 0x08;

export type WallDir = "N" | "E" | "W" | "S";
export type Cell = [number, number];

export const WALL_BIT: Record<WallDir, number> = { N: WALL_N, E: WALL_E, W: WALL_W, S: WALL_S };

export function mazeIndex(size: number, x: number, y: number): number {
  return x * size + y;
}

export function mazeSizeOf(count: number): number {
  const size = Math.round(Math.sqrt(count));
  if (size < 2 || size * size !== count) {
    throw new Error(`壁データが ${count} 個で、正方形の迷路になりません`);
  }
  return size;
}

// 改行・空白・末尾のカンマは区切りとして読み飛ばす(拡張で編集したファイルは
// 行末に "," が残っていることがある)。上位 4bit(踏破フラグ)は捨てる。
export function parseMazeText(text: string): number[] {
  const tokens = text.split(/[\s,]+/).filter((t) => t.length > 0);
  const walls = tokens.map((t) => {
    const v = Number(t);
    if (!Number.isInteger(v) || v < 0 || v > 0xff) throw new Error(`壁データに数値でない値があります: ${t}`);
    return v & 0x0f;
  });
  mazeSizeOf(walls.length);
  return walls;
}

// 拡張の get_text() と同じく 1 行 = 1 列 x。
export function formatMazeText(walls: number[], size: number): string {
  const lines: string[] = [];
  for (let x = 0; x < size; x++) lines.push(walls.slice(x * size, (x + 1) * size).join(","));
  return `${lines.join(",\n")}\n`;
}

// 拡張の README のテンプレートと同じ: 外周 + スタート区画の東壁。
export function blankMaze(size: number): number[] {
  const walls = new Array<number>(size * size).fill(0);
  for (let i = 0; i < size; i++) {
    walls[mazeIndex(size, 0, i)] |= WALL_W;
    walls[mazeIndex(size, size - 1, i)] |= WALL_E;
    walls[mazeIndex(size, i, 0)] |= WALL_S;
    walls[mazeIndex(size, i, size - 1)] |= WALL_N;
  }
  walls[mazeIndex(size, 0, 0)] |= WALL_E;
  walls[mazeIndex(size, 1, 0)] |= WALL_W;
  return walls;
}

// 壁 1 枚を 2 区画の組で表す。(x, y) の N/E を正とし、S/W は隣の区画の N/E に
// 読み替える。外周の S/W だけは隣が無いのでそのまま持つ。
export interface Edge {
  x: number;
  y: number;
  dir: WallDir;
}

export function normalizeEdge(e: Edge): Edge {
  if (e.dir === "S" && e.y > 0) return { x: e.x, y: e.y - 1, dir: "N" };
  if (e.dir === "W" && e.x > 0) return { x: e.x - 1, y: e.y, dir: "E" };
  return e;
}

export function edgeKey(e: Edge): string {
  return `${e.x},${e.y},${e.dir}`;
}

export function isOuterEdge(size: number, e: Edge): boolean {
  return (
    (e.dir === "N" && e.y === size - 1) ||
    (e.dir === "E" && e.x === size - 1) ||
    (e.dir === "S" && e.y === 0) ||
    (e.dir === "W" && e.x === 0)
  );
}

// 壁の両側の区画。外周は片側だけ。
function edgeSides(size: number, e: Edge): { idx: number; bit: number }[] {
  const n = normalizeEdge(e);
  const sides = [{ idx: mazeIndex(size, n.x, n.y), bit: WALL_BIT[n.dir] }];
  if (n.dir === "N" && n.y < size - 1) sides.push({ idx: mazeIndex(size, n.x, n.y + 1), bit: WALL_S });
  if (n.dir === "E" && n.x < size - 1) sides.push({ idx: mazeIndex(size, n.x + 1, n.y), bit: WALL_W });
  return sides;
}

// 両側の区画の言い分。片側だけに壁がある(食い違い)は "mismatch"。
export function edgeState(walls: number[], size: number, e: Edge): "wall" | "open" | "mismatch" {
  const sides = edgeSides(size, e);
  const n = sides.filter((s) => (walls[s.idx] & s.bit) !== 0).length;
  if (n === 0) return "open";
  return n === sides.length ? "wall" : "mismatch";
}

// 両側の区画をそろえて書く(食い違っていた壁もこれで揃う)。
export function setEdge(walls: number[], size: number, e: Edge, present: boolean): number[] {
  const out = walls.slice();
  for (const s of edgeSides(size, e)) out[s.idx] = present ? out[s.idx] | s.bit : out[s.idx] & ~s.bit & 0x0f;
  return out;
}
