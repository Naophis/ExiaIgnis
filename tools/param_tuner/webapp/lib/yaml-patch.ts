import { load as loadYaml } from "js-yaml";
import { isMap, isScalar, isSeq, parseDocument, type Document, type Node, type Pair, type YAMLMap, type YAMLSeq } from "yaml";
import { pathToString, type PathSeg } from "./machine-shared";
import { canonical, getAt, withValueAt } from "./yaml-compare";

// yaml の文字列を「キー 1 つ分だけ」書き換える(Node 依存なし)。
// パラメータの yaml はコメント(調整の経緯、コメントアウトした値)だらけなので、
// parse → dump では書き直さない。`yaml` パッケージで場所(文字の範囲)だけを調べ、
// 相手の機体のファイルから同じキーの文字列を切り出して差し込む。
// 書き換えたあとは必ず読み直して「そのキーだけが、その値になった」ことを確かめ、
// 違えば例外にする(呼ぶ側はファイルへ書かない)。

type Doc = Document.Parsed;

function parse(text: string, label: string): Doc {
  const doc = parseDocument(text, { prettyErrors: true });
  if (doc.errors.length > 0) throw new Error(`${label}: yaml を読めません: ${doc.errors[0].message}`);
  return doc;
}

interface Lines {
  starts: number[]; // 各行の先頭の位置
  lineOf: (pos: number) => number; // 0 始まり
  text: (line: number) => string; // 改行を含まない
  end: (line: number) => number; // 行末(改行の手前)の位置
}

function linesOf(text: string): Lines {
  const starts = [0];
  for (let i = 0; i < text.length; i++) if (text[i] === "\n") starts.push(i + 1);
  const lineOf = (pos: number) => {
    let lo = 0;
    let hi = starts.length - 1;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (starts[mid] <= pos) lo = mid;
      else hi = mid - 1;
    }
    return lo;
  };
  const end = (line: number) => (line + 1 < starts.length ? starts[line + 1] - 1 : text.length);
  return { starts, lineOf, end, text: (line) => text.slice(starts[line], end(line)) };
}

const isCommentLine = (s: string) => s.trimStart().startsWith("#");

function keyName(pair: Pair): string {
  const k = pair.key;
  return String(isScalar(k) ? k.value : k);
}

function findPair(map: YAMLMap, name: string): Pair | undefined {
  return map.items.find((p) => keyName(p) === name);
}

// 値の文字の範囲 [start, end)。ブロックの枝は range が末尾の改行まで含むので、
// 後ろの空白・改行を落とす。空の値(`key:`)は start === end。
function valueSpan(text: string, node: Node | null | undefined): [number, number] | null {
  const r = node?.range;
  if (!r) return null;
  let end = r[1];
  while (end > r[0] && /\s/.test(text[end - 1])) end--;
  return [r[0], end];
}

// キーの直後の `:` の次の位置
function afterColon(text: string, pair: Pair): number {
  const r = (pair.key as Node).range;
  if (!r) throw new Error("キーの位置が分かりません");
  const i = text.indexOf(":", r[1]);
  if (i < 0) throw new Error("キーの `:` が見つかりません");
  return i + 1;
}

// 2 行目以降の字下げを delta だけずらす
function reindent(block: string, delta: number): string {
  if (delta === 0 || !block.includes("\n")) return block;
  const lines = block.split("\n");
  for (let i = 1; i < lines.length; i++) {
    if (lines[i].trim() === "") continue;
    if (delta > 0) lines[i] = " ".repeat(delta) + lines[i];
    else {
      const lead = lines[i].length - lines[i].trimStart().length;
      if (lead < -delta) throw new Error("字下げを合わせられません");
      lines[i] = lines[i].slice(-delta);
    }
  }
  return lines.join("\n");
}

// 1 行目も含めて全部の行をずらす(行ごと差し込むとき)
function reindentAll(block: string, delta: number): string {
  if (delta === 0) return block;
  return block
    .split("\n")
    .map((l) => {
      if (l.trim() === "") return l;
      if (delta > 0) return " ".repeat(delta) + l;
      const lead = l.length - l.trimStart().length;
      if (lead < -delta) throw new Error("字下げを合わせられません");
      return l.slice(-delta);
    })
    .join("\n");
}

interface Located {
  // たどれた深さ(segs.length なら全部ある)
  depth: number;
  // depth の位置の親(マップか配列)。depth === segs.length のときは最後のキーの親
  parent: YAMLMap | YAMLSeq | null;
  pair: Pair | null; // 最後にたどったのがマップのキーならその組
  node: Node | null; // 最後にたどった値
}

function locate(doc: Doc, segs: readonly PathSeg[]): Located {
  let node: Node | null = doc.contents as Node | null;
  let parent: YAMLMap | YAMLSeq | null = null;
  let pair: Pair | null = null;
  for (let depth = 0; depth < segs.length; depth++) {
    const s = segs[depth];
    if (typeof s === "number") {
      if (!isSeq(node)) return { depth, parent: null, pair: null, node: null };
      if (s >= node.items.length) return { depth, parent: node, pair: null, node: null };
      parent = node;
      pair = null;
      node = node.items[s] as Node | null;
    } else {
      if (!isMap(node)) return { depth, parent: null, pair: null, node: null };
      const p = findPair(node, s);
      if (!p) return { depth, parent: node, pair: null, node: null };
      parent = node;
      pair = p;
      node = p.value as Node | null;
    }
  }
  return { depth: segs.length, parent, pair, node };
}

function colOf(lines: Lines, pos: number): number {
  return pos - lines.starts[lines.lineOf(pos)];
}

function replaceValue(srcText: string, dstText: string, src: Located, dst: Located): string {
  const sl = linesOf(srcText);
  const dl = linesOf(dstText);
  const sv = valueSpan(srcText, src.node);
  const dv = valueSpan(dstText, dst.node);

  // 値がキーと同じ行にあるか(`key: 1` / `key: [1, 2]`)、次の行から始まるか(ブロックの枝)
  const sameLine = (text: string, ls: Lines, loc: Located, span: [number, number] | null) => {
    if (!loc.pair) return true; // 配列の要素
    if (!span || span[0] === span[1]) return null; // 空の値
    const keyPos = (loc.pair.key as Node).range![0];
    return ls.lineOf(keyPos) === ls.lineOf(span[0]);
  };
  const sSame = sameLine(srcText, sl, src, sv);
  const dSame = sameLine(dstText, dl, dst, dv);

  if (sv && dv && sSame !== null && dSame !== null && sSame === dSame) {
    // 値の文字だけを入れ替える(行末コメントは相手側のものが残る)
    const piece = reindent(srcText.slice(sv[0], sv[1]), colOf(dl, dv[0]) - colOf(sl, sv[0]));
    return dstText.slice(0, dv[0]) + piece + dstText.slice(dv[1]);
  }
  if (!src.pair || !dst.pair) throw new Error("この形の値は自動で書き換えられません");
  // 片方が空の値、または「同じ行の値」と「次の行からの枝」の入れ替え:
  // `:` の直後から値の終わりまでを入れ替える
  const sFrom = afterColon(srcText, src.pair);
  const dFrom = afterColon(dstText, dst.pair);
  const sTo = sv ? Math.max(sv[1], sFrom) : sFrom;
  const dTo = dv ? Math.max(dv[1], dFrom) : dFrom;
  const sKeyCol = colOf(sl, (src.pair.key as Node).range![0]);
  const dKeyCol = colOf(dl, (dst.pair.key as Node).range![0]);
  const piece = reindent(srcText.slice(sFrom, sTo), dKeyCol - sKeyCol);
  return dstText.slice(0, dFrom) + piece + dstText.slice(dTo);
}

// ブロックのマップへ、src のキー(直前のコメント行ごと)を行単位で差し込む
function insertIntoBlockMap(
  srcText: string,
  dstText: string,
  srcParent: YAMLMap,
  dstParent: YAMLMap,
  srcPair: Pair,
): string {
  const sl = linesOf(srcText);
  const dl = linesOf(dstText);
  const sKeyPos = (srcPair.key as Node).range![0];
  const sKeyLine = sl.lineOf(sKeyPos);
  const sCol = colOf(sl, sKeyPos);
  if (sl.text(sKeyLine).slice(0, sCol).trim() !== "") throw new Error("キーが行の先頭にありません");

  // 直前に続くコメント行(空行を挟まないもの)は、そのキーの説明として一緒に持っていく
  let first = sKeyLine;
  while (first > 0 && isCommentLine(sl.text(first - 1))) first--;
  const sv = valueSpan(srcText, srcPair.value as Node | null);
  const last = sv && sv[1] > sv[0] ? sl.lineOf(sv[1] - 1) : sKeyLine;
  const block = srcText.slice(sl.starts[first], sl.end(last));

  if (dstParent.items.length === 0) throw new Error("相手側の枝が空です");
  const dCol = colOf(dl, (dstParent.items[0].key as Node).range![0]);
  const piece = reindentAll(block, dCol - sCol);

  // src での並びで直前にあるキーのうち、相手側にもあるものの直後へ入れる
  const srcIdx = srcParent.items.indexOf(srcPair);
  for (let i = srcIdx - 1; i >= 0; i--) {
    const prev = findPair(dstParent, keyName(srcParent.items[i]));
    if (!prev) continue;
    const pv = valueSpan(dstText, prev.value as Node | null);
    const prevLine = pv && pv[1] > pv[0] ? dl.lineOf(pv[1] - 1) : dl.lineOf((prev.key as Node).range![0]);
    const at = dl.end(prevLine);
    return `${dstText.slice(0, at)}\n${piece}${dstText.slice(at)}`;
  }
  // 前に無ければ、後ろにあるキー(とその直前のコメント行)の手前へ
  for (let i = srcIdx + 1; i < srcParent.items.length; i++) {
    const next = findPair(dstParent, keyName(srcParent.items[i]));
    if (!next) continue;
    let line = dl.lineOf((next.key as Node).range![0]);
    while (line > 0 && isCommentLine(dl.text(line - 1))) line--;
    const at = dl.starts[line];
    return `${dstText.slice(0, at)}${piece}\n${dstText.slice(at)}`;
  }
  // 共通のキーが 1 つも無い: 相手側の枝の最後へ
  const lastPair = dstParent.items[dstParent.items.length - 1];
  const lv = valueSpan(dstText, lastPair.value as Node | null);
  const lastLine = lv && lv[1] > lv[0] ? dl.lineOf(lv[1] - 1) : dl.lineOf((lastPair.key as Node).range![0]);
  const at = dl.end(lastLine);
  return `${dstText.slice(0, at)}\n${piece}${dstText.slice(at)}`;
}

// `{ left: 1, right: 2 }` の形のマップへキーを足す
function insertIntoFlowMap(
  srcText: string,
  dstText: string,
  srcParent: YAMLMap,
  dstParent: YAMLMap,
  srcPair: Pair,
): string {
  const sv = valueSpan(srcText, srcPair.value as Node | null);
  const keyRange = (srcPair.key as Node).range!;
  const piece = `${srcText.slice(keyRange[0], keyRange[1])}: ${sv ? srcText.slice(sv[0], sv[1]) : ""}`.trimEnd();
  if (piece.includes("\n")) throw new Error("複数行の値は { } の中へ入れられません");
  const srcIdx = srcParent.items.indexOf(srcPair);
  for (let i = srcIdx - 1; i >= 0; i--) {
    const prev = findPair(dstParent, keyName(srcParent.items[i]));
    if (!prev) continue;
    const pv = valueSpan(dstText, prev.value as Node | null);
    const at = pv ? pv[1] : (prev.key as Node).range![1];
    return `${dstText.slice(0, at)}, ${piece}${dstText.slice(at)}`;
  }
  if (dstParent.items.length > 0) {
    const at = (dstParent.items[0].key as Node).range![0];
    return `${dstText.slice(0, at)}${piece}, ${dstText.slice(at)}`;
  }
  const at = dstParent.range![0] + 1; // `{` の直後
  return `${dstText.slice(0, at)} ${piece} ${dstText.slice(at)}`;
}

// ブロックの配列(`- ...`)の最後へ、src の要素を行単位で足す
function appendToBlockSeq(srcText: string, dstText: string, srcParent: YAMLSeq, dstParent: YAMLSeq, index: number): string {
  if (index !== dstParent.items.length) {
    throw new Error("配列の途中の要素は足せません(前の要素から順に足してください)");
  }
  if (dstParent.items.length === 0 || dstParent.flow || srcParent.flow) {
    throw new Error("この形の配列には自動で足せません");
  }
  const sl = linesOf(srcText);
  const dl = linesOf(dstText);
  const dashLine = (ls: Lines, text: string, node: Node) => {
    const line = ls.lineOf(node.range![0]);
    const t = ls.text(line);
    const col = t.length - t.trimStart().length;
    if (t[col] !== "-") throw new Error("配列の要素の先頭(-)が見つかりません");
    return { line, col };
  };
  const sItem = srcParent.items[index] as Node;
  const s = dashLine(sl, srcText, sItem);
  const sv = valueSpan(srcText, sItem);
  const sLast = sv && sv[1] > sv[0] ? sl.lineOf(sv[1] - 1) : s.line;
  const block = srcText.slice(sl.starts[s.line], sl.end(sLast));

  const dItem = dstParent.items[dstParent.items.length - 1] as Node;
  const d = dashLine(dl, dstText, dItem);
  const dv = valueSpan(dstText, dItem);
  const dLast = dv && dv[1] > dv[0] ? dl.lineOf(dv[1] - 1) : d.line;
  const at = dl.end(dLast);
  return `${dstText.slice(0, at)}\n${reindentAll(block, d.col - s.col)}${dstText.slice(at)}`;
}

export interface CopyResult {
  text: string;
  // 実際に書き換えた場所。相手側に途中の枝から無いときは、いちばん上の無い枝(segs より短い)
  applied: PathSeg[];
}

// srcText の segs の値を、dstText の同じ場所へ写す。無ければキーごと足す。
export function copyPath(srcText: string, dstText: string, segs: readonly PathSeg[]): CopyResult {
  if (segs.length === 0) throw new Error("キーが指定されていません");
  const srcDoc = parse(srcText, "コピー元");
  const dstDoc = parse(dstText, "コピー先");
  const src = locate(srcDoc, segs);
  if (src.depth !== segs.length) throw new Error(`コピー元に ${pathToString(segs)} がありません`);
  const dst = locate(dstDoc, segs);

  let text: string;
  let applied: PathSeg[];
  if (dst.depth === segs.length) {
    applied = [...segs];
    text = replaceValue(srcText, dstText, src, dst);
  } else {
    // 相手側は segs[dst.depth] から先が無い。その枝ごと足す
    applied = segs.slice(0, dst.depth + 1);
    if (!dst.parent) throw new Error(`コピー先の ${pathToString(segs.slice(0, dst.depth))} の形が違います`);
    const srcSub = locate(srcDoc, applied);
    const seg = applied[applied.length - 1];
    if (typeof seg === "number") {
      if (!isSeq(dst.parent) || !isSeq(srcSub.parent)) throw new Error("配列ではありません");
      text = appendToBlockSeq(srcText, dstText, srcSub.parent, dst.parent, seg);
    } else {
      if (!isMap(dst.parent) || !isMap(srcSub.parent) || !srcSub.pair) throw new Error("マップではありません");
      text = dst.parent.flow
        ? insertIntoFlowMap(srcText, dstText, srcSub.parent, dst.parent, srcSub.pair)
        : insertIntoBlockMap(srcText, dstText, srcSub.parent, dst.parent, srcSub.pair);
    }
  }

  // 確認: 読み直して「applied の場所だけが src の値になった」こと
  const before = loadYaml(dstText);
  const want = withValueAt(before, applied, getAt(loadYaml(srcText), applied));
  let after: unknown;
  try {
    after = loadYaml(text);
  } catch (err) {
    throw new Error(`${pathToString(applied)}: 書き換えた結果が yaml として読めません (${(err as Error).message})`);
  }
  if (canonical(after) !== canonical(want)) {
    throw new Error(`${pathToString(applied)}: 書き換えた結果が想定と違うので中止しました(手で直してください)`);
  }
  return { text, applied };
}

export interface CopyManyResult {
  text: string;
  applied: string[]; // 書き換えたキー
  errors: { path: string; error: string }[];
}

// 複数のキーを順に写す。1 つ失敗しても残りは続ける(失敗したキーは errors に)。
export function copyPaths(srcText: string, dstText: string, paths: readonly PathSeg[][]): CopyManyResult {
  let text = dstText;
  const applied: string[] = [];
  const errors: { path: string; error: string }[] = [];
  for (const segs of paths) {
    try {
      const r = copyPath(srcText, text, segs);
      text = r.text;
      applied.push(pathToString(r.applied));
    } catch (err) {
      errors.push({ path: pathToString(segs), error: (err as Error).message });
    }
  }
  return { text, applied, errors };
}

// segs の値の文字だけを valueText に差し替える(用途別パラメータの画面用)。
// 行末のコメント・前後のコメント行・ほかのキーはそのまま残る。対象は葉だけ
// (スカラーと `[a, b]` の形の配列)。差し替えたあと読み直して「そのキーだけが
// その値になった」ことを確かめ、違えば例外にする(呼ぶ側はファイルへ書かない)。
export function setValueText(text: string, segs: readonly PathSeg[], valueText: string): string {
  const label = pathToString(segs);
  const piece = valueText.trim();
  if (piece === "") throw new Error(`${label}: 値が空です`);
  if (/[\r\n]/.test(piece)) throw new Error(`${label}: 値に改行は入れられません`);
  if (piece.includes("#")) throw new Error(`${label}: 値に # は入れられません(コメントは yaml の編集画面で)`);
  let value: unknown;
  try {
    value = (loadYaml(`v: ${piece}`) as { v: unknown }).v ?? null;
  } catch {
    throw new Error(`${label}: 値として読めません: ${piece}`);
  }
  if (value !== null && typeof value === "object" && !Array.isArray(value)) {
    throw new Error(`${label}: この形の値は入れられません: ${piece}`);
  }

  const doc = parse(text, "yaml");
  const loc = locate(doc, segs);
  if (loc.depth !== segs.length || !loc.pair) throw new Error(`${label} がありません`);
  if (isMap(loc.node) || (isSeq(loc.node) && !loc.node.flow)) {
    throw new Error(`${label}: この形の値は yaml の編集画面で直してください`);
  }
  const span = valueSpan(text, loc.node);
  let out: string;
  if (span && span[1] > span[0]) {
    if (text.slice(span[0], span[1]).includes("\n")) {
      throw new Error(`${label}: 複数行の値は yaml の編集画面で直してください`);
    }
    out = text.slice(0, span[0]) + piece + text.slice(span[1]);
  } else {
    // 空の値(`key:`): `:` の直後へ入れる
    const at = afterColon(text, loc.pair);
    out = `${text.slice(0, at)} ${piece}${text.slice(at)}`;
  }

  const want = withValueAt(loadYaml(text), segs, value);
  let after: unknown;
  try {
    after = loadYaml(out);
  } catch (err) {
    throw new Error(`${label}: 書き換えた結果が yaml として読めません (${(err as Error).message})`);
  }
  if (canonical(after) !== canonical(want)) {
    throw new Error(`${label}: 書き換えた結果が想定と違うので中止しました(yaml の編集画面で直してください)`);
  }
  return out;
}

// キーごとの行番号(1 始まり)。編集画面で「他の機体と違う行」に印を付けるため。
// 枝(マップ・配列の要素)も葉も入る。構文エラーの途中の下書きでは空を返す。
export function pathLines(text: string): Map<string, number> {
  const out = new Map<string, number>();
  let doc: Doc;
  try {
    doc = parseDocument(text);
  } catch {
    return out;
  }
  if (doc.errors.length > 0) return out;
  const lines = linesOf(text);
  const walk = (node: unknown, segs: PathSeg[]) => {
    if (isMap(node)) {
      for (const pair of node.items) {
        const r = (pair.key as Node | null)?.range;
        const next = [...segs, keyName(pair)];
        if (r) out.set(pathToString(next), lines.lineOf(r[0]) + 1);
        walk(pair.value, next);
      }
    } else if (isSeq(node) && node.items.some((it) => isMap(it))) {
      node.items.forEach((it, i) => {
        const r = (it as Node | null)?.range;
        const next = [...segs, i];
        if (r) out.set(pathToString(next), lines.lineOf(r[0]) + 1);
        walk(it, next);
      });
    }
  };
  walk(doc.contents, []);
  return out;
}

