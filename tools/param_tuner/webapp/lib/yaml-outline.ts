import { isMap, isScalar, isSeq, parseDocument, type Node, type Pair } from "yaml";
import { pathToString, type PathSeg } from "./machine-shared";

// yaml の「葉(値)の一覧」を、書かれている文字のまま取り出す(Node 依存なし)。
// 用途別パラメータの画面用: 値は読んだ数ではなく元の文字(0.0000067 / 0x0d)で持ち、
// 説明は yaml のコメントから取る(キーの行末と、キーの直前に続くコメント行)。
// 書き換えは lib/yaml-patch.ts の setValueText(値の文字だけを差し替える)。

export interface OutlineLeaf {
  segs: PathSeg[];
  path: string; // a.b.c
  line: number; // キーの行(1 始まり)
  raw: string; // 値の文字(空の値は "")
  // scalar = 1 つの値、list = [a, b, c] の形の配列(要素はスカラーだけ)、
  // other = それ以外(- の配列・マップの配列など。この画面では直さない)
  kind: "scalar" | "list" | "other";
  items?: string[]; // list のとき、要素ごとの文字
  inline: string; // 行末のコメント(# は除く)
  above: string[]; // 直前に続くコメント行(# は除く。空行を挟まないもの)
  // 自分にコメントが無く、前のキーから空行なしで続いているとき、そのまとまりの
  // 先頭のキー(そのコメントがまとまり全体の説明になっている)
  lead: string | null;
  parent: string | null; // 親のマップ(最上位なら null)
}

export interface OutlineMap {
  segs: PathSeg[];
  path: string;
  line: number;
  inline: string;
  above: string[];
  parent: string | null;
}

export interface Outline {
  leaves: OutlineLeaf[];
  maps: OutlineMap[];
}

const stripHash = (s: string) => s.replace(/^\s*#\s?/, "");

function keyName(pair: Pair): string {
  const k = pair.key;
  return String(isScalar(k) ? k.value : k);
}

// 構文エラーのときは例外(呼ぶ側が「yaml を読めません」と出す)。
export function outlineYaml(text: string): Outline {
  const doc = parseDocument(text, { prettyErrors: true });
  if (doc.errors.length > 0) throw new Error(doc.errors[0].message);

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
  const lineEnd = (line: number) => (line + 1 < starts.length ? starts[line + 1] - 1 : text.length);
  const lineText = (line: number) => text.slice(starts[line], lineEnd(line));
  const isComment = (line: number) => lineText(line).trimStart().startsWith("#");

  const aboveOf = (keyLine: number): string[] => {
    const out: string[] = [];
    for (let l = keyLine - 1; l >= 0 && isComment(l); l--) out.push(stripHash(lineText(l)));
    return out.reverse();
  };
  // pos から行末までがコメントなら、その中身
  const inlineFrom = (pos: number): string => {
    const rest = text.slice(pos, lineEnd(lineOf(pos))).trim();
    return rest.startsWith("#") ? rest.replace(/^#\s?/, "").trim() : "";
  };

  const leaves: OutlineLeaf[] = [];
  const maps: OutlineMap[] = [];

  const walk = (node: unknown, segs: PathSeg[], parent: string | null) => {
    if (!isMap(node)) return;
    // 直前のキー(空行・コメントを挟まずに続いているかを見る)
    let prevEndLine = -2;
    let prevLead: string | null = null;
    for (const pair of node.items) {
      const kr = (pair.key as Node | null)?.range;
      if (!kr) continue;
      const next = [...segs, keyName(pair)];
      const path = pathToString(next);
      const keyLine = lineOf(kr[0]);
      const above = aboveOf(keyLine);
      const value = pair.value as Node | null;
      const colon = text.indexOf(":", kr[1]);
      const afterColon = colon < 0 ? kr[1] : colon + 1;

      if (isMap(value) && !value.flow) {
        maps.push({ segs: next, path, line: keyLine + 1, inline: inlineFrom(afterColon), above, parent });
        walk(value, next, path);
        // マップの後ろに空行なしで続くキーは、別のまとまりとして扱う
        prevEndLine = -2;
        prevLead = null;
        continue;
      }

      const r = value?.range;
      let start = r ? r[0] : afterColon;
      let end = r ? r[1] : afterColon;
      while (end > start && /\s/.test(text[end - 1])) end--;
      if (end <= start) start = end = afterColon;
      const raw = text.slice(start, end);
      let kind: OutlineLeaf["kind"] = "scalar";
      let items: string[] | undefined;
      if (isSeq(value)) {
        if (value.flow && value.items.every((it) => isScalar(it) && (it as Node).range)) {
          kind = "list";
          items = value.items.map((it) => {
            const ir = (it as Node).range!;
            return text.slice(ir[0], ir[1]).trim();
          });
        } else {
          kind = "other";
        }
      } else if (isMap(value) || raw.includes("\n")) {
        kind = "other";
      }
      const endLine = end > start ? lineOf(end - 1) : keyLine;
      const lead: string | null = above.length === 0 && keyLine === prevEndLine + 1 ? prevLead : null;
      leaves.push({
        segs: next,
        path,
        line: keyLine + 1,
        raw,
        kind,
        items,
        inline: kind === "other" ? "" : inlineFrom(end),
        above,
        lead,
        parent,
      });
      prevEndLine = endLine;
      // まとまりの先頭 = コメントを持つキー。コメントの無いキーが続く間は引き継ぐ
      prevLead = above.length > 0 ? path : lead;
    }
  };
  walk(doc.contents, [], null);
  return { leaves, maps };
}
