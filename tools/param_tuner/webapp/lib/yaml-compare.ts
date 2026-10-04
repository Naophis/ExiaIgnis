import { pathToString, type PathSeg } from "./machine-shared";

// 機体どうしの yaml の比較(Node 依存なし。サーバーと編集画面の両方で使う)。
// 比べるのは js-yaml で読んだ値 = 機体へ送る JSON と同じもの。コメントや書き方
// (45 と 45.0、並び順)の違いは差にしない。

type PlainMap = Record<string, unknown>;

const isMap = (v: unknown): v is PlainMap => v !== null && typeof v === "object" && !Array.isArray(v);
// マップを含む配列(v_prof・profile_idx・exec_prof など)は要素ごとに降りる。
// スカラーだけの配列(ゲインの [a, b]、補正テーブル、ファイル名の一覧)は 1 つの値として比べる。
const isMapSeq = (v: unknown): v is unknown[] => Array.isArray(v) && v.some(isMap);

type Kind = "map" | "seq" | "leaf";
const kindOf = (v: unknown): Kind => (isMap(v) ? "map" : isMapSeq(v) ? "seq" : "leaf");

// 比較用の文字列(マップは鍵順に並べる)。表示にもそのまま使う。
export function canonical(v: unknown): string {
  if (isMap(v)) {
    return `{${Object.keys(v)
      .sort()
      .map((k) => `${JSON.stringify(k)}:${canonical(v[k])}`)
      .join(",")}}`;
  }
  if (Array.isArray(v)) return `[${v.map(canonical).join(",")}]`;
  if (typeof v === "number" && !Number.isFinite(v)) return String(v);
  return JSON.stringify(v ?? null);
}

export function countLeaves(v: unknown): number {
  if (isMap(v)) return Object.values(v).reduce((n: number, x) => n + countLeaves(x), 0);
  if (isMapSeq(v)) return v.reduce((n: number, x) => n + countLeaves(x), 0);
  return 1;
}

export interface DiffEntry {
  segs: PathSeg[];
  path: string; // a.b[3].c
  // 機体の並び順。canonical() の文字列。null = その機体にこのキー(枝)が無い
  values: (string | null)[];
  leafCount: number; // このエントリが表す値の数(枝ごと無いときは中の値の数)
}

// docs[i] = 機体 i の yaml を読んだ値(undefined = 比べない)。差のあるキーだけ返す。
// 枝ごと無い機体があるときは、いちばん上の無い枝で 1 件にまとめる。
export function compareTrees(docs: unknown[]): { entries: DiffEntry[]; total: number } {
  const entries: DiffEntry[] = [];
  let total = 0;

  const walk = (segs: PathSeg[], vals: unknown[]) => {
    const present = vals.filter((v) => v !== undefined);
    if (present.length === 0) return;
    const kinds = present.map(kindOf);
    const allSame = present.length === vals.length && kinds.every((k) => k === kinds[0]);
    if (allSame && kinds[0] === "map") {
      const keys: string[] = [];
      const seen = new Set<string>();
      for (const v of vals as PlainMap[]) {
        for (const k of Object.keys(v)) {
          if (!seen.has(k)) {
            seen.add(k);
            keys.push(k);
          }
        }
      }
      for (const k of keys) {
        walk(
          [...segs, k],
          (vals as PlainMap[]).map((v) => (Object.prototype.hasOwnProperty.call(v, k) ? (v[k] ?? null) : undefined)),
        );
      }
      return;
    }
    if (allSame && kinds[0] === "seq") {
      const len = Math.max(...(vals as unknown[][]).map((v) => v.length));
      for (let i = 0; i < len; i++) {
        walk(
          [...segs, i],
          (vals as unknown[][]).map((v) => (i < v.length ? (v[i] ?? null) : undefined)),
        );
      }
      return;
    }
    const leafCount = Math.max(...present.map(countLeaves));
    total += leafCount;
    const strs = vals.map((v) => (v === undefined ? null : canonical(v)));
    if (strs.every((s) => s !== null && s === strs[0])) return;
    entries.push({ segs, path: pathToString(segs), values: strs, leafCount });
  };

  if (docs.some((d) => d !== undefined)) {
    // ファイルの中身が空(null)やスカラーでも落ちないように、最上位はそのまま渡す
    walk(
      [],
      docs.map((d) => (d === undefined ? undefined : (d ?? null))),
    );
  }
  return { entries, total };
}

export function getAt(doc: unknown, segs: readonly PathSeg[]): unknown {
  let cur: unknown = doc;
  for (const s of segs) {
    if (typeof s === "number") {
      if (!Array.isArray(cur) || s >= cur.length) return undefined;
      cur = cur[s];
    } else {
      if (!isMap(cur) || !Object.prototype.hasOwnProperty.call(cur, s)) return undefined;
      cur = cur[s];
    }
    if (cur === undefined) cur = null;
  }
  return cur;
}

// doc の写しを作り、segs の位置へ value を入れて返す(途中の枝は既にあること)。
export function withValueAt(doc: unknown, segs: readonly PathSeg[], value: unknown): unknown {
  if (segs.length === 0) return value;
  const [head, ...rest] = segs;
  if (typeof head === "number") {
    if (!Array.isArray(doc)) throw new Error("配列ではありません");
    const next = doc.slice();
    next[head] = withValueAt(doc[head], rest, value);
    return next;
  }
  if (!isMap(doc)) throw new Error("マップではありません");
  return { ...doc, [head]: withValueAt(doc[head], rest, value) };
}

// 値を 1 行で出す(長い配列・枝は途中まで)。
export function displayValue(canon: string | null, max = 48): string {
  if (canon === null) return "(なし)";
  let s = canon;
  if (s.startsWith("[") || s.startsWith("{")) s = s.replace(/,/g, ", ").replace(/":/g, '": ').replace(/"/g, "");
  else if (s.startsWith('"') && s.endsWith('"')) s = s.slice(1, -1);
  return s.length > max ? `${s.slice(0, max - 1)}…` : s;
}
