import type { PropagateSuggestion } from "./machine-compare-shared";
import type { PathSeg } from "./machine-shared";
import type { Outline, OutlineLeaf, OutlineMap } from "./yaml-outline";

// 用途別パラメータ(クライアント / サーバー共用、Node 依存なし)。
//
// hardware.yaml / offset.yaml / sensor.yaml のキーを「何を調整するときに触るか」で
// 並べ直して出す画面のための、分類の読み方と当てはめ。分類そのものは
// tools/param_tuner/param_groups.yaml(全機体で共通)。yaml の中身は変えない。

// ===== 分類ファイル(param_groups.yaml)の形 =====

export interface CatalogTable {
  // 同じ長さの配列を、列をそろえた表として出す(先頭が横軸)
  table: string[];
  label?: string;
}
export type CatalogItem = string | CatalogTable;

export interface CatalogSection {
  label: string;
  note?: string;
  keys: CatalogItem[];
}

export interface CatalogPurpose {
  id: string;
  label: string;
  note?: string;
  sections: CatalogSection[];
}

export interface Catalog {
  // 短い名前 → profile からの相対パス(hw: hardware.yaml)
  files: Record<string, string>;
  purposes: CatalogPurpose[];
}

const isObj = (v: unknown): v is Record<string, unknown> => v !== null && typeof v === "object" && !Array.isArray(v);

// 読んだ yaml の形を確かめる。おかしければ、どこがおかしいかを書いた例外。
export function parseCatalog(doc: unknown): Catalog {
  if (!isObj(doc)) throw new Error("最上位がマップではありません");
  const files: Record<string, string> = {};
  if (!isObj(doc.files)) throw new Error("files がありません");
  for (const [alias, rel] of Object.entries(doc.files)) {
    if (typeof rel !== "string" || !/^(?:[\w-]+\/)?[\w.-]+\.yaml$/.test(rel)) {
      throw new Error(`files.${alias}: ファイル名が不正です`);
    }
    files[alias] = rel;
  }
  if (!Array.isArray(doc.purposes)) throw new Error("purposes がありません");
  const seen = new Set<string>();
  const checkKey = (where: string, key: unknown): string => {
    if (typeof key !== "string") throw new Error(`${where}: キーは "hw:名前" の形の文字列で書きます`);
    const m = /^([\w-]+):(.+)$/.exec(key);
    if (!m || !(m[1] in files)) throw new Error(`${where}: ${key} のファイルの短い名前が files にありません`);
    if (!/^[\w*]+(\.[\w*]+)*$/.test(m[2])) throw new Error(`${where}: ${key} のキーの書き方が不正です`);
    return key;
  };
  const purposes = doc.purposes.map((p, pi): CatalogPurpose => {
    if (!isObj(p)) throw new Error(`purposes[${pi}] がマップではありません`);
    const id = typeof p.id === "string" && p.id ? p.id : null;
    if (!id) throw new Error(`purposes[${pi}]: id がありません`);
    if (seen.has(id)) throw new Error(`purposes: id ${id} が重なっています`);
    seen.add(id);
    if (typeof p.label !== "string") throw new Error(`${id}: label がありません`);
    if (!Array.isArray(p.sections)) throw new Error(`${id}: sections がありません`);
    const sections = p.sections.map((s, si): CatalogSection => {
      const where = `${id}.sections[${si}]`;
      if (!isObj(s) || typeof s.label !== "string") throw new Error(`${where}: label がありません`);
      if (!Array.isArray(s.keys)) throw new Error(`${where} (${s.label}): keys がありません`);
      const keys = s.keys.map((k): CatalogItem => {
        if (isObj(k)) {
          if (!Array.isArray(k.table) || k.table.length < 1) throw new Error(`${where} (${s.label}): table が空です`);
          const table = k.table.map((t) => {
            const key = checkKey(`${where} (${s.label})`, t);
            if (key.includes("*")) throw new Error(`${where} (${s.label}): table には * を使えません (${key})`);
            return key;
          });
          return { table, label: typeof k.label === "string" ? k.label : undefined };
        }
        return checkKey(`${where} (${s.label})`, k);
      });
      return { label: s.label, note: typeof s.note === "string" ? s.note : undefined, keys };
    });
    return { id, label: p.label, note: typeof p.note === "string" ? p.note : undefined, sections };
  });
  return { files, purposes };
}

// ===== 当てはめ =====

export interface LeafRef {
  file: string; // profile からの相対パス
  path: string;
}

export type LayoutEntry =
  | ({ type: "leaf" } & LeafRef)
  | { type: "table"; label: string | null; rows: LeafRef[] }; // rows[0] が横軸

export interface LayoutSection {
  label: string;
  note: string | null;
  entries: LayoutEntry[];
}

export interface LayoutPurpose {
  id: string;
  label: string;
  note: string | null;
  sections: LayoutSection[];
}

export interface Layout {
  purposes: LayoutPurpose[];
  // どの用途にも入らなかったキー(yaml にあって分類に無い = ファームに足したばかりのキーなど)
  unclassified: LeafRef[];
  // 分類に書いてあるのに、この機体の yaml に無いキー
  unmatched: string[];
}

interface Occurrence {
  item: number; // 何番目に出てきたか(分類ファイルの順)
  text: string;
  file: string;
  segs: string[];
  exact: boolean;
  literal: number; // * 以外の文字数(細かい指定ほど大きい)
}

function globMatch(pattern: string, s: string): boolean {
  if (!pattern.includes("*")) return pattern === s;
  const re = new RegExp(`^${pattern.replace(/[.+^${}()|[\]\\]/g, "\\$&").replace(/\*/g, ".*")}$`);
  return re.test(s);
}

// 1 つのキーがどこに入るかの決まり:
//   - いちばん細かい指定が勝つ(段数 a.b > a、名前そのまま > * 付き、* 以外の文字が多い方)。
//   - 名前そのままの指定は、同じ指定を書いた所すべてに出る(横軸を 2 つの表で使うなど)。
//   - * 付きの指定は、分類ファイルで先に書いた 1 か所だけ。
// 例: `hw:gyro_pid` と `hw:gyro_pid.mpc_*` があれば、mpc_* は後者だけに出る。
export function resolveLayout(catalog: Catalog, outlines: Record<string, Outline | null>): Layout {
  const occs: Occurrence[] = [];
  const itemOcc: number[][][][] = []; // [purpose][section][item] → occurrence の番号(table は複数)
  catalog.purposes.forEach((p, pi) => {
    itemOcc[pi] = [];
    p.sections.forEach((s, si) => {
      itemOcc[pi][si] = [];
      s.keys.forEach((k, ki) => {
        const texts = typeof k === "string" ? [k] : k.table;
        itemOcc[pi][si][ki] = texts.map((text) => {
          const colon = text.indexOf(":");
          const pattern = text.slice(colon + 1);
          occs.push({
            item: occs.length,
            text,
            file: catalog.files[text.slice(0, colon)],
            segs: pattern.split("."),
            exact: !pattern.includes("*"),
            literal: pattern.replace(/\*/g, "").length,
          });
          return occs.length - 1;
        });
      });
    });
  });

  const assigned: LeafRef[][] = occs.map(() => []);
  const unclassified: LeafRef[] = [];
  for (const [file, outline] of Object.entries(outlines)) {
    if (!outline) continue;
    const fileOccs = occs.filter((o) => o.file === file);
    for (const leaf of outline.leaves) {
      const segs = leaf.segs.map(String);
      let best: Occurrence[] = [];
      let bestRank: [number, number, number] | null = null;
      for (const o of fileOccs) {
        if (o.segs.length > segs.length || !o.segs.every((s, i) => globMatch(s, segs[i]))) continue;
        const rank: [number, number, number] = [o.segs.length, o.exact ? 1 : 0, o.literal];
        const cmp = bestRank === null ? 1 : rank[0] - bestRank[0] || rank[1] - bestRank[1] || rank[2] - bestRank[2];
        if (cmp > 0) {
          best = [o];
          bestRank = rank;
        } else if (cmp === 0) {
          best.push(o);
        }
      }
      if (best.length === 0) {
        unclassified.push({ file, path: leaf.path });
        continue;
      }
      const targets = best[0].exact ? best : [best[0]];
      for (const o of targets) assigned[o.item].push({ file, path: leaf.path });
    }
  }

  const unmatched: string[] = [];
  const purposes = catalog.purposes.map((p, pi): LayoutPurpose => ({
    id: p.id,
    label: p.label,
    note: p.note ?? null,
    sections: p.sections.map((s, si): LayoutSection => {
      const entries: LayoutEntry[] = [];
      s.keys.forEach((k, ki) => {
        const ids = itemOcc[pi][si][ki];
        if (typeof k === "string") {
          const leaves = assigned[ids[0]];
          if (leaves.length === 0 && outlines[occs[ids[0]].file]) unmatched.push(k);
          for (const l of leaves) entries.push({ type: "leaf", ...l });
          return;
        }
        const rows: LeafRef[] = [];
        for (const id of ids) {
          // 表の行は 1 つのキー(配列)。マップを指したときなど、複数に当たったら表にしない
          if (assigned[id].length === 1) rows.push(assigned[id][0]);
          else if (outlines[occs[id].file]) unmatched.push(occs[id].text);
        }
        if (rows.length > 0) entries.push({ type: "table", label: k.label ?? null, rows });
      });
      return { label: s.label, note: s.note ?? null, entries };
    }),
  }));
  return { purposes, unclassified, unmatched };
}

// ===== API でやり取りする形 =====

export interface PurposeFile {
  file: string; // profile からの相対パス
  exists: boolean;
  outline: Outline | null;
  error: string | null; // 読めなかった理由(構文エラーなど)
}

export interface PurposeData {
  machine: string;
  catalogFile: string; // 分類ファイルの場所(画面に出す)
  catalogError: string | null;
  files: PurposeFile[];
  layout: Layout;
  // ほかの機体の、同じファイルを読んだ値(file → 機体 id → 値)
  others: Record<string, Record<string, unknown>>;
}

export interface PurposeEdit {
  file: string;
  segs: PathSeg[];
  text: string; // 新しい値の文字
  was: string; // 画面が読んだときの値の文字(そのあと別の所で変わっていたら書かない)
}

export interface PurposeSaveFile {
  file: string;
  saved: boolean;
  applied: string[];
  errors: { path: string; error: string }[];
  propagate: PropagateSuggestion[];
}

export interface PurposeSaveResult {
  files: PurposeSaveFile[];
}

// ===== 画面の部品が使う小さな関数 =====

export const leafKey = (file: string, path: string) => `${file}\n${path}`;

// 0 / 1 の切り替えとして出すキー(enable・*_enable・enable_*)
export function isFlagLeaf(leaf: OutlineLeaf): boolean {
  if (leaf.kind !== "scalar" || (leaf.raw !== "0" && leaf.raw !== "1")) return false;
  const name = String(leaf.segs[leaf.segs.length - 1]);
  return name === "enable" || name.endsWith("_enable") || name.startsWith("enable_");
}

// 左右の組: 名前の中の 1 か所だけが l / r(left / right、L45 / R45 など)で違うキー。
// 左のキーなら { side: "l", partner: 右のキーの名前, stem: 組の名前 } を返す。
export function sidePartner(name: string): { side: "l" | "r"; partner: string; stem: string } | null {
  const tokens = name.split("_");
  for (let i = 0; i < tokens.length; i++) {
    const t = tokens[i];
    let other: string | null = null;
    let side: "l" | "r" = "l";
    let m: RegExpExecArray | null;
    if ((m = /^l(\d*)$/.exec(t))) other = `r${m[1]}`;
    else if ((m = /^r(\d*)$/.exec(t))) [other, side] = [`l${m[1]}`, "r"];
    else if ((m = /^L(\d+)$/.exec(t))) other = `R${m[1]}`;
    else if ((m = /^R(\d+)$/.exec(t))) [other, side] = [`L${m[1]}`, "r"];
    else if ((m = /^left(\d*)$/.exec(t))) other = `right${m[1]}`;
    else if ((m = /^right(\d*)$/.exec(t))) [other, side] = [`left${m[1]}`, "r"];
    if (other === null) continue;
    const swap = [...tokens];
    swap[i] = other;
    const stem = [...tokens];
    stem[i] = side === "l" ? `${t} / ${other}` : `${other} / ${t}`;
    return { side, partner: swap.join("_"), stem: stem.join("_") };
  }
  return null;
}

// コメントの中の「以前の値」(コメントアウトした `key: 値`)の行か。
// 「ang_th: 収束とみなす…」のような説明の行と区別するため、値が数・配列・空のものだけ。
export function isOldValueLine(s: string): boolean {
  const m = /^\s*(?:#\s*)*[A-Za-z_]\w*:(.*)$/.exec(s);
  if (!m) return false;
  const value = m[1].replace(/\s#.*$/, "").trim();
  return value === "" || /^(-?[\d.]+(e-?\d+)?|0x[0-9a-fA-F]+|\[.*\]|true|false)$/.test(value);
}
const stripDate = (s: string) => s.replace(/^\s*\d{4}-\d{2}-\d{2}(\s+\d{1,2}(:\d{2}| 時))?\s*[::]?\s*/, "");

// 一覧の 1 行に出す短い説明: 行末のコメント → 直前のコメントの最初の文 →
// まとまり・親のコメントの中の「名前 : 説明」の行。
export function shortDesc(
  leaf: OutlineLeaf,
  lookup: { leaf: (path: string) => OutlineLeaf | undefined; map: (path: string) => OutlineMap | undefined },
): string {
  if (leaf.inline) return leaf.inline;
  const prose = leaf.above.filter((l) => l.trim() !== "" && !isOldValueLine(l));
  if (prose.length > 0) return stripDate(prose[0]).trim();
  // 「  depth_min : 谷底が…」の形の行を、まとまり・親のコメントから探す
  const name = String(leaf.segs[leaf.segs.length - 1]);
  const tokens = name.split("_");
  const sources: string[][] = [];
  if (leaf.lead) sources.push(lookup.leaf(leaf.lead)?.above ?? []);
  if (leaf.parent) sources.push(lookup.map(leaf.parent)?.above ?? []);
  for (const lines of sources) {
    for (let from = 0; from < tokens.length; from++) {
      const cand = tokens.slice(from).join("_").replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
      const re = new RegExp(`^\\s*${cand}(?:/\\w+)*\\s*[::]\\s*(.+)$`);
      for (const l of lines) {
        if (isOldValueLine(l)) continue;
        const m = re.exec(l);
        if (m) return m[1].trim();
      }
    }
  }
  return "";
}

export interface ParsedText {
  ok: boolean;
  error: string | null;
  value: unknown;
}

// 入力欄の文字を値として確かめる(元の値と同じ種類か)。parse は js-yaml の load を渡す。
export function checkValueText(
  text: string,
  original: string,
  parse: (src: string) => unknown,
): ParsedText {
  const t = text.trim();
  if (t === "") return { ok: false, error: "値が空です", value: null };
  if (/[\r\n#]/.test(t)) return { ok: false, error: "# と改行は入れられません", value: null };
  const read = (s: string): { v: unknown; ok: boolean } => {
    try {
      return { v: (parse(`v: ${s}`) as { v: unknown }).v ?? null, ok: true };
    } catch {
      return { v: null, ok: false };
    }
  };
  const now = read(t);
  if (!now.ok) return { ok: false, error: "値として読めません", value: null };
  const kind = (v: unknown) => (Array.isArray(v) ? "array" : v === null ? "null" : typeof v);
  const was = read(original.trim());
  if (was.ok && was.v !== null && kind(now.v) !== kind(was.v)) {
    const names: Record<string, string> = { number: "数値", string: "文字", boolean: "真偽", array: "配列", object: "マップ" };
    return { ok: false, error: `${names[kind(was.v)] ?? kind(was.v)}を入れてください`, value: now.v };
  }
  if (Array.isArray(now.v) && Array.isArray(was.v) && was.v.every((x) => typeof x === "number")) {
    if (!now.v.every((x) => typeof x === "number")) return { ok: false, error: "配列の中は数値だけです", value: now.v };
  }
  if (kind(now.v) === "object") return { ok: false, error: "この形の値は入れられません", value: now.v };
  return { ok: true, error: null, value: now.v };
}

// ↑↓ キーで最後の桁を 1 つ動かす(0.0375 → 0.0376)。big なら 1 つ上の桁。
// 10 進の数として書かれた値だけ(指数表記・16 進は動かさない)。
export function nudgeNumber(text: string, dir: 1 | -1, big: boolean): string | null {
  const m = /^(-?)(\d+)(?:\.(\d+))?$/.exec(text.trim());
  if (!m) return null;
  const decimals = m[3]?.length ?? 0;
  const step = Math.pow(10, -decimals) * (big ? 10 : 1);
  const next = Number(text) + dir * step;
  if (!Number.isFinite(next)) return null;
  const out = next.toFixed(decimals);
  // −0.000 のような表示にしない
  return Number(out) === 0 ? (0).toFixed(decimals) : out;
}
