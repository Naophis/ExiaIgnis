import fs from "node:fs";
import path from "node:path";
import { load as loadYaml } from "js-yaml";
import { suggestPropagation } from "./machine-compare";
import { pathToString } from "./machine-shared";
import { profileDir, readRegistry } from "./machines";
import {
  checkValueText,
  parseCatalog,
  resolveLayout,
  type Catalog,
  type PurposeData,
  type PurposeEdit,
  type PurposeFile,
  type PurposeSaveFile,
  type PurposeSaveResult,
} from "./param-purpose-shared";
import { TOOL_ROOT } from "./paths";
import { serialManager } from "./serial-manager";
import { outlineYaml, type Outline } from "./yaml-outline";
import { setValueText } from "./yaml-patch";

// 用途別パラメータ(サーバー専用)。分類は tools/param_tuner/param_groups.yaml
// (全機体で共通。機体のデータではないので DATA_ROOT ではなく TOOL_ROOT)。
// 開くたびに読むので、分類ファイルを直したら画面を読み直すだけで反映される。

export const CATALOG_FILE = path.join(TOOL_ROOT, "param_groups.yaml");
const CATALOG_LABEL = "tools/param_tuner/param_groups.yaml";
const EMPTY: Catalog = { files: {}, purposes: [] };

function readCatalog(): { catalog: Catalog; error: string | null } {
  try {
    return { catalog: parseCatalog(loadYaml(fs.readFileSync(CATALOG_FILE, "utf-8"))), error: null };
  } catch (err) {
    return { catalog: EMPTY, error: (err as Error).message.split("\n")[0] };
  }
}

// 分類ファイルに載っているファイルだけを触る(profile の外へ出ない)
function resolveFile(catalog: Catalog, machine: string, file: string): string {
  if (!Object.values(catalog.files).includes(file)) throw new Error(`用途別の対象ではないファイルです: ${file}`);
  return path.join(profileDir(machine), file);
}

function readFile(catalog: Catalog, machine: string, file: string): PurposeFile & { text: string | null } {
  const p = resolveFile(catalog, machine, file);
  if (!fs.existsSync(p)) return { file, exists: false, outline: null, error: null, text: null };
  const text = fs.readFileSync(p, "utf-8");
  try {
    return { file, exists: true, outline: outlineYaml(text), error: null, text };
  } catch (err) {
    return { file, exists: true, outline: null, error: (err as Error).message.split("\n")[0], text };
  }
}

export function readPurposeData(machine: string): PurposeData {
  const { catalog, error } = readCatalog();
  const rels = [...new Set(Object.values(catalog.files))];
  const files = rels.map((rel) => readFile(catalog, machine, rel));
  const outlines: Record<string, Outline | null> = {};
  for (const f of files) outlines[f.file] = f.outline;

  const others: Record<string, Record<string, unknown>> = {};
  const machines = readRegistry().machines;
  for (const rel of rels) {
    others[rel] = {};
    for (const m of machines) {
      if (m.id === machine) continue;
      const p = path.join(profileDir(m.id), rel);
      if (!fs.existsSync(p)) continue;
      try {
        others[rel][m.id] = loadYaml(fs.readFileSync(p, "utf-8")) ?? null;
      } catch {
        // 読めない機体は比べない(機体比較の画面が理由を出す)
      }
    }
  }
  return {
    machine,
    catalogFile: CATALOG_LABEL,
    catalogError: error,
    files: files.map(({ file, exists, outline, error: e }) => ({ file, exists, outline, error: e })),
    layout: resolveLayout(catalog, outlines),
    others,
  };
}

// 値を書き換える。ファイルごとに「全部入るか、何も書かないか」(途中まで入った状態にしない)。
// 書くのは値の文字だけで、コメントやほかのキーは変わらない(lib/yaml-patch.ts の setValueText)。
export function applyPurposeEdits(machine: string, edits: PurposeEdit[]): PurposeSaveResult {
  const { catalog, error } = readCatalog();
  if (error) throw new Error(`分類ファイルを読めません: ${error}`);
  const byFile = new Map<string, PurposeEdit[]>();
  for (const e of edits) {
    if (!byFile.has(e.file)) byFile.set(e.file, []);
    byFile.get(e.file)!.push(e);
  }
  const out: PurposeSaveFile[] = [];
  for (const [file, list] of byFile) {
    const res: PurposeSaveFile = { file, saved: false, applied: [], errors: [], propagate: [] };
    out.push(res);
    const cur = readFile(catalog, machine, file);
    if (cur.text === null || !cur.outline) {
      res.errors.push({ path: "", error: cur.error ? `yaml を読めません: ${cur.error}` : "ファイルがありません" });
      continue;
    }
    const leaves = new Map(cur.outline.leaves.map((l) => [l.path, l]));
    let text = cur.text;
    for (const e of list) {
      const p = pathToString(e.segs);
      try {
        const leaf = leaves.get(p);
        if (!leaf) throw new Error("キーがありません(yaml が変わっています。読み直してください)");
        if (leaf.kind === "other") throw new Error("この形の値は yaml の編集画面で直してください");
        if (leaf.raw !== e.was) {
          throw new Error(`別の所で値が変わっています(いまの値 ${leaf.raw})。読み直してください`);
        }
        const check = checkValueText(e.text, leaf.raw, (src) => loadYaml(src));
        if (!check.ok) throw new Error(check.error ?? "値が不正です");
        text = setValueText(text, e.segs, e.text);
        res.applied.push(p);
      } catch (err) {
        const msg = (err as Error).message;
        res.errors.push({ path: p, error: msg.startsWith(`${p}: `) ? msg.slice(p.length + 2) : msg });
      }
    }
    if (res.errors.length > 0) {
      res.applied = [];
      continue;
    }
    if (text !== cur.text) {
      fs.writeFileSync(resolveFile(catalog, machine, file), text, "utf-8");
      // yaml の編集画面の保存と同じく、コンソールに残す(どのキーを変えたかも)
      serialManager.emit("log", `[edit] saved: ${machine}/${file} (${res.applied.join(", ")})`);
      res.propagate = suggestPropagation(machine, file, cur.text, text);
    }
    res.saved = true;
  }
  return { files: out };
}
