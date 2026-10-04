import { randomUUID } from "node:crypto";
import fs from "node:fs";
import path from "node:path";
import { load as loadYaml } from "js-yaml";
import type {
  CompareFile,
  CompareResult,
  FileValues,
  PropagateSuggestion,
  SyncResult,
} from "./machine-compare-shared";
import { isFileSpecific, isSpecific, type PathSeg } from "./machine-shared";
import { profileDir, readRegistry } from "./machines";
import { BASE_FILES } from "./serial-manager";
import { compareTrees } from "./yaml-compare";
import { copyPaths } from "./yaml-patch";

// 機体どうしのパラメータの比較と同期(サーバー専用)。
// 比べるのは機体へ送るファイルだけ: profile 直下の system / hardware / am32 と、
// profile/<mode>/*.yaml。同じ場所にある迷路の yaml やテンプレートは比べない。

// profile からの相対パス。"hardware.yaml" か "<mode>/<name>.yaml" だけを通す。
function resolveFile(machine: string, file: string): string {
  const m = /^(?:([\w-]+)\/)?([\w.-]+\.yaml)$/.exec(file);
  if (!m || file.includes("..")) throw new Error("不正なファイル名です");
  if (!m[1] && !BASE_FILES.includes(m[2])) throw new Error("比較の対象ではないファイルです");
  return path.join(profileDir(machine), file);
}

function listFiles(machine: string, mode: string): string[] {
  const dir = profileDir(machine);
  const out = BASE_FILES.filter((f) => fs.existsSync(path.join(dir, f)));
  const modeDir = path.join(dir, mode);
  if (fs.existsSync(modeDir)) {
    for (const f of fs.readdirSync(modeDir).sort()) if (f.endsWith(".yaml")) out.push(`${mode}/${f}`);
  }
  return out;
}

function readDoc(machine: string, file: string): { doc?: unknown; error: string | null; present: boolean } {
  const p = resolveFile(machine, file);
  if (!fs.existsSync(p)) return { present: false, error: null };
  try {
    return { present: true, error: null, doc: loadYaml(fs.readFileSync(p, "utf-8")) ?? null };
  } catch (err) {
    return { present: true, error: (err as Error).message.split("\n")[0] };
  }
}

function compareFile(machines: string[], file: string): CompareFile {
  const registry = readRegistry();
  const docs = machines.map((m) => readDoc(m, file));
  const { entries, total } = compareTrees(docs.map((d) => d.doc));
  return {
    file,
    present: docs.map((d) => d.present),
    errors: docs.map((d) => d.error),
    total,
    fileSpecific: isFileSpecific(registry.specific, file),
    // ファイルごと無い機体がいるときは、キーの差は出さない(「ファイルなし」として出す)
    entries: docs.every((d) => d.present)
      ? entries.map((e) => ({ ...e, specific: isSpecific(registry.specific, file, e.path) }))
      : [],
  };
}

export function compareMachines(mode: string, onlyFile?: string): CompareResult {
  const machines = readRegistry().machines.map((m) => m.id);
  const files: string[] = [];
  if (onlyFile) files.push(onlyFile);
  else {
    const seen = new Set<string>();
    for (const m of machines) {
      for (const f of listFiles(m, mode)) {
        if (!seen.has(f)) {
          seen.add(f);
          files.push(f);
        }
      }
    }
    // base(profile 直下)→ mode の順、その中は名前順
    files.sort((a, b) => Number(a.includes("/")) - Number(b.includes("/")) || a.localeCompare(b, "en", { numeric: true }));
  }
  return { machines, files: files.map((f) => compareFile(machines, f)) };
}

export function readFileValues(file: string): FileValues {
  const machines = readRegistry().machines.map((m) => m.id);
  const values: Record<string, unknown> = {};
  for (const m of machines) {
    const d = readDoc(m, file);
    if (d.present && !d.error) values[m] = d.doc;
  }
  return { machines, values };
}

// ===== 同期(元に戻せるように、書き換える前の中身を覚えておく) =====

interface UndoRecord {
  file: string; // 絶対パス
  before: string | null; // null = ファイルが無かった
  after: string;
  label: string;
}

const undoStore = new Map<string, UndoRecord>();
const UNDO_KEEP = 30;

function remember(rec: UndoRecord): string {
  const token = randomUUID();
  undoStore.set(token, rec);
  while (undoStore.size > UNDO_KEEP) undoStore.delete(undoStore.keys().next().value as string);
  return token;
}

export function undoSync(token: string): string {
  const rec = undoStore.get(token);
  if (!rec) throw new Error("元に戻せる記録がありません(サーバーを再起動した・古い操作)");
  const now = fs.existsSync(rec.file) ? fs.readFileSync(rec.file, "utf-8") : null;
  // そのあと誰かが書き換えていたら、上書きで消さない
  if (now !== rec.after) throw new Error("そのあとファイルが変わっているので元に戻せません");
  if (rec.before === null) fs.unlinkSync(rec.file);
  else fs.writeFileSync(rec.file, rec.before, "utf-8");
  undoStore.delete(token);
  return rec.label;
}

// from の機体の値を to の機体へ写す(コメントは残す。lib/yaml-patch.ts)。
export function syncKeys(req: { from: string; to: string; file: string; paths: PathSeg[][] }): SyncResult {
  if (req.from === req.to) throw new Error("同じ機体です");
  const srcPath = resolveFile(req.from, req.file);
  const dstPath = resolveFile(req.to, req.file);
  if (!fs.existsSync(srcPath)) throw new Error(`${req.from} に ${req.file} がありません`);
  if (!fs.existsSync(dstPath)) throw new Error(`${req.to} に ${req.file} がありません(ファイルごとコピーしてください)`);
  const before = fs.readFileSync(dstPath, "utf-8");
  const r = copyPaths(fs.readFileSync(srcPath, "utf-8"), before, req.paths);
  if (r.text === before) return { applied: r.applied, errors: r.errors, undo: null };
  fs.writeFileSync(dstPath, r.text, "utf-8");
  const undo = remember({
    file: dstPath,
    before,
    after: r.text,
    label: `${req.to}/${req.file}`,
  });
  return { applied: r.applied, errors: r.errors, undo };
}

// ファイルごと写す(相手に無いファイルを作る・コメントも含めて丸ごと揃える)。
export function copyWholeFile(req: { from: string; to: string; file: string }): SyncResult {
  if (req.from === req.to) throw new Error("同じ機体です");
  const srcPath = resolveFile(req.from, req.file);
  const dstPath = resolveFile(req.to, req.file);
  if (!fs.existsSync(srcPath)) throw new Error(`${req.from} に ${req.file} がありません`);
  const text = fs.readFileSync(srcPath, "utf-8");
  const before = fs.existsSync(dstPath) ? fs.readFileSync(dstPath, "utf-8") : null;
  if (before === text) return { applied: [], errors: [], undo: null };
  fs.mkdirSync(path.dirname(dstPath), { recursive: true });
  fs.writeFileSync(dstPath, text, "utf-8");
  const undo = remember({ file: dstPath, before, after: text, label: `${req.to}/${req.file}` });
  return { applied: ["(ファイル全体)"], errors: [], undo };
}

// 編集画面で保存したとき: 変えたキーのうち、ほかの機体も「変える前と同じ値」だったもの
// (= そろっていた値)を、同じ変更を入れる候補として返す。固有のキーは出さない。
// 元から違っていたキーは機体ごとの値とみなして出さない。
export function suggestPropagation(machine: string, file: string, oldText: string | null, newText: string): PropagateSuggestion[] {
  if (oldText === null || oldText === newText) return [];
  let oldDoc: unknown;
  let newDoc: unknown;
  try {
    oldDoc = loadYaml(oldText) ?? null;
    newDoc = loadYaml(newText) ?? null;
  } catch {
    return [];
  }
  const registry = readRegistry();
  const out: PropagateSuggestion[] = [];
  for (const other of registry.machines) {
    if (other.id === machine) continue;
    const d = readDoc(other.id, file);
    if (!d.present || d.error) continue;
    const { entries } = compareTrees([oldDoc, newDoc, d.doc]);
    const paths = entries
      .filter((e) => e.values[0] !== e.values[1] && e.values[1] !== null && e.values[2] === e.values[0])
      .filter((e) => !isSpecific(registry.specific, file, e.path))
      .map((e) => ({ segs: e.segs, path: e.path, from: e.values[0], to: e.values[1] }));
    if (paths.length > 0) out.push({ machine: other.id, paths });
  }
  return out;
}
