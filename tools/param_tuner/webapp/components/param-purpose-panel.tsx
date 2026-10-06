"use client";

import { memo, useCallback, useEffect, useMemo, useRef, useState, type KeyboardEvent, type ReactNode } from "react";
import { load as loadYaml } from "js-yaml";
import { toast } from "sonner";
import { MachineChip } from "@/components/machine-chip";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { ResizableHandle, ResizablePanel, ResizablePanelGroup } from "@/components/ui/resizable";
import { SendCancelled, apiFetch, findMachine, useMachines } from "@/lib/machine-client";
import type { PropagateSuggestion, SyncResult } from "@/lib/machine-compare-shared";
import { isSpecific } from "@/lib/machine-shared";
import {
  checkValueText,
  isFlagLeaf,
  isOldValueLine,
  leafKey,
  nudgeNumber,
  shortDesc,
  sidePartner,
  type LayoutPurpose,
  type LayoutSection,
  type PurposeData,
  type PurposeEdit,
  type PurposeSaveResult,
} from "@/lib/param-purpose-shared";
import { cn } from "@/lib/utils";
import { canonical, displayValue, getAt } from "@/lib/yaml-compare";
import type { OutlineLeaf, OutlineMap } from "@/lib/yaml-outline";

// 用途別パラメータ: hardware.yaml / offset.yaml / sensor.yaml のキーを「何を調整するときに
// 触るか」で並べ直して出し、その場で値を直す。分類は tools/param_tuner/param_groups.yaml
// (全機体で共通)、説明は yaml のコメントから取る。保存は値の文字だけを差し替える
// (コメント・並び・ほかのキーは変わらない)。yaml の編集画面はそのまま残してある。

interface Props {
  mode: string;
  onClose: () => void;
  // 保存した(未送信の印や、ほかの画面の値を取り直す)
  onFilesChanged: () => void;
  // yaml の編集画面で、そのキーの行を開く(file は profile からの相対パス)
  onOpenYaml: (file: string, line: number) => void;
}

const PREFS_KEY = "exia-param-purpose-v1";
const UNCLASSIFIED = "__unclassified";

// 閉じて開き直しても未保存の入力が残るように、機体ごとに覚えておく(ページを読み込み直すと消える)。
// was = 入力したときの保存済みの値。開き直したとき yaml の値が変わっていたら、その入力は捨てる。
const draftStore = new Map<string, Record<string, { text: string; was: string }>>();

type Filter = "all" | "dirty" | "diff";

// これより長い値(桁の多い数など)は、列をそろえずに中身の長さの欄で出す
const WIDE_LEN = 12;

interface Item {
  key: string;
  file: string;
  alias: string;
  leaf: OutlineLeaf;
  desc: string;
  hay: string; // 絞り込み用(小文字)
  savedCanon: string; // 保存済みの値(比べる形)
  others: { machine: string; canon: string | null }[];
  specific: boolean;
}

type Row =
  | { kind: "head"; key: string; file: string; alias: string; map: OutlineMap; desc: string; enable: Item | null }
  | { kind: "single"; key: string; item: Item; indent: boolean }
  | { kind: "pair"; key: string; l: Item; r: Item; label: string; indent: boolean }
  | { kind: "table"; key: string; label: string | null; rows: Item[] };

interface Block {
  purpose: LayoutPurpose;
  sections: { section: LayoutSection; rows: Row[] }[];
}

const FILE_TAG: Record<number, string> = {
  0: "text-sky-300 border-sky-300/40",
  1: "text-emerald-300 border-emerald-300/40",
  2: "text-rose-300 border-rose-300/40",
};

// その行が持つ値(見出しは enable だけ)
const rowItems = (row: Row): Item[] =>
  row.kind === "single" ? [row.item] : row.kind === "pair" ? [row.l, row.r] : row.kind === "table" ? row.rows : row.enable ? [row.enable] : [];

function splitList(text: string): string[] | null {
  const t = text.trim();
  if (!t.startsWith("[") || !t.endsWith("]")) return null;
  const inner = t.slice(1, -1).trim();
  return inner === "" ? [] : inner.split(",").map((s) => s.trim());
}

// 書き方(空白)だけの違いは変更としない
function sameText(a: string, b: string): boolean {
  const la = splitList(a);
  const lb = splitList(b);
  if (la && lb) return la.join(",") === lb.join(",");
  return a.trim() === b.trim();
}

function canonOfText(text: string): string | null {
  try {
    return canonical((loadYaml(`v: ${text}`) as { v: unknown }).v ?? null);
  } catch {
    return null;
  }
}

const displayWidth = (t: string) => {
  let w = 0;
  for (const ch of t) w += ch.charCodeAt(0) > 0xff ? 2 : 1;
  return w;
};

// yaml のコメントは 80 桁前後で手で折り返してある。右の欄は狭いので、折り返しで切れただけの
// 行(長い行の続きで、字下げが同じ)はつなげて、欄の幅で折り返し直す。表や箇条書きの行
// (数字・記号で始まる行、字下げの違う行、短い行の次)はそのまま。
function reflowComment(lines: string[]): string[] {
  const out: string[] = [];
  const indent = (t: string) => t.length - t.trimStart().length;
  for (const line of lines) {
    const prev = out[out.length - 1];
    const joinable =
      prev !== undefined &&
      line.trim() !== "" &&
      !isOldValueLine(prev) &&
      !isOldValueLine(line) &&
      indent(line) === indent(prev) &&
      displayWidth(prev) >= 64 &&
      !/[。.::;)」』]$/.test(prev.trimEnd()) &&
      !/^[\d\-*・#\[|+]/.test(line.trimStart());
    if (!joinable) {
      out.push(line);
      continue;
    }
    const a = prev.trimEnd();
    const b = line.trimStart();
    out[out.length - 1] = a + (/[\w,]$/.test(a) && /^\w/.test(b) ? " " : "") + b;
  }
  return out;
}

// 表の行の名前: 共通の頭を落として、違う所だけ出す(sensor_deg_limitter_ + v / str / piller)
function tableRowLabels(paths: string[]): string[] {
  if (paths.length < 2) return paths;
  let n = 0;
  const first = paths[0];
  while (n < first.length && paths.every((p) => p[n] === first[n])) n++;
  const dot = first.lastIndexOf(".", n - 1);
  if (dot >= 0) return paths.map((p) => p.slice(dot + 1));
  while (n > 0 && first[n - 1] !== "_") n--;
  return paths.map((p) => p.slice(n) || p);
}

function ConfirmButton({ label, confirmLabel, title, onConfirm }: { label: string; confirmLabel: string; title?: string; onConfirm: () => void }) {
  const [armed, setArmed] = useState(false);
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);
  useEffect(() => () => void (timer.current && clearTimeout(timer.current)), []);
  return (
    <Button
      size="sm"
      variant={armed ? "destructive" : "ghost"}
      title={title}
      onBlur={() => setArmed(false)}
      onClick={() => {
        if (!armed) {
          setArmed(true);
          if (timer.current) clearTimeout(timer.current);
          timer.current = setTimeout(() => setArmed(false), 4000);
          return;
        }
        setArmed(false);
        onConfirm();
      }}
    >
      {armed ? confirmLabel : label}
    </Button>
  );
}

function Toggle({ on, dirty, onChange, title }: { on: boolean; dirty: boolean; onChange: (next: boolean) => void; title?: string }) {
  return (
    <button
      type="button"
      role="switch"
      aria-checked={on}
      title={title}
      onClick={(e) => {
        e.stopPropagation();
        onChange(!on);
      }}
      className={cn(
        "inline-flex h-5 w-[3.25rem] shrink-0 items-center rounded-full border px-0.5 text-[0.65rem] font-semibold transition-colors",
        on ? "justify-end border-primary-bright/60 bg-primary-bright/25 text-primary-bright" : "justify-start border-border bg-muted/40 text-muted-foreground",
        dirty && "ring-1 ring-primary-bright",
      )}
    >
      {on && <span className="px-1">ON</span>}
      <span className={cn("size-3.5 rounded-full", on ? "bg-primary-bright" : "bg-muted-foreground/60")} />
      {!on && <span className="px-1">OFF</span>}
    </button>
  );
}

export const ParamPurposePanel = memo(function ParamPurposePanel({ mode, onClose, onFilesChanged, onOpenYaml }: Props) {
  const { registry, current, board, guardedSend } = useMachines();
  const machine = findMachine(registry, current);
  const [data, setData] = useState<PurposeData | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [edits, setEdits] = useState<Record<string, string>>({});
  const [purposeId, setPurposeId] = useState<string | null>(null);
  const [query, setQuery] = useState("");
  const [filter, setFilter] = useState<Filter>("all");
  const [focus, setFocus] = useState<string | null>(null); // 行の key(右の欄に出す)
  const [busy, setBusy] = useState<"save" | "send" | null>(null);
  const [propagate, setPropagate] = useState<{ file: string; s: PropagateSuggestion }[]>([]);
  const restored = useRef(false);
  const listRef = useRef<HTMLDivElement | null>(null);

  const load = useCallback(async () => {
    try {
      const res = await apiFetch("/api/param-purpose");
      const d = await res.json();
      if (!res.ok) throw new Error(d.error ?? "読み込みに失敗しました");
      setData(d as PurposeData);
      setError(null);
    } catch (err) {
      setError((err as Error).message);
    }
  }, []);

  useEffect(() => {
    // eslint-disable-next-line react-hooks/set-state-in-effect
    void load();
    // 別のエディタ(VSCode など)で yaml を直して戻ってきたときに読み直す
    const onFocus = () => void load();
    window.addEventListener("focus", onFocus);
    return () => window.removeEventListener("focus", onFocus);
  }, [load]);

  useEffect(() => {
    try {
      const saved = JSON.parse(localStorage.getItem(PREFS_KEY) ?? "null") as { purpose?: string } | null;
      // eslint-disable-next-line react-hooks/set-state-in-effect
      if (saved?.purpose) setPurposeId(saved.purpose);
    } catch {
      // 覚えていなくても動く
    }
  }, []);

  // ===== 読んだものを引きやすい形にする =====

  const model = useMemo(() => {
    if (!data) return null;
    const aliasOf = new Map<string, { alias: string; idx: number }>();
    data.files.forEach((f, i) => aliasOf.set(f.file, { alias: f.file.replace(/^.*\//, "").replace(/\.yaml$/, ""), idx: i }));
    const items = new Map<string, Item>();
    const maps = new Map<string, OutlineMap>();
    for (const f of data.files) {
      if (!f.outline) continue;
      const leafByPath = new Map(f.outline.leaves.map((l) => [l.path, l]));
      const mapByPath = new Map(f.outline.maps.map((m) => [m.path, m]));
      for (const m of f.outline.maps) maps.set(leafKey(f.file, m.path), m);
      const lookup = { leaf: (p: string) => leafByPath.get(p), map: (p: string) => mapByPath.get(p) };
      const otherDocs = data.others[f.file] ?? {};
      for (const leaf of f.outline.leaves) {
        const desc = shortDesc(leaf, lookup);
        const alias = aliasOf.get(f.file)!.alias;
        items.set(leafKey(f.file, leaf.path), {
          key: leafKey(f.file, leaf.path),
          file: f.file,
          alias,
          leaf,
          desc,
          hay: `${alias} ${leaf.path} ${desc} ${leaf.inline} ${leaf.above.join(" ")}`.toLowerCase(),
          savedCanon: canonOfText(leaf.raw) ?? "",
          others: Object.entries(otherDocs).map(([m, doc]) => {
            const v = getAt(doc, leaf.segs);
            return { machine: m, canon: v === undefined ? null : canonical(v) };
          }),
          specific: isSpecific(registry.specific, f.file, leaf.path),
        });
      }
    }

    const purposes: LayoutPurpose[] = [...data.layout.purposes];
    if (data.layout.unclassified.length > 0) {
      const byFile = new Map<string, LayoutSection>();
      for (const u of data.layout.unclassified) {
        if (!byFile.has(u.file)) byFile.set(u.file, { label: u.file, note: null, entries: [] });
        byFile.get(u.file)!.entries.push({ type: "leaf", ...u });
      }
      purposes.push({
        id: UNCLASSIFIED,
        label: "未分類",
        note: `分類(${data.catalogFile})に載っていないキー。そのファイルに書くと用途に入ります`,
        sections: [...byFile.values()],
      });
    }

    const blocks: Block[] = purposes.map((purpose) => ({
      purpose,
      sections: purpose.sections.map((section) => {
        const rows: Row[] = [];
        const inSection = new Set<string>();
        for (const e of section.entries) if (e.type === "leaf") inSection.add(leafKey(e.file, e.path));
        const used = new Set<string>();
        let curParent: string | null = null; // 直前の行の親(file\npath)
        for (const e of section.entries) {
          if (e.type === "table") {
            const tr = e.rows.map((r) => items.get(leafKey(r.file, r.path))).filter((x): x is Item => !!x);
            if (tr.length > 0) rows.push({ kind: "table", key: `t:${tr.map((r) => r.key).join("|")}`, label: e.label, rows: tr });
            curParent = null;
            continue;
          }
          const k = leafKey(e.file, e.path);
          const item = items.get(k);
          if (!item || used.has(k)) continue;
          const parentKey = item.leaf.parent ? leafKey(e.file, item.leaf.parent) : null;
          if (parentKey !== curParent) {
            curParent = parentKey;
            const map = parentKey ? maps.get(parentKey) : undefined;
            if (parentKey && map) {
              // マップの見出し。enable(0/1)があれば見出しに切り替えを出す
              const en = items.get(leafKey(e.file, `${map.path}.enable`));
              const enable = en && inSection.has(en.key) && isFlagLeaf(en.leaf) ? en : null;
              if (enable) used.add(enable.key);
              const prose = map.above.filter((l) => l.trim() !== "" && !isOldValueLine(l));
              rows.push({
                kind: "head",
                key: `h:${parentKey}`,
                file: e.file,
                alias: item.alias,
                map,
                desc: map.inline || (prose[0] ?? "").replace(/^\s*\d{4}-\d{2}-\d{2}\s*[::]?\s*/, "").trim(),
                enable,
              });
              if (enable && enable.key === k) continue;
            }
          }
          if (used.has(k)) continue;
          const indent = parentKey !== null;
          // 左右の組は 1 行に
          const name = String(item.leaf.segs[item.leaf.segs.length - 1]);
          const sp = item.leaf.kind === "scalar" && !isFlagLeaf(item.leaf) ? sidePartner(name) : null;
          if (sp) {
            const pk = leafKey(e.file, item.leaf.parent ? `${item.leaf.parent}.${sp.partner}` : sp.partner);
            const partner = items.get(pk);
            if (partner && inSection.has(pk) && !used.has(pk) && partner.leaf.kind === "scalar") {
              used.add(k);
              used.add(pk);
              const [l, r] = sp.side === "l" ? [item, partner] : [partner, item];
              rows.push({ kind: "pair", key: `p:${l.key}`, l, r, label: sp.stem, indent });
              continue;
            }
          }
          used.add(k);
          rows.push({ kind: "single", key: k, item, indent });
        }
        return { section, rows };
      }),
    }));
    return { items, maps, blocks, aliasOf };
  }, [data, registry.specific]);

  // 未保存の入力を、開き直したときに戻す(yaml の値が変わっていたものは捨てる)
  useEffect(() => {
    if (!model || !current || restored.current) return;
    restored.current = true;
    const saved = draftStore.get(current);
    if (!saved) return;
    const next: Record<string, string> = {};
    let dropped = 0;
    for (const [k, d] of Object.entries(saved)) {
      const item = model.items.get(k);
      if (item && item.leaf.raw === d.was) next[k] = d.text;
      else dropped++;
    }
    // eslint-disable-next-line react-hooks/set-state-in-effect
    if (Object.keys(next).length > 0) setEdits(next);
    if (dropped > 0) toast.info(`yaml の値が変わっていたので、未保存の入力 ${dropped} 個を捨てました`);
  }, [model, current]);

  useEffect(() => {
    if (!model || !current || !restored.current) return;
    const out: Record<string, { text: string; was: string }> = {};
    for (const [k, text] of Object.entries(edits)) {
      const item = model.items.get(k);
      if (item) out[k] = { text, was: item.leaf.raw };
    }
    if (Object.keys(out).length > 0) draftStore.set(current, out);
    else draftStore.delete(current);
  }, [edits, model, current]);

  // ===== 値 =====

  const textOf = useCallback((item: Item) => edits[item.key] ?? item.leaf.raw, [edits]);
  const isDirty = useCallback((item: Item) => item.key in edits, [edits]);

  const setEdit = useCallback(
    (item: Item, text: string) => {
      setEdits((prev) => {
        const next = { ...prev };
        if (sameText(text, item.leaf.raw)) delete next[item.key];
        else next[item.key] = text;
        return next;
      });
    },
    [],
  );

  const problems = useMemo(() => {
    const out = new Map<string, string>();
    if (!model) return out;
    for (const [k, text] of Object.entries(edits)) {
      const item = model.items.get(k);
      if (!item) continue;
      const c = checkValueText(text, item.leaf.raw, (src) => loadYaml(src));
      if (!c.ok) out.set(k, c.error ?? "値が不正です");
    }
    return out;
  }, [edits, model]);

  // ほかの機体と違うか(いま入力している値で比べる)
  const diffOf = useCallback(
    (item: Item): { machine: string; canon: string | null }[] => {
      if (item.others.length === 0) return [];
      const cur = item.key in edits ? canonOfText(edits[item.key]) : item.savedCanon;
      return item.others.filter((o) => o.canon !== cur);
    },
    [edits],
  );

  // ===== 絞り込み =====

  const needle = query.trim().toLowerCase();
  const crossing = needle !== "" || filter !== "all"; // 用途をまたいで探している
  // sectionHit = その見出しの名前が検索に合っている(「柱」で「柱の谷」の中身が全部出る)
  const matches = useCallback(
    (item: Item, sectionHit: boolean) => {
      if (needle && !sectionHit && !item.hay.includes(needle)) return false;
      if (filter === "dirty" && !isDirty(item)) return false;
      if (filter === "diff" && diffOf(item).length === 0) return false;
      return true;
    },
    [needle, filter, isDirty, diffOf],
  );

  const view = useMemo(() => {
    if (!model) return { blocks: [] as Block[], counts: new Map<string, { n: number; dirty: number; todo: number; hit: number }>() };
    const counts = new Map<string, { n: number; dirty: number; todo: number; hit: number }>();
    const out: Block[] = [];
    for (const b of model.blocks) {
      const c = { n: 0, dirty: 0, todo: 0, hit: 0 };
      const sections = b.sections
        .map(({ section, rows }) => {
          const kept: Row[] = [];
          const sectionHit = needle !== "" && section.label.toLowerCase().includes(needle);
          // 見出し(マップ)は、その下の行が 1 つでも残るときだけ出す
          let pendingHead: Row | null = null;
          for (const row of rows) {
            const its = rowItems(row);
            for (const it of its) {
              c.n++;
              if (isDirty(it)) c.dirty++;
              if (!it.specific && diffOf(it).length > 0) c.todo++;
            }
            if (!crossing) {
              kept.push(row);
              continue;
            }
            const hit = its.some((it) => matches(it, sectionHit));
            if (hit) c.hit += its.length;
            if (row.kind === "head") {
              pendingHead = hit ? null : row;
              if (hit) kept.push(row);
              continue;
            }
            if (row.kind === "table" || !row.indent) pendingHead = null;
            if (!hit) continue;
            if (pendingHead) {
              kept.push(pendingHead);
              pendingHead = null;
            }
            kept.push(row);
          }
          return { section, rows: kept };
        })
        .filter((s) => s.rows.length > 0);
      counts.set(b.purpose.id, c);
      if (sections.length > 0) out.push({ purpose: b.purpose, sections });
    }
    return { blocks: out, counts };
  }, [model, crossing, needle, matches, isDirty, diffOf]);

  const activeId = model?.blocks.some((b) => b.purpose.id === purposeId) ? purposeId : (model?.blocks[0]?.purpose.id ?? null);
  const shown = crossing ? view.blocks : view.blocks.filter((b) => b.purpose.id === activeId);

  const selectPurpose = (id: string) => {
    setPurposeId(id);
    setQuery("");
    setFilter("all");
    listRef.current?.scrollTo({ top: 0 });
    try {
      localStorage.setItem(PREFS_KEY, JSON.stringify({ purpose: id }));
    } catch {
      // 覚えられなくても動く
    }
  };

  // ===== 保存・送信 =====

  const dirtyCount = Object.keys(edits).length;
  const totalTodo = useMemo(() => {
    if (!model) return 0;
    let n = 0;
    for (const it of model.items.values()) if (!it.specific && diffOf(it).length > 0) n++;
    return n;
  }, [model, diffOf]);
  const hasOthers = registry.machines.length >= 2;

  const save = useCallback(
    async (thenSend: boolean) => {
      if (!model || busy || dirtyCount === 0) return;
      if (problems.size > 0) {
        toast.error(`入力に誤りがあります(${problems.size} 個)。赤い欄を直してください`);
        return;
      }
      setBusy(thenSend ? "send" : "save");
      try {
        const list: PurposeEdit[] = [];
        for (const [k, text] of Object.entries(edits)) {
          const item = model.items.get(k);
          if (item) list.push({ file: item.file, segs: item.leaf.segs, text: text.trim(), was: item.leaf.raw });
        }
        const res = await apiFetch("/api/param-purpose", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ edits: list }),
        });
        const result = (await res.json()) as PurposeSaveResult & { error?: string };
        if (!res.ok) throw new Error(result.error ?? "保存に失敗しました");
        const savedFiles = result.files.filter((f) => f.saved);
        const failed = result.files.filter((f) => !f.saved);
        if (savedFiles.length > 0) {
          const n = savedFiles.reduce((s, f) => s + f.applied.length, 0);
          toast.success(`${machine?.label ?? current} の ${savedFiles.map((f) => f.file).join("・")}: ${n} 個の値を保存しました`);
          const done = new Set(savedFiles.map((f) => f.file));
          setEdits((prev) => {
            const next: Record<string, string> = {};
            for (const [k, v] of Object.entries(prev)) if (!done.has(k.slice(0, k.indexOf("\n")))) next[k] = v;
            return next;
          });
          setPropagate(savedFiles.flatMap((f) => f.propagate.map((s) => ({ file: f.file, s }))));
        }
        for (const f of failed) {
          toast.error(`${f.file}: 保存できませんでした(このファイルは何も書き換えていません)`, {
            description: f.errors.slice(0, 4).map((e) => (e.path ? `${e.path}: ${e.error}` : e.error)).join("\n"),
          });
        }
        await load();
        onFilesChanged();
        if (thenSend && failed.length === 0) {
          for (const f of savedFiles) {
            const slash = f.file.indexOf("/");
            const body = slash < 0 ? { scope: "base", file: f.file } : { scope: "mode", file: f.file.slice(slash + 1) };
            try {
              const sr = await guardedSend((force) =>
                apiFetch("/api/send", {
                  method: "POST",
                  headers: { "Content-Type": "application/json" },
                  body: JSON.stringify({ mode, ...body, force }),
                }),
              );
              const sd = await sr.json();
              if (!sr.ok) throw new Error(sd.error ?? "送信に失敗しました");
              toast.success(`${f.file}: 送信完了`);
            } catch (err) {
              if (err instanceof SendCancelled) break;
              toast.error(`${f.file}: 保存しましたが、送信できませんでした: ${(err as Error).message}`, {
                description: "左の一覧の「未送信を送信」で送り直せます",
              });
              break;
            }
          }
          onFilesChanged();
        }
      } catch (err) {
        toast.error((err as Error).message);
      } finally {
        setBusy(null);
      }
    },
    [model, busy, dirtyCount, problems, edits, machine, current, load, onFilesChanged, guardedSend, mode],
  );

  useEffect(() => {
    const handler = (e: globalThis.KeyboardEvent) => {
      if (!(e.ctrlKey || e.metaKey) || e.key.toLowerCase() !== "s") return;
      e.preventDefault();
      void save(false);
    };
    window.addEventListener("keydown", handler);
    return () => window.removeEventListener("keydown", handler);
  }, [save]);

  // 保存した変更を、同じ値だったほかの機体にも入れる(yaml の編集画面と同じ扱い)
  const applyPropagation = async (file: string, s: PropagateSuggestion) => {
    if (!current) return;
    try {
      const res = await fetch("/api/machines/sync", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ from: current, to: s.machine, file, paths: s.paths.map((p) => p.segs) }),
      });
      const d = (await res.json()) as SyncResult & { error?: string };
      if (!res.ok) throw new Error(d.error ?? "書き換えに失敗しました");
      const label = findMachine(registry, s.machine)?.label ?? s.machine;
      if (d.applied.length > 0) {
        toast.success(`${label} の ${file} にも入れました: ${d.applied.join(", ")}`, {
          description: "機体へはまだ送っていません(その機体をつないで送信)",
          action: d.undo
            ? {
                label: "元に戻す",
                onClick: () =>
                  void fetch("/api/machines/sync", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ undo: d.undo }),
                  })
                    .then((r) => r.json())
                    .then((u) => {
                      if (u.error) toast.error(u.error);
                      else toast.success(`${u.label}: 元に戻しました`);
                      void load();
                      onFilesChanged();
                    }),
              }
            : undefined,
        });
      }
      if (d.errors.length > 0) toast.error(`${label}: ${d.errors.map((e) => `${e.path}: ${e.error}`).join(" / ")}`);
      setPropagate((prev) => prev.filter((x) => !(x.file === file && x.s.machine === s.machine)));
      await load();
      onFilesChanged();
    } catch (err) {
      toast.error((err as Error).message);
    }
  };

  // ===== 部品 =====

  const fileTag = (file: string, alias: string) => (
    <span
      className={cn("shrink-0 rounded border px-1 font-mono text-[0.6rem] leading-4", FILE_TAG[model?.aliasOf.get(file)?.idx ?? 0] ?? FILE_TAG[0])}
      title={file}
    >
      {alias}
    </span>
  );

  const onNumberKey = (e: KeyboardEvent<HTMLInputElement>, item: Item) => {
    if (e.key !== "ArrowUp" && e.key !== "ArrowDown") return;
    const next = nudgeNumber(textOf(item), e.key === "ArrowUp" ? 1 : -1, e.shiftKey);
    if (next === null) return;
    e.preventDefault();
    setEdit(item, next);
  };

  // width: 欄の幅。"fit" = 中身の長さに合わせる(配列・長い値)
  const valueInput = (item: Item, className: string, rowKey: string, width: string) => {
    const dirty = isDirty(item);
    const problem = problems.get(item.key);
    return (
      <input
        value={textOf(item)}
        spellCheck={false}
        aria-invalid={problem ? true : undefined}
        title={problem ?? (dirty ? `保存済みの値: ${item.leaf.raw}(↑↓ で最後の桁を 1 つ動かす)` : "↑↓ で最後の桁を 1 つ動かす(Shift で 10 倍)")}
        onChange={(e) => setEdit(item, e.target.value)}
        onKeyDown={(e) => onNumberKey(e, item)}
        onFocus={() => setFocus(rowKey)}
        data-param={item.leaf.path}
        style={{ width: width === "fit" ? `${Math.min(Math.max(textOf(item).length + 3, 14), 70)}ch` : width }}
        className={cn(
          "h-6 min-w-0 rounded border bg-input/30 px-1.5 text-right font-mono text-xs outline-none focus-visible:border-ring focus-visible:ring-2 focus-visible:ring-ring/50",
          dirty ? "border-primary-bright text-primary-bright" : "border-input",
          problem && "border-destructive text-destructive",
          className,
        )}
      />
    );
  };

  const diffChip = (its: Item[]) => {
    if (!hasOthers) return null;
    const parts: { label: string; value: string; specific: boolean }[] = [];
    for (const it of its) {
      for (const d of diffOf(it)) {
        parts.push({
          label: findMachine(registry, d.machine)?.label ?? d.machine,
          value: displayValue(d.canon, 14),
          specific: it.specific,
        });
      }
    }
    if (parts.length === 0) return null;
    const todo = parts.some((p) => !p.specific);
    const text = its.length === 1 && parts.length === 1 ? `${parts[0].label} ${parts[0].value}` : parts.map((p) => p.value).join(" / ");
    return (
      <span
        className={cn("max-w-44 shrink-0 truncate rounded border px-1 font-mono text-[0.65rem] leading-4", todo ? "border-amber-400/50 text-amber-300" : "border-violet-400/50 text-violet-300")}
        title={`ほかの機体の値(${todo ? "未整理 = そろえるか、機体比較で「固有」にする" : "固有 = 機体ごとに違ってよい値"})\n${parts.map((p) => `${p.label}: ${p.value}`).join("\n")}`}
        data-machine-diff={todo ? "todo" : "specific"}
      >
        {its.length === 1 && parts.length === 1 ? text : `${parts[0].label} ${text}`}
      </span>
    );
  };

  const revertButton = (its: Item[]) =>
    its.some(isDirty) ? (
      <button
        type="button"
        className="shrink-0 rounded px-1 text-xs text-muted-foreground hover:bg-muted hover:text-foreground"
        title={`保存済みの値に戻す(${its.filter(isDirty).map((it) => it.leaf.raw).join(" / ")})`}
        onClick={(e) => {
          e.stopPropagation();
          setEdits((prev) => {
            const next = { ...prev };
            for (const it of its) delete next[it.key];
            return next;
          });
        }}
      >
        ↺
      </button>
    ) : null;

  // 行は見出し(section)ごとの格子の 1 行(subgrid)。列の幅はその見出しの中でいちばん長い中身に
  // 合わせるので、左右の組が無い見出しでは値の欄が狭くなり、説明に幅が回る。
  const rowClass = (key: string, extra?: string) =>
    cn(
      "col-span-full grid cursor-default grid-cols-subgrid items-center rounded px-1.5 py-px hover:bg-muted/50",
      focus === key && "bg-primary-bright/10 ring-1 ring-primary-bright/40",
      extra,
    );

  // tag = 行ごとにファイルの札を出す(見出しの中に複数のファイルのキーがあるとき)、
  // pair = その見出しに左右の組がある(1 つの値の欄を、組の左の欄にそろえる)
  // width = 値の欄の幅(その見出しの中でいちばん長い値に合わせる)
  const renderRow = (row: Row, ctx: { tag: boolean; pair: boolean; width: string }) => {
    if (row.kind === "head") {
      const en = row.enable;
      const off = en ? textOf(en).trim() === "0" : false;
      return (
        <div
          key={row.key}
          className={cn("col-span-full mt-1 flex cursor-default items-center gap-2 rounded px-1.5 py-0.5 hover:bg-muted/50", focus === row.key && "bg-primary-bright/10 ring-1 ring-primary-bright/40")}
          onClick={() => setFocus(row.key)}
          data-param-head={row.map.path}
        >
          {ctx.tag && fileTag(row.file, row.alias)}
          <span className={cn("shrink-0 font-mono text-xs font-semibold", off ? "text-muted-foreground" : "text-foreground")}>{row.map.path}</span>
          {en && (
            <Toggle
              on={!off}
              dirty={isDirty(en)}
              onChange={(next) => setEdit(en, next ? "1" : "0")}
              title={`${en.leaf.path}(${isDirty(en) ? `保存済みの値: ${en.leaf.raw}` : "0 / 1"})`}
            />
          )}
          <span className="min-w-0 flex-1 truncate text-xs text-muted-foreground" title={row.desc}>
            {row.desc}
          </span>
          {en && diffChip([en])}
          {en && revertButton([en])}
        </div>
      );
    }
    if (row.kind === "table") return renderTable(row, ctx.tag);
    const its = rowItems(row);
    const pad = ctx.pair ? "pl-4" : "";
    const first = its[0];
    const name = row.kind === "pair" ? row.label : String(first.leaf.segs[first.leaf.segs.length - 1]);
    const dirty = its.some(isDirty);
    const descs = row.kind === "pair" && row.l.desc !== row.r.desc ? [row.l.desc && `L: ${row.l.desc}`, row.r.desc && `R: ${row.r.desc}`].filter(Boolean).join(" | ") : first.desc || (row.kind === "pair" ? row.r.desc : "");
    const wide = row.kind === "single" && (first.leaf.kind !== "scalar" || textOf(first).length > WIDE_LEN);
    return (
      <div key={row.key} className={rowClass(row.key)} onClick={() => setFocus(row.key)} data-param-row={first.leaf.path}>
        <span className={cn("flex min-w-0 items-center gap-1.5", row.indent && "pl-4")}>
          {ctx.tag && !row.indent && fileTag(first.file, first.alias)}
          <span className={cn("truncate font-mono text-xs", dirty && "text-primary-bright")} title={its.map((it) => it.leaf.path).join(" / ")}>
            {name}
          </span>
          {dirty && <span className="size-1.5 shrink-0 rounded-full bg-primary-bright" />}
        </span>
        {row.kind === "pair" ? (
          <span className="flex items-center gap-1">
            <span className="w-3 text-center text-[0.65rem] text-muted-foreground">L</span>
            {valueInput(row.l, "", row.key, ctx.width)}
            <span className="w-3 text-center text-[0.65rem] text-muted-foreground">R</span>
            {valueInput(row.r, "", row.key, ctx.width)}
          </span>
        ) : first.leaf.kind === "other" ? (
          <span className="col-span-2 truncate font-mono text-xs text-muted-foreground" title="この形の値は yaml の編集画面で直します(右の「yaml で開く」)">
            {first.leaf.raw.split("\n")[0]} …
          </span>
        ) : isFlagLeaf(first.leaf) ? (
          <span className={cn("flex items-center", pad)}>
            <Toggle on={textOf(first).trim() !== "0"} dirty={dirty} onChange={(next) => setEdit(first, next ? "1" : "0")} title={dirty ? `保存済みの値: ${first.leaf.raw}` : "0 / 1"} />
          </span>
        ) : wide ? (
          <span className={cn("col-span-2 flex min-w-0 items-center gap-2", pad)}>
            {valueInput(first, "shrink text-left", row.key, "fit")}
            <span className="min-w-12 flex-1 truncate text-xs text-muted-foreground" title={descs}>
              {descs}
            </span>
          </span>
        ) : (
          <span className={cn("flex items-center", pad)}>{valueInput(first, "", row.key, ctx.width)}</span>
        )}
        {!(row.kind === "single" && (wide || first.leaf.kind === "other")) && (
          <span className="min-w-0 truncate text-xs text-muted-foreground" title={descs}>
            {descs}
          </span>
        )}
        <span className="flex items-center justify-end gap-1">
          {diffChip(its)}
          {revertButton(its)}
        </span>
      </div>
    );
  };

  const renderTable = (row: Extract<Row, { kind: "table" }>, tag: boolean) => {
    const labels = tableRowLabels(row.rows.map((r) => r.leaf.path));
    const lists = row.rows.map((r) => splitList(textOf(r)) ?? r.leaf.items ?? []);
    const cols = Math.max(0, ...lists.map((l) => l.length));
    const uneven = lists.some((l) => l.length !== cols);
    const setCell = (ri: number, ci: number, v: string) => {
      const next = [...lists[ri]];
      next[ci] = v;
      setEdit(row.rows[ri], `[${next.join(", ")}]`);
    };
    const resize = (delta: 1 | -1) => {
      row.rows.forEach((r, ri) => {
        const cur = lists[ri];
        if (delta === 1) setEdit(r, `[${[...cur, cur[cur.length - 1] ?? "0"].join(", ")}]`);
        else if (cur.length > 1) setEdit(r, `[${cur.slice(0, -1).join(", ")}]`);
      });
    };
    return (
      <div key={row.key} className="col-span-full mt-1 flex flex-col gap-0.5 px-1.5 py-0.5" data-param-table={row.rows[0].leaf.path}>
        <div className="flex items-center gap-2 text-xs">
          {tag && fileTag(row.rows[0].file, row.rows[0].alias)}
          <span className="font-medium">{row.label ?? row.rows.map((r) => r.leaf.path).join(" / ")}</span>
          {uneven && (
            <span className="text-amber-300" title="行ごとの要素の数が違うと、ファームは表を使わない・途中までしか読まないことがあります">
              長さが違います({lists.map((l) => l.length).join(" / ")})
            </span>
          )}
          <div className="flex-1" />
          <button type="button" className="rounded border border-border px-1.5 text-xs text-muted-foreground hover:bg-muted hover:text-foreground" title="全部の行の最後に 1 列足す(最後の値の写し)" onClick={() => resize(1)}>
            + 列
          </button>
          <button type="button" className="rounded border border-border px-1.5 text-xs text-muted-foreground hover:bg-muted hover:text-foreground" title="全部の行の最後の 1 列を消す" onClick={() => resize(-1)}>
            − 列
          </button>
        </div>
        <div className="overflow-x-auto">
          <table className="border-separate border-spacing-x-0.5 border-spacing-y-px">
            <tbody>
              {row.rows.map((r, ri) => {
                const rk = `${row.key}#${ri}`;
                const dirty = isDirty(r);
                const problem = problems.get(r.key);
                return (
                  <tr key={r.key} className={cn(focus === rk && "bg-primary-bright/10")} onClick={() => setFocus(rk)}>
                    <th
                      className={cn("max-w-56 truncate pr-2 text-left font-mono text-xs font-normal", ri === 0 ? "text-muted-foreground" : "text-foreground", dirty && "text-primary-bright")}
                      title={`${r.leaf.path}${ri === 0 ? "(横軸)" : ""}${problem ? `\n${problem}` : ""}`}
                    >
                      {labels[ri]}
                      {ri === 0 && <span className="ml-1 text-[0.6rem]">横軸</span>}
                    </th>
                    {lists[ri].map((cell, ci) => (
                      <td key={ci}>
                        <input
                          value={cell}
                          spellCheck={false}
                          aria-invalid={problem ? true : undefined}
                          onChange={(e) => setCell(ri, ci, e.target.value)}
                          onFocus={() => setFocus(rk)}
                          onKeyDown={(e) => {
                            if (e.key !== "ArrowUp" && e.key !== "ArrowDown") return;
                            const next = nudgeNumber(cell, e.key === "ArrowUp" ? 1 : -1, e.shiftKey);
                            if (next === null) return;
                            e.preventDefault();
                            setCell(ri, ci, next);
                          }}
                          className={cn(
                            "h-6 w-[3.75rem] rounded border px-1 text-right font-mono text-xs outline-none focus-visible:border-ring focus-visible:ring-2 focus-visible:ring-ring/50",
                            ri === 0 ? "bg-muted/60 text-muted-foreground" : "bg-input/30",
                            dirty && (r.leaf.items?.[ci] ?? "") !== cell ? "border-primary-bright text-primary-bright" : "border-input",
                            problem && "border-destructive",
                          )}
                        />
                      </td>
                    ))}
                    <td className="pl-1">
                      <span className="flex items-center gap-1">
                        {diffChip([r])}
                        {revertButton([r])}
                      </span>
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      </div>
    );
  };

  // ===== 右の欄(選んだ行の yaml のコメントと、ほかの機体の値) =====

  const findFocus = (): { items: Item[]; map: OutlineMap | null; file: string | null } | null => {
    if (!model || !focus) return null;
    for (const b of model.blocks) {
      for (const s of b.sections) {
        for (const row of s.rows) {
          if (row.kind === "table") {
            if (!focus.startsWith(`${row.key}#`)) continue;
            const it = row.rows[Number(focus.slice(row.key.length + 1))];
            return it ? { items: [it], map: null, file: it.file } : null;
          }
          if (row.key !== focus) continue;
          if (row.kind === "head") return { items: row.enable ? [row.enable] : [], map: row.map, file: row.file };
          return { items: row.kind === "pair" ? [row.l, row.r] : [row.item], map: null, file: null };
        }
      }
    }
    return null;
  };
  const focusInfo = findFocus();

  const commentBlock = (lines: string[], inline: string) => {
    const all = reflowComment(inline ? [...lines, inline] : lines);
    if (all.length === 0) return null;
    return (
      <pre className="rounded border border-border/60 bg-background/40 p-1.5 font-mono text-[0.7rem] leading-relaxed whitespace-pre-wrap">
        {all.map((l, i) => (
          <span key={i} className={isOldValueLine(l) ? "text-muted-foreground/70" : undefined}>
            {l}
            {"\n"}
          </span>
        ))}
      </pre>
    );
  };

  const detailOf = (it: Item): ReactNode => {
    const dirty = isDirty(it);
    const problem = problems.get(it.key);
    const diffs = new Set(diffOf(it).map((d) => d.machine));
    const lead = it.leaf.lead ? model?.items.get(leafKey(it.file, it.leaf.lead)) : undefined;
    const parent = it.leaf.parent ? model?.maps.get(leafKey(it.file, it.leaf.parent)) : undefined;
    return (
      <div key={it.key} className="flex flex-col gap-1.5" data-param-detail={it.leaf.path}>
        <div className="flex items-center gap-1.5">
          {fileTag(it.file, it.alias)}
          <span className="min-w-0 flex-1 font-mono text-xs font-semibold break-all">{it.leaf.path}</span>
          <Button size="xs" variant="outline" title={`${it.file} の ${it.leaf.line} 行目を yaml の編集画面で開く(未保存の入力はこの画面に残ります)`} onClick={() => onOpenYaml(it.file, it.leaf.line)}>
            yaml で開く
          </Button>
        </div>
        <div className="flex flex-wrap items-center gap-x-3 gap-y-1 text-xs">
          <span>
            <span className="text-muted-foreground">{dirty ? "入力中 " : "値 "}</span>
            <span className={cn("font-mono", dirty && "text-primary-bright", problem && "text-destructive")}>{displayValue(textOf(it), 60)}</span>
          </span>
          {dirty && (
            <span>
              <span className="text-muted-foreground">保存済み </span>
              <span className="font-mono">{displayValue(it.leaf.raw, 60)}</span>
            </span>
          )}
          {problem && <span className="text-destructive">{problem}</span>}
        </div>
        {it.others.length > 0 && (
          <div className="flex flex-wrap items-center gap-x-2 gap-y-1 text-xs">
            {it.others.map((o) => (
              <span key={o.machine} className="flex items-center gap-1">
                <MachineChip machine={findMachine(registry, o.machine)} fallback={o.machine} solid={false} />
                <span className={cn("font-mono", diffs.has(o.machine) ? (it.specific ? "text-violet-300" : "text-amber-300") : "text-muted-foreground")}>
                  {displayValue(o.canon, 40)}
                </span>
              </span>
            ))}
            {diffs.size > 0 && (
              <span className={it.specific ? "text-violet-300" : "text-amber-300"} title="そろえる・固有にするのは、ヘッダーの「機体比較」で">
                {it.specific ? "固有" : "未整理"}
              </span>
            )}
          </div>
        )}
        {commentBlock(it.leaf.above, it.leaf.inline) ?? (!lead && !parent && <span className="text-xs text-muted-foreground">yaml にコメントはありません</span>)}
        {lead && lead.key !== it.key && (
          <>
            <span className="text-[0.7rem] text-muted-foreground">まとまりのコメント({lead.leaf.path} から続くキー)</span>
            {commentBlock(lead.leaf.above, "")}
          </>
        )}
        {parent && (parent.above.length > 0 || parent.inline) && (
          <>
            <span className="text-[0.7rem] text-muted-foreground">{parent.path} のコメント</span>
            {commentBlock(parent.above, parent.inline)}
          </>
        )}
      </div>
    );
  };

  // ===== 画面 =====

  const filterButtons: [Filter, string, string][] = [
    ["all", "用途ごと", "左の用途ごとに出す"],
    ["dirty", `未保存 ${dirtyCount}`, "この画面で直して、まだ保存していない値だけ(用途をまたいで出す)"],
  ];
  if (hasOthers) filterButtons.push(["diff", `機体差 ${totalTodo}`, "ほかの機体と値が違うキーだけ(数は未整理の分。固有も一覧には出す)"]);

  return (
    <Card
      className="flex h-full min-w-0 flex-1 flex-col overflow-hidden"
      style={machine ? { boxShadow: `inset 0 3px 0 ${machine.color}` } : undefined}
      data-param-purpose
    >
      <CardHeader className="flex flex-row flex-wrap items-center gap-2 space-y-0">
        <CardTitle className="flex items-center gap-1.5">
          <span className="shrink-0">用途別</span>
          {machine && <MachineChip machine={machine} title={`machines/${machine.id}/profile のファイルを直す`} />}
        </CardTitle>
        <Input
          placeholder="キー・説明を全用途から探す..."
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === "Escape") setQuery("");
          }}
          className="h-7 w-60"
          data-param-search
        />
        <div className="flex items-center gap-1">
          {filterButtons.map(([key, text, tip]) => (
            <Button key={key} size="xs" variant={filter === key ? "default" : "outline"} title={tip} onClick={() => setFilter(key)} data-filter={key}>
              {text}
            </Button>
          ))}
        </div>
        <div className="flex-1" />
        {problems.size > 0 && <span className="text-xs text-destructive">入力の誤り {problems.size}</span>}
        {dirtyCount > 0 && (
          <ConfirmButton label="取り消す" confirmLabel={`未保存の ${dirtyCount} 個を捨てる`} title="未保存の入力を全部捨てて、保存済みの値に戻す" onConfirm={() => setEdits({})} />
        )}
        <Button size="sm" variant="ghost" onClick={() => void load()} title="yaml と分類ファイルを読み直す(未保存の入力は残る)">
          読み直す
        </Button>
        <Button
          size="sm"
          variant={dirtyCount > 0 ? "default" : "outline"}
          disabled={busy !== null || dirtyCount === 0}
          onClick={() => void save(false)}
          title="値を yaml に書く(コメントや並びは変わらない)。Ctrl+S"
          data-param-save
        >
          {busy === "save" ? "保存中..." : dirtyCount > 0 ? `保存 (${dirtyCount})` : "保存"}
        </Button>
        <Button
          size="sm"
          variant="secondary"
          disabled={busy !== null || dirtyCount === 0 || !board.serial}
          onClick={() => void save(true)}
          title={!board.serial ? "基板がつながっていません(保存だけできます)" : "保存して、変えたファイルを機体へ送る"}
          data-param-save-send
        >
          {busy === "send" ? "送信中..." : "保存+送信"}
        </Button>
        <Button size="sm" variant="outline" onClick={onClose}>
          閉じる
        </Button>
      </CardHeader>
      <CardContent className="flex min-h-0 flex-1 flex-col gap-1.5 overflow-hidden">
        {propagate.map(({ file, s }) => {
          const target = findMachine(registry, s.machine);
          return (
            <div
              key={`${file}:${s.machine}`}
              className="flex flex-wrap items-center gap-2 rounded-md border border-primary-bright/50 bg-primary-bright/10 px-2 py-1 text-xs"
              data-propagate={s.machine}
            >
              <MachineChip machine={target} fallback={s.machine} />
              <span className="min-w-0 flex-1" title={s.paths.map((p) => `${p.path}: ${p.from ?? "(なし)"} → ${p.to}`).join("\n")}>
                も同じ値でした({file}): <span className="font-mono">{s.paths.slice(0, 6).map((p) => p.path).join(", ")}</span>
                {s.paths.length > 6 && ` ほか ${s.paths.length - 6} 個`}
              </span>
              <Button size="xs" onClick={() => void applyPropagation(file, s)}>
                {target?.label ?? s.machine} にも同じ変更を入れる
              </Button>
              <Button size="xs" variant="ghost" title="この機体だけの変更にする" onClick={() => setPropagate((prev) => prev.filter((x) => !(x.file === file && x.s.machine === s.machine)))}>
                入れない
              </Button>
            </div>
          );
        })}
        {error ? (
          <span className="text-sm text-destructive">{error}</span>
        ) : !data || !model ? (
          <span className="text-sm text-muted-foreground">読み込み中...</span>
        ) : (
          <>
            {data.catalogError && (
              <div className="rounded-md border border-destructive/60 bg-destructive/10 px-2 py-1 text-xs text-destructive" data-catalog-error>
                分類ファイル({data.catalogFile})を読めません: {data.catalogError}
              </div>
            )}
            {data.files
              .filter((f) => f.error || !f.exists)
              .map((f) => (
                <div key={f.file} className="rounded-md border border-amber-500/50 px-2 py-1 text-xs text-amber-200">
                  {f.file}: {f.exists ? `yaml を読めません(${f.error})。yaml の編集画面で直してください` : "この機体にファイルがありません"}
                </div>
              ))}
            <div className="flex min-h-0 flex-1 gap-2 overflow-hidden">
              {/* 左: 用途 */}
              <nav className="flex w-44 shrink-0 flex-col gap-0.5 overflow-auto rounded-md border p-1" data-purpose-nav>
                {model.blocks.map((b) => {
                  const c = view.counts.get(b.purpose.id);
                  const active = !crossing && b.purpose.id === activeId;
                  const dim = crossing && (c?.hit ?? 0) === 0;
                  return (
                    <button
                      key={b.purpose.id}
                      type="button"
                      onClick={() => selectPurpose(b.purpose.id)}
                      title={b.purpose.note ?? undefined}
                      data-purpose={b.purpose.id}
                      className={cn(
                        "flex items-center gap-1 rounded px-1.5 py-1 text-left text-sm",
                        active ? "bg-primary text-primary-foreground" : "hover:bg-muted",
                        dim && "opacity-40",
                        b.purpose.id === UNCLASSIFIED && !active && "text-amber-300",
                      )}
                    >
                      <span className="min-w-0 flex-1 truncate">{b.purpose.label}</span>
                      {(c?.dirty ?? 0) > 0 && (
                        <span className={cn("size-1.5 shrink-0 rounded-full", active ? "bg-primary-foreground" : "bg-primary-bright")} title={`未保存 ${c!.dirty}`} />
                      )}
                      {hasOthers && (c?.todo ?? 0) > 0 && (
                        <span className={cn("shrink-0 text-[0.65rem]", active ? "text-primary-foreground/80" : "text-amber-300")} title={`ほかの機体と違う未整理のキー ${c!.todo}`}>
                          ≠{c!.todo}
                        </span>
                      )}
                      <span className={cn("shrink-0 text-[0.65rem] tabular-nums", active ? "text-primary-foreground/80" : "text-muted-foreground")}>
                        {crossing ? (c?.hit ?? 0) : (c?.n ?? 0)}
                      </span>
                    </button>
                  );
                })}
                {data.layout.unmatched.length > 0 && (
                  <span
                    className="mt-1 px-1.5 text-[0.65rem] text-amber-300"
                    title={`分類(${data.catalogFile})に書いてあるのに、この機体の yaml に無いキー:\n${data.layout.unmatched.join("\n")}`}
                    data-unmatched
                  >
                    yaml に無いキー {data.layout.unmatched.length}
                  </span>
                )}
              </nav>
              {/* 中: 値の一覧 / 右: 選んだ行の説明(境はドラッグで動かせる) */}
              <ResizablePanelGroup direction="horizontal" autoSaveId="param-purpose-detail" className="min-h-0 min-w-0 flex-1">
              <ResizablePanel defaultSize={64} minSize={35} className="min-w-0">
              <div ref={listRef} className="h-full overflow-auto rounded-md border p-1.5" data-param-list>
                {shown.length === 0 ? (
                  <span className="text-sm text-muted-foreground">
                    {filter === "dirty" ? "未保存の値はありません。" : filter === "diff" ? "ほかの機体と違うキーはありません。" : needle ? "該当するキーがありません。" : "この用途にはキーがありません。"}
                  </span>
                ) : (
                  shown.map((b) => (
                    <div key={b.purpose.id} className="mb-3 flex flex-col gap-2" data-purpose-block={b.purpose.id}>
                      <div className="flex items-baseline gap-2 border-b border-primary-bright/40 pb-0.5">
                        <span className="text-sm font-semibold text-primary-bright">{b.purpose.label}</span>
                        {b.purpose.note && <span className="min-w-0 truncate text-xs text-muted-foreground">{b.purpose.note}</span>}
                        {crossing && (
                          <button type="button" className="ml-auto shrink-0 text-xs text-muted-foreground underline-offset-2 hover:underline" onClick={() => selectPurpose(b.purpose.id)}>
                            この用途を全部見る
                          </button>
                        )}
                      </div>
                      {b.sections.map(({ section, rows }, si) => {
                        // 1 つのファイルのキーだけの見出しは、ファイルの札を見出しに 1 つだけ出す
                        const files = new Map<string, string>();
                        for (const row of rows) {
                          if (row.kind === "head") files.set(row.file, row.alias);
                          else for (const it of rowItems(row)) files.set(it.file, it.alias);
                        }
                        // 値の欄の幅は、その見出しの中でいちばん長い値に合わせる(長すぎる値は別扱い)
                        let longest = 5;
                        for (const row of rows) {
                          if (row.kind !== "single" && row.kind !== "pair") continue;
                          for (const it of rowItems(row)) {
                            const len = textOf(it).length;
                            if (it.leaf.kind === "scalar" && len <= WIDE_LEN) longest = Math.max(longest, len);
                          }
                        }
                        const ctx = {
                          tag: files.size > 1,
                          pair: rows.some((r) => r.kind === "pair"),
                          width: `calc(${longest + 1}ch + 0.8rem)`,
                        };
                        return (
                          <section key={si} className="flex flex-col" data-section={section.label}>
                            <div className="flex items-center gap-2 px-1.5">
                              <span className="text-xs font-semibold">{section.label}</span>
                              {!ctx.tag && [...files].map(([file, alias]) => <span key={file}>{fileTag(file, alias)}</span>)}
                              {section.note && <span className="min-w-0 truncate text-[0.7rem] text-muted-foreground">{section.note}</span>}
                            </div>
                            <div className="grid grid-cols-[fit-content(21rem)_max-content_minmax(0,1fr)_max-content] gap-x-2">
                              {rows.map((row) => renderRow(row, ctx))}
                            </div>
                          </section>
                        );
                      })}
                    </div>
                  ))
                )}
              </div>
              </ResizablePanel>
              <ResizableHandle withHandle />
              <ResizablePanel defaultSize={36} minSize={15} className="min-w-0">
              <aside className="flex h-full flex-col gap-3 overflow-auto rounded-md border p-2" data-param-aside>
                {!focusInfo ? (
                  <span className="text-xs leading-relaxed text-muted-foreground">
                    行を選ぶと、yaml のコメント(調整の経緯)とほかの機体の値がここに出ます。
                    <br />
                    値は入力欄で直して「保存」(Ctrl+S)。↑↓ で最後の桁を 1 つ動かせます。コメントを書く・キーを足すのは「yaml で開く」から。
                  </span>
                ) : (
                  <>
                    {focusInfo.map && (
                      <div className="flex flex-col gap-1.5" data-param-detail-map={focusInfo.map.path}>
                        <div className="flex items-center gap-1.5">
                          {focusInfo.file && fileTag(focusInfo.file, model.aliasOf.get(focusInfo.file)?.alias ?? "")}
                          <span className="min-w-0 flex-1 font-mono text-xs font-semibold break-all">{focusInfo.map.path}</span>
                          <Button size="xs" variant="outline" onClick={() => focusInfo.file && onOpenYaml(focusInfo.file, focusInfo.map!.line)}>
                            yaml で開く
                          </Button>
                        </div>
                        {commentBlock(focusInfo.map.above, focusInfo.map.inline) ?? <span className="text-xs text-muted-foreground">yaml にコメントはありません</span>}
                      </div>
                    )}
                    {!focusInfo.map && focusInfo.items.map(detailOf)}
                  </>
                )}
              </aside>
              </ResizablePanel>
              </ResizablePanelGroup>
            </div>
          </>
        )}
      </CardContent>
    </Card>
  );
});
