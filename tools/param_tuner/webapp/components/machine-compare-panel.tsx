"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { toast } from "sonner";
import { MachineChip } from "@/components/machine-chip";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { ScrollArea } from "@/components/ui/scroll-area";
import { findMachine, useMachines } from "@/lib/machine-client";
import type { CompareEntry, CompareFile, CompareResult, SyncResult } from "@/lib/machine-compare-shared";
import type { BoardInfo, MachineRegistry, PathSeg } from "@/lib/machine-shared";
import { displayValue } from "@/lib/yaml-compare";

interface Props {
  mode: string;
  onClose: () => void;
  // どこかの機体のファイルを書き換えた(未送信の表示などを取り直す)
  onFilesChanged: () => void;
  // 「固有」の登録を変えた
  onRegistry: (registry: MachineRegistry, board: BoardInfo) => void;
  // その機体のファイルを編集画面で開く
  onEdit: (machine: string, file: string) => void;
}

type Filter = "todo" | "specific" | "all";

// 1 回目のクリックで確認の表示に変わり、2 回目で実行する(まとめて書き換える操作用)
function ConfirmButton({
  label,
  confirmLabel,
  title,
  disabled,
  onConfirm,
}: {
  label: string;
  confirmLabel: string;
  title?: string;
  disabled?: boolean;
  onConfirm: () => void;
}) {
  const [armed, setArmed] = useState(false);
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);
  useEffect(() => () => void (timer.current && clearTimeout(timer.current)), []);
  return (
    <Button
      size="xs"
      variant={armed ? "destructive" : "outline"}
      disabled={disabled}
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

function fileCounts(f: CompareFile) {
  let todo = 0;
  let specific = 0;
  for (const e of f.entries) {
    if (e.specific) specific++;
    else todo++;
  }
  return { todo, specific, missingFile: f.present.some((p) => !p) };
}

// 機体どうしのパラメータの違いの一覧と同期。
//   未整理 = 機体で値が違う(または片方に無い)のに「固有」に登録していないキー。
//            そろえる(→ / ←)か、機体ごとに違ってよい値なら「固有」にする。
//   固有   = 機体ごとに違ってよいと登録したキー(タイヤ径・センサーのゲインなど)。
export function MachineComparePanel({ mode, onClose, onFilesChanged, onRegistry, onEdit }: Props) {
  const { registry, current } = useMachines();
  const [result, setResult] = useState<CompareResult | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [selected, setSelected] = useState<string | null>(null);
  const [base, setBase] = useState<string | null>(current);
  const [filter, setFilter] = useState<Filter>("todo");
  const [query, setQuery] = useState("");
  const [busy, setBusy] = useState(false);

  const load = useCallback(async () => {
    try {
      const res = await fetch(`/api/machines/compare?mode=${encodeURIComponent(mode)}`);
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "読み込みに失敗しました");
      setResult(data as CompareResult);
      setError(null);
    } catch (err) {
      setError((err as Error).message);
    }
  }, [mode]);

  useEffect(() => {
    // eslint-disable-next-line react-hooks/set-state-in-effect
    void load();
    // 別のエディタ(VSCode など)で yaml を直して戻ってきたときに読み直す
    const onFocus = () => void load();
    window.addEventListener("focus", onFocus);
    return () => window.removeEventListener("focus", onFocus);
  }, [load]);

  const machines = useMemo(() => result?.machines ?? [], [result]);
  const baseId = base && machines.includes(base) ? base : (machines[0] ?? null);
  const baseIdx = baseId ? machines.indexOf(baseId) : -1;
  const others = machines.filter((m) => m !== baseId);

  const files = useMemo(() => result?.files ?? [], [result]);
  // 最初は、未整理の差がある最初のファイルを開く
  const selectedFile =
    files.find((f) => f.file === selected) ??
    files.find((f) => fileCounts(f).todo > 0 || fileCounts(f).missingFile) ??
    files[0] ??
    null;

  const totals = useMemo(() => {
    let todo = 0;
    let specific = 0;
    for (const f of files) {
      const c = fileCounts(f);
      todo += c.todo;
      specific += c.specific;
    }
    return { todo, specific };
  }, [files]);

  const needle = query.trim().toLowerCase();
  const rows = useMemo(() => {
    if (!selectedFile) return [];
    return selectedFile.entries.filter((e) => {
      if (filter === "todo" && e.specific) return false;
      if (filter === "specific" && !e.specific) return false;
      if (needle && !e.path.toLowerCase().includes(needle)) return false;
      return true;
    });
  }, [selectedFile, filter, needle]);

  const undo = async (token: string) => {
    try {
      const res = await fetch("/api/machines/sync", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ undo: token }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "元に戻せませんでした");
      toast.success(`${data.label}: 元に戻しました`);
      await load();
      onFilesChanged();
    } catch (err) {
      toast.error((err as Error).message);
    }
  };

  const sync = async (from: string, to: string, file: string, body: { paths: PathSeg[][] } | { whole: true }) => {
    setBusy(true);
    try {
      const res = await fetch("/api/machines/sync", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ from, to, file, ...body }),
      });
      const data = (await res.json()) as SyncResult & { error?: string };
      if (!res.ok) throw new Error(data.error ?? "書き換えに失敗しました");
      const toLabel = findMachine(registry, to)?.label ?? to;
      if (data.applied.length > 0) {
        const what =
          "whole" in body
            ? "ファイルごとコピーしました"
            : data.applied.length === 1
              ? `${data.applied[0]} を書き換えました`
              : `${data.applied.length} 個のキーを書き換えました`;
        toast.success(`${toLabel} の ${file}: ${what}`, {
          description: "機体へはまだ送っていません(その機体をつないで送信)",
          action: data.undo ? { label: "元に戻す", onClick: () => void undo(data.undo!) } : undefined,
        });
      }
      if (data.errors.length > 0) {
        toast.error(
          `${toLabel} の ${file}: ${data.errors.length} 個は書き換えられませんでした`,
          { description: data.errors.slice(0, 3).map((e) => `${e.path}: ${e.error}`).join("\n") },
        );
      }
      await load();
      onFilesChanged();
    } catch (err) {
      toast.error((err as Error).message);
    } finally {
      setBusy(false);
    }
  };

  const setSpecific = async (file: string, paths: true | string[], on: boolean) => {
    setBusy(true);
    try {
      const res = await fetch("/api/machines", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: "specific", file, paths, on }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "登録に失敗しました");
      onRegistry(data.registry, data.board);
      await load();
    } catch (err) {
      toast.error((err as Error).message);
    } finally {
      setBusy(false);
    }
  };

  const label = (id: string) => findMachine(registry, id)?.label ?? id;

  return (
    <Card className="flex h-full min-w-0 flex-1 flex-col overflow-hidden" data-compare-panel>
      <CardHeader className="flex flex-row flex-wrap items-center gap-2 space-y-0">
        <CardTitle>機体比較</CardTitle>
        <span className="text-xs text-muted-foreground">基準</span>
        <div className="flex gap-1">
          {machines.map((m) => (
            <button key={m} type="button" onClick={() => setBase(m)} title={`${label(m)} を基準(左の列)にする`}>
              <MachineChip machine={findMachine(registry, m)} fallback={m} solid={m === baseId} />
            </button>
          ))}
        </div>
        <div className="flex items-center gap-1">
          {(
            [
              ["todo", `未整理 ${totals.todo}`, "値が違う・片方に無いのに「固有」に登録していないキー。そろえるか、固有にする"],
              ["specific", `固有 ${totals.specific}`, "機体ごとに違ってよいと登録したキー"],
              ["all", "両方", "未整理と固有の両方"],
            ] as const
          ).map(([key, text, tip]) => (
            <Button
              key={key}
              size="xs"
              variant={filter === key ? "default" : "outline"}
              title={tip}
              onClick={() => setFilter(key)}
              data-filter={key}
            >
              {text}
            </Button>
          ))}
        </div>
        <Input
          placeholder="キーを絞り込み..."
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          className="h-7 w-44"
        />
        <div className="flex-1" />
        <Button size="sm" variant="ghost" onClick={() => void load()} title="ファイルを読み直す(エディタで直接書き換えたとき)">
          再読込
        </Button>
        <Button size="sm" variant="outline" onClick={onClose}>
          閉じる
        </Button>
      </CardHeader>
      <CardContent className="flex min-h-0 flex-1 gap-2 overflow-hidden">
        {error ? (
          <span className="text-sm text-destructive">{error}</span>
        ) : !result ? (
          <span className="text-sm text-muted-foreground">読み込み中...</span>
        ) : machines.length < 2 ? (
          <span className="text-sm text-muted-foreground">
            比べる機体がありません。ヘッダーの「機体設定」から機体を追加してください。
          </span>
        ) : (
          <>
            {/* 左: ファイルごとの差の数 */}
            <ScrollArea className="min-h-0 w-60 shrink-0 rounded-md border">
              <div className="flex flex-col p-1" data-compare-files>
                {files.map((f) => {
                  const c = fileCounts(f);
                  const active = selectedFile?.file === f.file;
                  const clean = c.todo === 0 && !c.missingFile;
                  return (
                    <button
                      key={f.file}
                      type="button"
                      data-file={f.file}
                      onClick={() => setSelected(f.file)}
                      className={`flex items-center justify-between gap-1 rounded px-1.5 py-0.5 text-left text-sm hover:bg-muted ${active ? "bg-muted" : ""} ${clean && c.specific === 0 ? "text-muted-foreground" : ""}`}
                    >
                      <span className="truncate">{f.file}</span>
                      <span className="flex shrink-0 items-center gap-1 text-xs">
                        {c.missingFile && <span className="rounded bg-destructive/20 px-1 text-destructive">ファイルなし</span>}
                        {f.errors.some(Boolean) && <span className="rounded bg-destructive/20 px-1 text-destructive">構文エラー</span>}
                        {c.todo > 0 && (
                          <span className="rounded bg-amber-500/20 px-1 font-semibold text-amber-300" title="未整理の差">
                            {c.todo}
                          </span>
                        )}
                        {(c.specific > 0 || f.fileSpecific) && (
                          <span className="text-muted-foreground" title={f.fileSpecific ? "ファイル全体が固有" : "固有の差"}>
                            固{c.specific}
                          </span>
                        )}
                        {clean && c.specific === 0 && !f.fileSpecific && <span title="全機体で同じ">＝</span>}
                      </span>
                    </button>
                  );
                })}
              </div>
            </ScrollArea>

            {/* 右: 選んだファイルのキーごとの差 */}
            {selectedFile && baseId && (
              <div className="flex min-w-0 flex-1 flex-col gap-1.5 overflow-hidden">
                <div className="flex flex-wrap items-center gap-1.5 text-xs" data-compare-actions>
                  <span className="text-sm font-medium">{selectedFile.file}</span>
                  <label
                    className="flex cursor-pointer items-center gap-1 text-muted-foreground"
                    title="このファイルは機体ごとに別物(どのキーが違っても未整理に数えない)"
                  >
                    <input
                      type="checkbox"
                      checked={selectedFile.fileSpecific}
                      disabled={busy}
                      onChange={(e) => void setSpecific(selectedFile.file, true, e.target.checked)}
                    />
                    ファイル全体が固有
                  </label>
                  {(() => {
                    const todo = selectedFile.entries.filter((e) => !e.specific);
                    return (
                      todo.length > 0 && (
                        <ConfirmButton
                          label={`未整理 ${todo.length} 個を固有にする`}
                          confirmLabel={`${todo.length} 個を「機体ごとに違ってよい」に登録する`}
                          title="いま未整理のキーをすべて「固有」に登録する(値は変えない)"
                          disabled={busy}
                          onConfirm={() => void setSpecific(selectedFile.file, todo.map((e) => e.path), true)}
                        />
                      )
                    );
                  })()}
                  <div className="flex-1" />
                  {others.map((o) => {
                    const oi = machines.indexOf(o);
                    const filePresent = selectedFile.present[oi] && selectedFile.present[baseIdx];
                    if (!filePresent) return null;
                    const todo = selectedFile.entries.filter((e) => !e.specific);
                    const missing = todo.filter((e) => e.values[oi] === null && e.values[baseIdx] !== null);
                    const differ = todo.filter((e) => e.values[baseIdx] !== null && e.values[oi] !== e.values[baseIdx]);
                    return (
                      <span key={o} className="flex items-center gap-1">
                        {missing.length > 0 && (
                          <Button
                            size="xs"
                            variant="outline"
                            disabled={busy}
                            title={`${label(o)} に無いキーを、${label(baseId)} の値で足す(説明のコメントごと。既にあるキーは変えない)`}
                            onClick={() => void sync(baseId, o, selectedFile.file, { paths: missing.map((e) => e.segs) })}
                            data-add-missing={o}
                          >
                            無いキー {missing.length} 個を {label(o)} へ追加
                          </Button>
                        )}
                        {differ.length > 0 && (
                          <ConfirmButton
                            label={`未整理をすべて ${label(baseId)} → ${label(o)}`}
                            confirmLabel={`${label(o)} の ${differ.length} 個を ${label(baseId)} の値で上書きする`}
                            title={`未整理のキーをすべて ${label(baseId)} の値にそろえる(固有のキーは変えない)`}
                            disabled={busy}
                            onConfirm={() => void sync(baseId, o, selectedFile.file, { paths: differ.map((e) => e.segs) })}
                          />
                        )}
                      </span>
                    );
                  })}
                </div>

                {selectedFile.present.some((p) => !p) ? (
                  <div className="flex flex-col gap-1.5 rounded-md border p-2 text-sm" data-missing-file>
                    {machines.map((m, i) =>
                      selectedFile.present[i] ? null : (
                        <div key={m} className="flex flex-wrap items-center gap-2">
                          <MachineChip machine={findMachine(registry, m)} fallback={m} />
                          <span>にこのファイルがありません。</span>
                          {machines
                            .filter((_, j) => selectedFile.present[j])
                            .map((src) => (
                              <Button
                                key={src}
                                size="xs"
                                variant="outline"
                                disabled={busy}
                                onClick={() => void sync(src, m, selectedFile.file, { whole: true })}
                              >
                                {label(src)} からコピー
                              </Button>
                            ))}
                        </div>
                      ),
                    )}
                  </div>
                ) : (
                  <div className="min-h-0 flex-1 overflow-auto rounded-md border">
                    <table className="border-collapse text-xs" data-compare-table>
                      <thead className="sticky top-0 z-10 bg-card">
                        <tr className="border-b text-left">
                          <th className="px-1.5 py-1 font-medium">キー</th>
                          <th className="px-1.5 py-1 font-medium">
                            <span className="flex items-center gap-1">
                              <MachineChip machine={findMachine(registry, baseId)} fallback={baseId} />
                              <button
                                type="button"
                                className="text-muted-foreground underline-offset-2 hover:underline"
                                onClick={() => onEdit(baseId, selectedFile.file)}
                              >
                                編集
                              </button>
                            </span>
                          </th>
                          {others.map((o) => (
                            <th key={o} className="px-1.5 py-1 font-medium">
                              <span className="flex items-center gap-1">
                                <MachineChip machine={findMachine(registry, o)} fallback={o} />
                                <button
                                  type="button"
                                  className="text-muted-foreground underline-offset-2 hover:underline"
                                  onClick={() => onEdit(o, selectedFile.file)}
                                >
                                  編集
                                </button>
                              </span>
                            </th>
                          ))}
                          <th
                            className="px-1.5 py-1 text-center font-medium"
                            title="機体ごとに違ってよい値として登録する(未整理に数えなくなる)"
                          >
                            固有
                          </th>
                        </tr>
                      </thead>
                      <tbody>
                        {rows.length === 0 && (
                          <tr>
                            <td colSpan={3 + others.length} className="px-2 py-3 text-center text-muted-foreground">
                              {selectedFile.entries.length === 0
                                ? "このファイルは全機体で同じ値です"
                                : filter === "todo"
                                  ? "未整理の差はありません(違いはすべて「固有」に登録済み)"
                                  : "該当するキーがありません"}
                            </td>
                          </tr>
                        )}
                        {rows.map((e) => (
                          <CompareRow
                            key={e.path}
                            entry={e}
                            file={selectedFile}
                            machines={machines}
                            baseId={baseId}
                            others={others}
                            busy={busy}
                            label={label}
                            onSync={(from, to) => void sync(from, to, selectedFile.file, { paths: [e.segs] })}
                            onSpecific={(on) => void setSpecific(selectedFile.file, [e.path], on)}
                          />
                        ))}
                      </tbody>
                    </table>
                  </div>
                )}
              </div>
            )}
          </>
        )}
      </CardContent>
    </Card>
  );
}

function ValueText({ value, differs }: { value: string | null; differs: boolean }) {
  if (value === null) return <span className="text-destructive/80 italic">なし</span>;
  return (
    <span className={`font-mono ${differs ? "text-amber-300" : ""}`} title={value}>
      {displayValue(value)}
    </span>
  );
}

function CompareRow({
  entry,
  file,
  machines,
  baseId,
  others,
  busy,
  label,
  onSync,
  onSpecific,
}: {
  entry: CompareEntry;
  file: CompareFile;
  machines: string[];
  baseId: string;
  others: string[];
  busy: boolean;
  label: (id: string) => string;
  onSync: (from: string, to: string) => void;
  onSpecific: (on: boolean) => void;
}) {
  const bi = machines.indexOf(baseId);
  const bv = entry.values[bi];
  return (
    <tr className={`border-b border-border/50 hover:bg-muted/40 ${entry.specific ? "opacity-60" : ""}`} data-path={entry.path}>
      <td className="max-w-[26rem] py-0.5 pr-4 pl-1.5 font-mono" title={entry.path}>
        <span className="block truncate">
          {entry.path}
          {entry.leafCount > 1 && <span className="text-muted-foreground"> ({entry.leafCount} 個の値)</span>}
        </span>
      </td>
      <td className="min-w-28 py-0.5 pr-4 pl-1.5">
        <ValueText value={bv} differs={false} />
      </td>
      {others.map((o) => {
        const oi = machines.indexOf(o);
        const ov = entry.values[oi];
        const same = ov === bv;
        return (
          <td key={o} className="min-w-36 py-0.5 pr-4 pl-1.5">
            <span className="flex items-center gap-1">
              <button
                type="button"
                disabled={busy || same || bv === null}
                title={bv === null ? `${label(baseId)} に無いので入れられません` : `${label(baseId)} の値を ${label(o)} へ入れる`}
                className="rounded border px-1 leading-4 hover:bg-muted disabled:opacity-30"
                onClick={() => onSync(baseId, o)}
                data-push={o}
              >
                →
              </button>
              <button
                type="button"
                disabled={busy || same || ov === null}
                title={ov === null ? `${label(o)} に無いので入れられません` : `${label(o)} の値を ${label(baseId)} へ入れる`}
                className="rounded border px-1 leading-4 hover:bg-muted disabled:opacity-30"
                onClick={() => onSync(o, baseId)}
                data-pull={o}
              >
                ←
              </button>
              <ValueText value={ov} differs={!same} />
            </span>
          </td>
        );
      })}
      <td className="px-1.5 py-0.5 text-center">
        <input
          type="checkbox"
          checked={entry.specific}
          disabled={busy || file.fileSpecific}
          title={
            file.fileSpecific
              ? "ファイル全体が固有になっています"
              : entry.specific
                ? "固有(機体ごとに違ってよい)。外すと未整理に戻る"
                : "機体ごとに違ってよい値として登録する"
          }
          onChange={(ev) => onSpecific(ev.target.checked)}
        />
      </td>
    </tr>
  );
}
