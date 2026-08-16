"use client";

import { Fragment, useEffect, useState } from "react";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import {
  emptyStatusRow,
  emptyVelBlock,
  REF_FIELD_KEYS,
  STATUS_LABELS,
  STATUS_ROW_KEYS,
  type ParamMatrixRow,
  type StatusCell,
  type StatusRow,
  type VelBlock,
} from "@/lib/param-matrix-shared";

interface Props {
  onClose: () => void;
}

type RefKey = (typeof REF_FIELD_KEYS)[number];
type ExecKey = "execFast" | "execNormal" | "execSlow";
type VelKey = "search" | "fast" | "dia";

const REF_ROWS: { key: RefKey; label: string }[] = [
  { key: "suction", label: "suction" },
  { key: "large", label: "Large90" },
  { key: "orval", label: "Orval180" },
  { key: "dia45", label: "Dia45" },
  { key: "dia45_2", label: "Dia45_2" },
  { key: "dia135", label: "Dia135" },
  { key: "dia135_2", label: "Dia135_2" },
  { key: "dia90", label: "Dia90" },
];

const VEL_GROUPS: { block: VelKey; label: string }[] = [
  { block: "search", label: "search" },
  { block: "fast", label: "fast_run" },
  { block: "dia", label: "fast_run_dia" },
];

const VEL_FIELDS: { field: keyof VelBlock; label: string }[] = [
  { field: "v", label: "v_max" },
  { field: "a", label: "accl" },
  { field: "d", label: "decel" },
  { field: "w0", label: "w_max" },
  { field: "w1", label: "w_end" },
  { field: "a2", label: "alpha" },
];

const EXEC_ROWS: { key: ExecKey; label: string }[] = [
  { key: "execFast", label: "fast_idx" },
  { key: "execNormal", label: "normal_idx" },
  { key: "execSlow", label: "slow_idx" },
];

function remapIndex(v: number, deletedIdx: number, newLength: number): { value: number; clamped: boolean } {
  if (v === deletedIdx) return { value: Math.max(0, newLength - 1), clamped: true };
  if (v > deletedIdx) return { value: v - 1, clamped: false };
  return { value: v, clamped: false };
}

function cloneRow(r: ParamMatrixRow): ParamMatrixRow {
  return {
    ...r,
    search: { ...r.search },
    fast: { ...r.fast },
    dia: { ...r.dia },
    status: { ...r.status },
  };
}

export function ParamMatrixPanel({ onClose }: Props) {
  const [rows, setRows] = useState<ParamMatrixRow[] | null>(null);
  const [addedVMax, setAddedVMax] = useState<number[]>([]);
  const [dirty, setDirty] = useState(false);
  const [saving, setSaving] = useState(false);
  const [newVMaxInput, setNewVMaxInput] = useState("");

  useEffect(() => {
    void fetch("/api/param-matrix")
      .then((res) => res.json())
      .then((data) => {
        if (data.error) throw new Error(data.error);
        setRows(data.rows as ParamMatrixRow[]);
      })
      .catch((err) => toast.error(`読み込みに失敗しました: ${(err as Error).message}`));
  }, []);

  const updateRow = (idx: number, updater: (row: ParamMatrixRow) => ParamMatrixRow) => {
    setRows((prev) => (prev ? prev.map((r, i) => (i === idx ? updater(cloneRow(r)) : r)) : prev));
    setDirty(true);
  };

  const cycleStatus = (idx: number, key: keyof StatusRow) => {
    updateRow(idx, (r) => {
      const cur = r.status[key];
      const next: StatusCell = cur === null ? "o" : cur === "o" ? "x" : null;
      r.status[key] = next;
      return r;
    });
  };

  const addColumn = () => {
    if (!rows) return;
    const v = parseInt(newVMaxInput, 10);
    if (!Number.isInteger(v) || v <= 0) {
      toast.error("正しい速度(mm/sec)を入力してください");
      return;
    }
    if (rows.some((r) => r.vMax === v)) {
      toast.error(`速度 ${v} は既に存在します`);
      return;
    }
    const last = rows[rows.length - 1];
    const newRow: ParamMatrixRow = last
      ? { ...cloneRow(last), vMax: v, status: emptyStatusRow() }
      : {
          vMax: v,
          suction: 0,
          large: 0,
          orval: 0,
          dia45: 0,
          dia45_2: 0,
          dia135: 0,
          dia135_2: 0,
          dia90: 0,
          search: emptyVelBlock(),
          fast: emptyVelBlock(),
          dia: emptyVelBlock(),
          execFast: 0,
          execNormal: 0,
          execSlow: 0,
          status: emptyStatusRow(),
        };
    setRows([...rows, newRow]);
    setAddedVMax((prev) => [...prev, v]);
    setNewVMaxInput("");
    setDirty(true);
  };

  const deleteColumn = (idx: number) => {
    if (!rows) return;
    const target = rows[idx];
    if (
      !window.confirm(
        `列 t_${target.vMax}.yaml (${target.vMax}mm/sec) をパラメータ表から削除します。\nt_${target.vMax}.yaml 自体のファイルは削除されません。よろしいですか？`
      )
    ) {
      return;
    }
    const newLength = rows.length - 1;
    let anyClamped = false;
    const next = rows
      .filter((_, i) => i !== idx)
      .map((r) => {
        const updated = cloneRow(r);
        for (const key of REF_FIELD_KEYS) {
          const { value, clamped } = remapIndex(r[key], idx, newLength);
          updated[key] = value;
          if (clamped) anyClamped = true;
        }
        return updated;
      });
    setRows(next);
    setAddedVMax((prev) => prev.filter((v) => v !== target.vMax));
    setDirty(true);
    if (anyClamped) {
      toast.warning(
        "削除した列を参照していた項目があったため、自動的に別の列を指すよう調整しました。保存前に内容を確認してください。"
      );
    }
  };

  const save = async () => {
    if (!rows) return;
    setSaving(true);
    try {
      const res = await fetch("/api/param-matrix", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ rows, newVMaxFiles: addedVMax }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "保存に失敗しました");
      toast.success("profiles.yaml / vel_prof.yaml / run_prf.yaml を保存しました");
      setDirty(false);
      setAddedVMax([]);
    } catch (err) {
      toast.error((err as Error).message);
    } finally {
      setSaving(false);
    }
  };

  return (
    <Card className="flex flex-1 flex-col overflow-hidden">
      <CardHeader className="flex-row items-center justify-between space-y-0">
        <CardTitle>パラメータ表 (hf)</CardTitle>
        <div className="flex items-center gap-2">
          {dirty && <span className="text-xs text-muted-foreground">未保存の変更があります</span>}
          <Button size="sm" onClick={save} disabled={!rows || saving}>
            {saving ? "保存中..." : "保存"}
          </Button>
          <Button size="sm" variant="outline" onClick={onClose}>
            閉じる
          </Button>
        </div>
      </CardHeader>
      <CardContent className="flex flex-1 flex-col gap-2 overflow-hidden p-0 px-4 pb-4">
        {!rows ? (
          <div className="flex flex-1 items-center justify-center text-sm text-muted-foreground">読み込み中...</div>
        ) : (
          <>
            <div className="flex shrink-0 items-center gap-2">
              <Input
                className="w-32"
                placeholder="新しい速度(mm/sec)"
                inputMode="numeric"
                value={newVMaxInput}
                onChange={(e) => setNewVMaxInput(e.target.value)}
              />
              <Button size="sm" variant="outline" onClick={addColumn}>
                ＋ 列を追加
              </Button>
            </div>
            <div className="min-h-0 flex-1 overflow-auto rounded-md border border-border">
              <table className="border-collapse text-[11px]">
                <thead>
                  <tr>
                    <th className="sticky left-0 top-0 z-20 min-w-[110px] border-b border-r border-border bg-card px-2 py-1 text-left">
                      mm/sec
                    </th>
                    {rows.map((r, idx) => (
                      <th
                        key={`${idx}-${r.vMax}`}
                        className="sticky top-0 z-10 min-w-[70px] border-b border-l border-border bg-card px-1 py-1 text-center font-semibold"
                      >
                        <VMaxCell
                          value={r.vMax}
                          onCommit={(v) => updateRow(idx, (row) => ({ ...row, vMax: v }))}
                        />
                        <button
                          type="button"
                          onClick={() => deleteColumn(idx)}
                          className="mt-0.5 block w-full text-[10px] text-destructive hover:underline"
                        >
                          削除
                        </button>
                      </th>
                    ))}
                  </tr>
                </thead>
                <tbody>
                  <SectionLabelRow label="調整状況" colSpan={rows.length + 1} />
                  {STATUS_ROW_KEYS.map((key) => (
                    <tr key={key}>
                      <RowLabel label={STATUS_LABELS[key]} />
                      {rows.map((r, idx) => (
                        <td key={idx} className="border-l border-b border-border p-0 text-center">
                          <StatusButton value={r.status[key]} onClick={() => cycleStatus(idx, key)} />
                        </td>
                      ))}
                    </tr>
                  ))}

                  <SectionLabelRow label="ターンプロファイル参照 (行番号)" colSpan={rows.length + 1} />
                  {REF_ROWS.map(({ key, label }) => (
                    <tr key={key}>
                      <RowLabel label={label} />
                      {rows.map((r, idx) => (
                        <td key={idx} className="border-l border-b border-border p-0">
                          <NumberCell
                            value={r[key]}
                            onCommit={(v) => updateRow(idx, (row) => ({ ...row, [key]: Math.round(v) }))}
                          />
                        </td>
                      ))}
                    </tr>
                  ))}

                  {VEL_GROUPS.map(({ block, label }) => (
                    <Fragment key={block}>
                      <SectionLabelRow label={label} colSpan={rows.length + 1} />
                      {VEL_FIELDS.map(({ field, label: fieldLabel }) => (
                        <tr key={`${block}-${field}`}>
                          <RowLabel label={fieldLabel} />
                          {rows.map((r, idx) => (
                            <td key={idx} className="border-l border-b border-border p-0">
                              <NumberCell
                                value={r[block][field]}
                                onCommit={(v) =>
                                  updateRow(idx, (row) => ({ ...row, [block]: { ...row[block], [field]: v } }))
                                }
                              />
                            </td>
                          ))}
                        </tr>
                      ))}
                    </Fragment>
                  ))}

                  <SectionLabelRow label="実行時パラメータ (行番号)" colSpan={rows.length + 1} />
                  {EXEC_ROWS.map(({ key, label }) => (
                    <tr key={key}>
                      <RowLabel label={label} />
                      {rows.map((r, idx) => (
                        <td key={idx} className="border-l border-b border-border p-0">
                          <NumberCell
                            value={r[key]}
                            onCommit={(v) => updateRow(idx, (row) => ({ ...row, [key]: Math.round(v) }))}
                          />
                        </td>
                      ))}
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </>
        )}
      </CardContent>
    </Card>
  );
}

function SectionLabelRow({ label, colSpan }: { label: string; colSpan: number }) {
  return (
    <tr>
      <td
        colSpan={colSpan}
        className="sticky left-0 border-b border-border bg-muted px-2 py-1 font-semibold text-muted-foreground"
      >
        {label}
      </td>
    </tr>
  );
}

function RowLabel({ label }: { label: string }) {
  return (
    <td className="sticky left-0 z-10 min-w-[110px] border-b border-r border-border bg-card px-2 py-1 text-muted-foreground">
      {label}
    </td>
  );
}

function StatusButton({ value, onClick }: { value: StatusCell; onClick: () => void }) {
  const label = value === "o" ? "○" : value === "x" ? "×" : "";
  const color =
    value === "o"
      ? "text-primary"
      : value === "x"
        ? "text-destructive"
        : "text-muted-foreground/40";
  return (
    <button type="button" onClick={onClick} className={`h-6 w-full text-sm font-bold hover:bg-muted ${color}`}>
      {label || "・"}
    </button>
  );
}

// Uncontrolled on purpose: with dozens of columns x rows this avoids a React
// re-render per keystroke. `key={value}` remounts (re-seeding defaultValue)
// only when the committed value actually changes externally (load, add/
// delete column, another cell's edit) - not while the user is still typing.
function NumberCell({ value, onCommit }: { value: number; onCommit: (v: number) => void }) {
  return (
    <input
      key={value}
      type="text"
      inputMode="decimal"
      defaultValue={value}
      onBlur={(e) => {
        const n = Number(e.target.value);
        if (e.target.value.trim() !== "" && !Number.isNaN(n)) onCommit(n);
        else e.target.value = String(value);
      }}
      className="h-6 w-16 bg-transparent px-1 text-center outline-none focus:bg-muted"
    />
  );
}

function VMaxCell({ value, onCommit }: { value: number; onCommit: (v: number) => void }) {
  return (
    <input
      key={value}
      type="text"
      inputMode="numeric"
      defaultValue={value}
      onBlur={(e) => {
        const n = parseInt(e.target.value, 10);
        if (Number.isInteger(n) && n > 0) onCommit(n);
        else e.target.value = String(value);
      }}
      className="h-6 w-16 bg-transparent px-1 text-center font-semibold outline-none focus:bg-muted"
    />
  );
}
