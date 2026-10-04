"use client";

import { useEffect, useMemo, useState } from "react";
import { load as loadYaml } from "js-yaml";
import type { LineMark } from "./cm-machine-diff";
import { findMachine } from "./machine-client";
import type { FileValues } from "./machine-compare-shared";
import { isSpecific, type MachineRegistry } from "./machine-shared";
import { compareTrees, displayValue } from "./yaml-compare";
import { pathLines } from "./yaml-patch";

export interface MachineDiff {
  marks: LineMark[];
  todo: number; // 未整理の差の数
  specific: number; // 固有の差の数
  // ほかの機体にあって、この下書きに無いキー(印を付ける行が無いので一覧で出す)
  missingHere: string[];
}

const EMPTY: MachineDiff = { marks: [], todo: 0, specific: 0, missingHere: [] };

// 編集中の下書きを、ほかの機体の同じファイルと比べる。
//   registry = 登録簿(page.tsx は MachineContext の外側なので、コンテキストではなく引数で受ける)
//   machine  = 編集しているファイルの機体、file = profile からの相対パス
//   nonce    = 変えると、ほかの機体の値を取り直す(保存・同期のあと)
export function useMachineDiff(
  registry: MachineRegistry | null,
  machine: string | null,
  file: string | null,
  draft: string | null,
  nonce: number,
): MachineDiff {
  const [values, setValues] = useState<FileValues | null>(null);
  const multi = (registry?.machines.length ?? 0) >= 2;

  useEffect(() => {
    if (!multi || !file) return;
    let alive = true;
    void fetch(`/api/machines/compare?values=1&file=${encodeURIComponent(file)}`)
      .then((r) => r.json())
      .then((d) => {
        if (alive && !d.error) setValues(d as FileValues);
      })
      .catch(() => undefined);
    return () => {
      alive = false;
    };
  }, [multi, file, nonce]);

  // 入力のたびに全部を比べ直さない(hardware.yaml は 1200 行ある)
  const [settled, setSettled] = useState(draft);
  useEffect(() => {
    const t = setTimeout(() => setSettled(draft), 250);
    return () => clearTimeout(t);
  }, [draft]);

  return useMemo(() => {
    if (!multi || !registry || !machine || !file || !values || settled === null) return EMPTY;
    let doc: unknown;
    try {
      doc = loadYaml(settled) ?? null;
    } catch {
      return EMPTY; // 入力途中の構文エラー。直るまで印は出さない
    }
    const others = values.machines.filter((m) => m !== machine && m in values.values);
    if (others.length === 0) return EMPTY;
    const { entries } = compareTrees([doc, ...others.map((m) => values.values[m])]);
    const lines = pathLines(settled);
    const byLine = new Map<number, LineMark>();
    const missingHere: string[] = [];
    let todo = 0;
    let specific = 0;
    for (const e of entries) {
      // 下書きと違う機体がいるキーだけ(ほかの機体どうしの違いは、この画面では出さない)
      const differing = others.filter((_, i) => e.values[i + 1] !== e.values[0]);
      if (differing.length === 0) continue;
      const spec = isSpecific(registry.specific, file, e.path);
      if (spec) specific++;
      else todo++;
      // キーがこの下書きに無いときは、いちばん近い親の行に付ける
      let line = lines.get(e.path);
      if (line === undefined) {
        let p = e.path;
        while (line === undefined && p.length > 0) {
          const cut = Math.max(p.lastIndexOf("."), p.lastIndexOf("["));
          if (cut <= 0) break;
          p = p.slice(0, cut);
          line = lines.get(p);
        }
      }
      const key = e.path.slice(Math.max(e.path.lastIndexOf("."), 0)).replace(/^\./, "");
      const tips = differing.map((m) => {
        const v = e.values[others.indexOf(m) + 1];
        return `${findMachine(registry, m)?.label ?? m}  ${key}: ${displayValue(v, 80)}`;
      });
      if (e.values[0] === null) tips.unshift(`この機体には ${key} がありません`);
      if (e.values[0] === null) missingHere.push(e.path);
      if (line === undefined) continue;
      const cur = byLine.get(line);
      if (cur) {
        cur.tips.push(...tips);
        if (!spec) cur.kind = "todo";
      } else {
        byLine.set(line, { line, kind: spec ? "specific" : "todo", tips });
      }
    }
    return { marks: [...byLine.values()].sort((a, b) => a.line - b.line), todo, specific, missingHere };
  }, [multi, machine, file, values, settled, registry]);
}
