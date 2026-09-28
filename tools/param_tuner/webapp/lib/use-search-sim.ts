"use client";

import { useEffect, useMemo, useRef, useState } from "react";
import type { Cell } from "./maze-shared";
import type { SearchSimResult } from "./search-sim";

// 迷路タブの「探索」: 迷路・ゴールが変わるたびに(少し待って)/api/maze/search で
// SearchController::exec() の探索を回し直す。
const DEBOUNCE_MS = 400;

export function useSearchSim(enabled: boolean, walls: number[], goals: Cell[]) {
  const [result, setResult] = useState<SearchSimResult | null>(null);
  const [busy, setBusy] = useState(false);
  const reqId = useRef(0);

  const goalsKey = JSON.stringify(goals);
  useEffect(() => {
    if (!enabled || walls.length === 0) return;
    const id = ++reqId.current;
    const timer = setTimeout(async () => {
      setBusy(true);
      try {
        const res = await fetch("/api/maze/search", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ walls, goals: JSON.parse(goalsKey) }),
        });
        const data = (await res.json()) as SearchSimResult;
        if (id === reqId.current) setResult(data);
      } catch (err) {
        if (id === reqId.current) setResult({ ok: false, error: (err as Error).message, log: "" });
      } finally {
        if (id === reqId.current) setBusy(false);
      }
    }, DEBOUNCE_MS);
    return () => clearTimeout(timer);
  }, [enabled, walls, goalsKey]);

  // 各ステップの判断時点でロボットが持っている地図(ファームの並び)。変化分を順に当てる。
  const maps = useMemo(() => {
    const steps = result?.ok ? (result.steps ?? []) : [];
    const n = result?.maze_size ?? 0;
    const out: Uint8Array[] = [];
    let cur = new Uint8Array(n * n);
    for (const st of steps) {
      cur = cur.slice();
      for (const [i, v] of st.c) cur[i] = v;
      out.push(cur);
    }
    return out;
  }, [result]);

  return { result, busy, maps };
}
