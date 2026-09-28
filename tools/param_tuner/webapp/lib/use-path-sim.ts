"use client";

import { useEffect, useRef, useState } from "react";
import type { Cell } from "./maze-shared";
import type { ExecOption, PathDirection, PathSimResult } from "./path-sim";

// 迷路タブの「経路」: 迷路・ゴール・走行パラメータが変わるたびに(少し待って)
// /api/maze/path で MainTask::path_run() の経路生成を回し直す。
const PREFS_KEY = "exia-maze-path-prefs-v1";
const DEBOUNCE_MS = 250;

function readPrefs(): { exec: number; direction: PathDirection } {
  try {
    const v = JSON.parse(localStorage.getItem(PREFS_KEY) ?? "{}");
    return { exec: Number.isInteger(v.exec) ? v.exec : 0, direction: v.direction === "left" ? "left" : "right" };
  } catch {
    return { exec: 0, direction: "right" };
  }
}

export function usePathSim(enabled: boolean, walls: number[], goals: Cell[]) {
  const [options, setOptions] = useState<ExecOption[]>([]);
  const [exec, setExec] = useState(0);
  const [direction, setDirection] = useState<PathDirection>("right");
  const [prefsLoaded, setPrefsLoaded] = useState(false);
  const [result, setResult] = useState<PathSimResult | null>(null);
  const [busy, setBusy] = useState(false);
  const reqId = useRef(0);

  useEffect(() => {
    if (!enabled) return;
    if (!prefsLoaded) {
      const p = readPrefs();
      // localStorage は描画後にしか読めない(SSR)。
      // eslint-disable-next-line react-hooks/set-state-in-effect
      setExec(p.exec);
      setDirection(p.direction);
      setPrefsLoaded(true);
    }
    void (async () => {
      try {
        const res = await fetch("/api/maze/path");
        const data = await res.json();
        if (res.ok) setOptions(data.options as ExecOption[]);
      } catch {
        // 選択肢が無くても番号で回せる
      }
    })();
  }, [enabled, prefsLoaded]);

  useEffect(() => {
    if (!prefsLoaded) return;
    try {
      localStorage.setItem(PREFS_KEY, JSON.stringify({ exec, direction }));
    } catch {
      // 保存できなくても動く
    }
  }, [exec, direction, prefsLoaded]);

  const goalsKey = JSON.stringify(goals);
  useEffect(() => {
    if (!enabled || !prefsLoaded || walls.length === 0) return;
    const id = ++reqId.current;
    const timer = setTimeout(async () => {
      setBusy(true);
      try {
        const res = await fetch("/api/maze/path", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ walls, goals: JSON.parse(goalsKey), exec, direction }),
        });
        const data = (await res.json()) as PathSimResult;
        if (id === reqId.current) setResult(data);
      } catch (err) {
        if (id === reqId.current) setResult({ ok: false, error: (err as Error).message, log: "" });
      } finally {
        if (id === reqId.current) setBusy(false);
      }
    }, DEBOUNCE_MS);
    return () => clearTimeout(timer);
  }, [enabled, prefsLoaded, walls, goalsKey, exec, direction]);

  return { options, exec, setExec, direction, setDirection, result, busy };
}
