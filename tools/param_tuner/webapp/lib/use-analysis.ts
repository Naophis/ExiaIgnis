"use client";

// ログの重畳解析(旧 analyze_*.py / wall_off_edge_check.py / turn_exit_check.py)の
// 設定と計算。プロットタブ(log-plot-panel.tsx)と詳細ログ解析ページ
// (log-detail-view.tsx)の両方から使うので、設定の持ち方と計算をここに一本化する。
// UI は components/analysis-toggles.tsx。

import { useMemo, useState } from "react";
import {
  computeHfEdgeEvents,
  computeMotionTransitionEvents,
  computeSensorDropEvents,
  computeSensorTroughEvents,
  computeWallOffEdgeEvents,
  type AnalysisEvent,
  type RowPointMap,
} from "@/lib/log-analysis";
import { analyzeTurnExits, turnExitLabel, type TurnExitRow } from "@/lib/turn-exit";

export const TRANSITION_COLUMNS = [
  "left45_d",
  "left45_2_d",
  "left45_3_d",
  "right45_d",
  "right45_2_d",
  "right45_3_d",
];

export const TROUGH_COLUMNS = ["left45_d", "right45_d", "left90_d", "right90_d"] as const;
export type TroughColumn = (typeof TROUGH_COLUMNS)[number];

export const TROUGH_COLUMN_COLOR: Record<TroughColumn, string> = {
  left45_d: "#e06c75",
  right45_d: "#61afef",
  left90_d: "#98c379",
  right90_d: "#d19a66",
};

// left45_d/right45_d はセンサー生値からの変換後の距離。同時に、変換元の
// 生距離(sen_dist_l45/sen_dist_r45)も破線で重ねて見比べられるようにする。
export const TROUGH_COMPANION_COLUMN: Partial<Record<TroughColumn, string>> = {
  left45_d: "sen_dist_l45",
  right45_d: "sen_dist_r45",
};

/** イベント一覧の文字色。kind はマーカーの形/色(trajectory-plot.tsx)と対応させる。 */
export const EVENT_COLOR: Record<AnalysisEvent["kind"], string> = {
  drop: "text-red-400",
  rise: "text-emerald-400",
  "state-start": "text-amber-400",
  "state-end": "text-sky-400",
  "state-start-sensor": "text-amber-300",
  "state-end-sensor": "text-sky-300",
  trough: "text-purple-400",
  "trough-rise": "text-cyan-400",
  "wall-off-anchor": "text-sky-400",
  "wall-off-anchor-sensor": "text-sky-300",
  "wall-off-actual": "text-orange-400",
  "wall-off-actual-sensor": "text-orange-300",
  "wall-off-arm": "text-yellow-400",
  "wall-off-edge": "text-green-400",
  "wall-off-edge-sensor": "text-green-300",
  "hf-edge": "text-fuchsia-400",
  "hf-edge-sensor": "text-fuchsia-300",
  "turn-exit": "text-lime-400",
};

function parseStates(text: string): number[] {
  return text
    .split(",")
    .map((s) => parseFloat(s.trim()))
    .filter((n) => !Number.isNaN(n));
}

export interface AnalysisSettings {
  drop: {
    enabled: boolean;
    setEnabled: (v: boolean) => void;
    motionState: number;
    setMotionState: (v: number) => void;
    low: number;
    setLow: (v: number) => void;
    high: number;
    setHigh: (v: number) => void;
    left: boolean;
    setLeft: (v: boolean) => void;
    right: boolean;
    setRight: (v: boolean) => void;
  };
  transition: {
    enabled: boolean;
    setEnabled: (v: boolean) => void;
    states: string;
    setStates: (v: string) => void;
  };
  trough: {
    enabled: boolean;
    setEnabled: (v: boolean) => void;
    motionState: number;
    setMotionState: (v: number) => void;
    states: string;
    setStates: (v: string) => void;
    eps: number;
    setEps: (v: number) => void;
    medianWindow: number;
    setMedianWindow: (v: number) => void;
    columns: Record<TroughColumn, boolean>;
    toggleColumn: (c: TroughColumn, v: boolean) => void;
  };
  wallOff: {
    enabled: boolean;
    setEnabled: (v: boolean) => void;
    motionStates: string;
    setMotionStates: (v: string) => void;
    baselineN: number;
    setBaselineN: (v: number) => void;
    armDelta: number;
    setArmDelta: (v: number) => void;
    fitLo: number;
    setFitLo: (v: number) => void;
    fitHi: number;
    setFitHi: (v: number) => void;
  };
  turnExit: {
    enabled: boolean;
    setEnabled: (v: boolean) => void;
    limit: number;
    setLimit: (v: number) => void;
  };
}

/** 解析の設定をまとめて持つ。既定値は両画面で同じ。 */
export function useAnalysisSettings(overrides?: { turnExitEnabled?: boolean }): AnalysisSettings {
  const [dropEnabled, setDropEnabled] = useState(false);
  const [dropMotionState, setDropMotionState] = useState(4);
  const [dropLow, setDropLow] = useState(70);
  const [dropHigh, setDropHigh] = useState(80);
  const [dropColLeft, setDropColLeft] = useState(true);
  const [dropColRight, setDropColRight] = useState(true);

  const [transitionEnabled, setTransitionEnabled] = useState(false);
  const [transitionStates, setTransitionStates] = useState("6,13");

  const [troughEnabled, setTroughEnabled] = useState(false);
  const [troughMotionState, setTroughMotionState] = useState(4);
  const [troughStates, setTroughStates] = useState("14,1");
  const [troughEps, setTroughEps] = useState(3);
  const [troughMedianWindow, setTroughMedianWindow] = useState(3);
  const [troughColumns, setTroughColumns] = useState<Record<TroughColumn, boolean>>({
    left45_d: true,
    right45_d: true,
    left90_d: false,
    right90_d: false,
  });

  const [wallOffEnabled, setWallOffEnabled] = useState(false);
  const [wallOffMotionStates, setWallOffMotionStates] = useState("6,13");
  const [wallOffBaselineN, setWallOffBaselineN] = useState(5);
  const [wallOffArmDelta, setWallOffArmDelta] = useState(1.0);
  const [wallOffFitLo, setWallOffFitLo] = useState(1.0);
  const [wallOffFitHi, setWallOffFitHi] = useState(5.0);

  const [turnExitEnabled, setTurnExitEnabled] = useState(overrides?.turnExitEnabled ?? false);
  const [turnExitLimit, setTurnExitLimit] = useState(6);

  return {
    drop: {
      enabled: dropEnabled,
      setEnabled: setDropEnabled,
      motionState: dropMotionState,
      setMotionState: setDropMotionState,
      low: dropLow,
      setLow: setDropLow,
      high: dropHigh,
      setHigh: setDropHigh,
      left: dropColLeft,
      setLeft: setDropColLeft,
      right: dropColRight,
      setRight: setDropColRight,
    },
    transition: {
      enabled: transitionEnabled,
      setEnabled: setTransitionEnabled,
      states: transitionStates,
      setStates: setTransitionStates,
    },
    trough: {
      enabled: troughEnabled,
      setEnabled: setTroughEnabled,
      motionState: troughMotionState,
      setMotionState: setTroughMotionState,
      states: troughStates,
      setStates: setTroughStates,
      eps: troughEps,
      setEps: setTroughEps,
      medianWindow: troughMedianWindow,
      setMedianWindow: setTroughMedianWindow,
      columns: troughColumns,
      toggleColumn: (c, v) => setTroughColumns((prev) => ({ ...prev, [c]: v })),
    },
    wallOff: {
      enabled: wallOffEnabled,
      setEnabled: setWallOffEnabled,
      motionStates: wallOffMotionStates,
      setMotionStates: setWallOffMotionStates,
      baselineN: wallOffBaselineN,
      setBaselineN: setWallOffBaselineN,
      armDelta: wallOffArmDelta,
      setArmDelta: setWallOffArmDelta,
      fitLo: wallOffFitLo,
      setFitLo: setWallOffFitLo,
      fitHi: wallOffFitHi,
      setFitHi: setWallOffFitHi,
    },
    turnExit: {
      enabled: turnExitEnabled,
      setEnabled: setTurnExitEnabled,
      limit: turnExitLimit,
      setLimit: setTurnExitLimit,
    },
  };
}

export interface AnalysisInput {
  // CSV ファイル出現順の生行(trajectory.ts のソート済み行ではない)
  rawRows: Record<string, number>[];
  pointByRow: RowPointMap;
  xOffset: number;
  // hf(4kHz相当)壁切れ検出のマーカーを出すか。軌跡側の hf 点群表示と連動させる。
  showHf: boolean;
  logName?: string;
}

export interface AnalysisResult {
  events: AnalysisEvent[];
  /** 時系列グラフに重ねる分(seriesIndex を持つものだけ) */
  chartMarkers: AnalysisEvent[];
  turnExitRows: TurnExitRow[];
}

export function useAnalysisEvents(s: AnalysisSettings, input: AnalysisInput): AnalysisResult {
  const { rawRows, pointByRow, xOffset, showHf, logName } = input;

  const dropEvents = useMemo<AnalysisEvent[]>(() => {
    if (!s.drop.enabled || rawRows.length === 0) return [];
    const columns = [s.drop.left && "left45_d", s.drop.right && "right45_d"].filter(Boolean) as string[];
    if (columns.length === 0) return [];
    return computeSensorDropEvents(rawRows, {
      motionState: s.drop.motionState,
      low: s.drop.low,
      high: s.drop.high,
      columns,
      pointByRow,
      xOffset,
    });
  }, [rawRows, s.drop, pointByRow, xOffset]);

  const transitionEvents = useMemo<AnalysisEvent[]>(() => {
    if (!s.transition.enabled || rawRows.length === 0) return [];
    const states = parseStates(s.transition.states);
    if (states.length === 0) return [];
    return computeMotionTransitionEvents(rawRows, { states, columns: TRANSITION_COLUMNS, pointByRow, xOffset });
  }, [rawRows, s.transition, pointByRow, xOffset]);

  const troughEvents = useMemo<AnalysisEvent[]>(() => {
    if (!s.trough.enabled || rawRows.length === 0) return [];
    const columns = TROUGH_COLUMNS.filter((c) => s.trough.columns[c]);
    if (columns.length === 0) return [];
    const states = parseStates(s.trough.states);
    if (states.length === 0) return [];
    return computeSensorTroughEvents(rawRows, {
      motionState: s.trough.motionState,
      states,
      eps: s.trough.eps,
      medianWindow: s.trough.medianWindow,
      columns,
      pointByRow,
      xOffset,
    });
  }, [rawRows, s.trough, pointByRow, xOffset]);

  const wallOffEvents = useMemo<AnalysisEvent[]>(() => {
    if (!s.wallOff.enabled || rawRows.length === 0) return [];
    const motionStates = parseStates(s.wallOff.motionStates);
    if (motionStates.length === 0) return [];
    return computeWallOffEdgeEvents(rawRows, {
      motionStates,
      baselineN: s.wallOff.baselineN,
      armDelta: s.wallOff.armDelta,
      fitLo: s.wallOff.fitLo,
      fitHi: s.wallOff.fitHi,
      pointByRow,
      xOffset,
    });
  }, [rawRows, s.wallOff, pointByRow, xOffset]);

  const hfEvents = useMemo<AnalysisEvent[]>(() => {
    if (!showHf || rawRows.length === 0 || !("hf_edge_rel" in rawRows[0])) return [];
    return computeHfEdgeEvents(rawRows, { pointByRow, xOffset });
  }, [showHf, rawRows, pointByRow, xOffset]);

  const turnExitRows = useMemo<TurnExitRow[]>(() => {
    if (!s.turnExit.enabled || rawRows.length === 0) return [];
    return analyzeTurnExits(rawRows, { log: logName ?? "" });
  }, [s.turnExit.enabled, rawRows, logName]);

  const turnExitEvents = useMemo<AnalysisEvent[]>(() => {
    const out: AnalysisEvent[] = [];
    for (const t of turnExitRows) {
      const r = rawRows[t.exitIdx];
      if (!r || !Number.isFinite(r.x) || !Number.isFinite(r.y)) continue;
      out.push({ x: r.x, y: r.y, anchored: "robot", kind: "turn-exit", label: turnExitLabel(t) });
    }
    return out;
  }, [turnExitRows, rawRows]);

  const events = useMemo(
    () => [...dropEvents, ...transitionEvents, ...troughEvents, ...wallOffEvents, ...hfEvents, ...turnExitEvents],
    [dropEvents, transitionEvents, troughEvents, wallOffEvents, hfEvents, turnExitEvents]
  );

  const chartMarkers = useMemo(
    () => [...troughEvents, ...wallOffEvents, ...hfEvents].filter((e) => e.seriesIndex !== undefined),
    [troughEvents, wallOffEvents, hfEvents]
  );

  return { events, chartMarkers, turnExitRows };
}
