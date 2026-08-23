"use client";

import { useCallback, useEffect, useMemo, useState } from "react";
import { CopyIcon } from "lucide-react";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { TrajectoryPlot } from "@/components/trajectory-plot";
import {
  computeMotionTransitionEvents,
  computeSensorDropEvents,
  computeSensorTroughEvents,
  type AnalysisEvent,
} from "@/lib/log-analysis";
import { buildTrajectoryData, parseCsv, type TrajectoryPoint } from "@/lib/trajectory";

const TRANSITION_COLUMNS = ["left45_d", "left45_2_d", "left45_3_d", "right45_d", "right45_2_d", "right45_3_d"];
const TROUGH_COLUMNS = ["left45_d", "right45_d", "left90_d", "right90_d"] as const;

const EVENT_COLOR: Record<AnalysisEvent["kind"], string> = {
  drop: "text-red-400",
  rise: "text-emerald-400",
  "state-start": "text-amber-400",
  "state-end": "text-sky-400",
  trough: "text-purple-400",
  "trough-rise": "text-cyan-400",
};

interface LogFileInfo {
  name: string;
  mtimeMs: number;
  size: number;
}

function formatDate(mtimeMs: number): string {
  const d = new Date(mtimeMs);
  const p = (n: number) => String(n).padStart(2, "0");
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`;
}

function formatClickInfo(p: TrajectoryPoint): string {
  const parts = [`ts=${p.timestamp}`, `pos=(${p.x.toFixed(1)}, ${p.y.toFixed(1)})`];
  if (p.raw.ang_kf !== undefined) parts.push(`ang_kf=${p.raw.ang_kf.toFixed(1)}°`);
  parts.push(`ang_corrected=${((p.angleCorrected * 180) / Math.PI).toFixed(1)}°`);
  if (p.raw.left45_d !== undefined) parts.push(`left45_d=${p.raw.left45_d.toFixed(1)}`);
  if (p.raw.right45_d !== undefined) parts.push(`right45_d=${p.raw.right45_d.toFixed(1)}`);
  return parts.join(" | ");
}

interface AutoOpenRequest {
  file: string;
  nonce: number;
}

export function LogPlotPanel({ autoOpen }: { autoOpen?: AutoOpenRequest | null }) {
  const [files, setFiles] = useState<LogFileInfo[]>([]);
  const [selected, setSelected] = useState<string | null>(null);
  const [csvText, setCsvText] = useState<string | null>(null);
  const [showLeft45, setShowLeft45] = useState(true);
  const [showRight45, setShowRight45] = useState(true);
  const [clickInfo, setClickInfo] = useState<string | null>(null);
  const [pjBusy, setPjBusy] = useState(false);

  // analyze_sensor_drop.py 相当のオーバーレイ設定
  const [dropEnabled, setDropEnabled] = useState(false);
  const [dropMotionState, setDropMotionState] = useState(4);
  const [dropLow, setDropLow] = useState(70);
  const [dropHigh, setDropHigh] = useState(80);
  const [dropColLeft, setDropColLeft] = useState(true);
  const [dropColRight, setDropColRight] = useState(true);

  // analyze_motion_state_transitions.py 相当のオーバーレイ設定
  const [transitionEnabled, setTransitionEnabled] = useState(false);
  const [transitionStates, setTransitionStates] = useState("6,13");

  // analyze_sensor_trough.py 相当のオーバーレイ設定
  const [troughEnabled, setTroughEnabled] = useState(false);
  const [troughMotionState, setTroughMotionState] = useState(4);
  const [troughStates, setTroughStates] = useState("14,1");
  const [troughEps, setTroughEps] = useState(3);
  const [troughMedianWindow, setTroughMedianWindow] = useState(3);
  const [troughColumns, setTroughColumns] = useState<Record<(typeof TROUGH_COLUMNS)[number], boolean>>({
    left45_d: true,
    right45_d: true,
    left90_d: false,
    right90_d: false,
  });

  const refreshFiles = useCallback(async () => {
    const res = await fetch("/api/logs");
    const data = await res.json();
    const nextFiles = data.files as LogFileInfo[];
    setFiles(nextFiles);
    setSelected((prev) => (prev && nextFiles.some((f) => f.name === prev) ? prev : (nextFiles[0]?.name ?? null)));
  }, []);

  useEffect(() => {
    // eslint-disable-next-line react-hooks/set-state-in-effect
    void refreshFiles();
    const interval = setInterval(() => void refreshFiles(), 3000);
    return () => clearInterval(interval);
  }, [refreshFiles]);

  useEffect(() => {
    if (!selected) {
      // eslint-disable-next-line react-hooks/set-state-in-effect
      setCsvText(null);
      return;
    }
    let cancelled = false;
    void fetch(`/api/logs/content?name=${encodeURIComponent(selected)}`)
      .then((res) => res.text())
      .then((text) => {
        if (!cancelled) setCsvText(text);
      });
    return () => {
      cancelled = true;
    };
  }, [selected]);

  // 解析(旧 analyze_*.py)は CSV ファイル出現順に対して行うため、
  // trajectory.ts のタイムスタンプソート済み行とは別に生の行を保持する。
  const rawRows = useMemo(() => (csvText ? parseCsv(csvText) : []), [csvText]);

  const trajectoryData = useMemo(() => buildTrajectoryData(rawRows), [rawRows]);

  // buildTrajectoryData() reuses row objects verbatim as TrajectoryPoint.raw,
  // so this identity-keyed map lets drop/rise markers re-anchor to the
  // sensor's projected wall position (same as the left45/right45 dots)
  // instead of the robot's own (x, y).
  const pointByRow = useMemo(
    () => new Map(trajectoryData?.allPoints.map((p) => [p.raw, p]) ?? []),
    [trajectoryData]
  );

  const dropEvents = useMemo<AnalysisEvent[]>(() => {
    if (!dropEnabled || rawRows.length === 0) return [];
    const columns = [dropColLeft && "left45_d", dropColRight && "right45_d"].filter(Boolean) as string[];
    if (columns.length === 0) return [];
    return computeSensorDropEvents(rawRows, {
      motionState: dropMotionState,
      low: dropLow,
      high: dropHigh,
      columns,
      pointByRow,
    });
  }, [rawRows, dropEnabled, dropMotionState, dropLow, dropHigh, dropColLeft, dropColRight, pointByRow]);

  const transitionEvents = useMemo<AnalysisEvent[]>(() => {
    if (!transitionEnabled || rawRows.length === 0) return [];
    const states = transitionStates
      .split(",")
      .map((s) => parseFloat(s.trim()))
      .filter((n) => !Number.isNaN(n));
    if (states.length === 0) return [];
    return computeMotionTransitionEvents(rawRows, { states, columns: TRANSITION_COLUMNS });
  }, [rawRows, transitionEnabled, transitionStates]);

  const troughEvents = useMemo<AnalysisEvent[]>(() => {
    if (!troughEnabled || rawRows.length === 0) return [];
    const columns = TROUGH_COLUMNS.filter((c) => troughColumns[c]);
    if (columns.length === 0) return [];
    const states = troughStates
      .split(",")
      .map((s) => parseFloat(s.trim()))
      .filter((n) => !Number.isNaN(n));
    if (states.length === 0) return [];
    return computeSensorTroughEvents(rawRows, {
      motionState: troughMotionState,
      states,
      eps: troughEps,
      medianWindow: troughMedianWindow,
      columns,
      pointByRow,
    });
  }, [
    rawRows,
    troughEnabled,
    troughMotionState,
    troughStates,
    troughEps,
    troughMedianWindow,
    troughColumns,
    pointByRow,
  ]);

  const analysisEvents = useMemo(
    () => [...dropEvents, ...transitionEvents, ...troughEvents],
    [dropEvents, transitionEvents, troughEvents]
  );

  const openPlotJuggler = useCallback(async (name?: string) => {
    const target = name ?? selected;
    if (!target) return;
    setPjBusy(true);
    try {
      const res = await fetch("/api/logs/plotjuggler", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: "open", name: target }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "起動に失敗しました");
      toast.success(`PlotJuggler で開いています: ${target}`);
    } catch (err) {
      toast.error((err as Error).message);
    } finally {
      setPjBusy(false);
    }
  }, [selected]);

  // Triggered by clicking the "PlotJugglerで開く" action on the save
  // notification toast (see app/page.tsx's "saved" SSE handler). Keyed off
  // `nonce` so repeated requests for the same file still re-fire.
  useEffect(() => {
    if (!autoOpen) return;
    // eslint-disable-next-line react-hooks/set-state-in-effect
    setSelected(autoOpen.file);
    void openPlotJuggler(autoOpen.file);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [autoOpen?.nonce]);

  const copyFileName = async (name: string) => {
    try {
      await navigator.clipboard.writeText(name);
      toast.success(`コピーしました: ${name}`);
    } catch (err) {
      toast.error((err as Error).message);
    }
  };

  const openLogsFolder = async () => {
    try {
      const res = await fetch("/api/logs/open-folder", { method: "POST" });
      if (!res.ok) {
        const data = await res.json();
        throw new Error(data.error ?? "フォルダを開けませんでした");
      }
    } catch (err) {
      toast.error((err as Error).message);
    }
  };

  const killPlotJuggler = async () => {
    try {
      const res = await fetch("/api/logs/plotjuggler", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: "kill" }),
      });
      if (!res.ok) throw new Error("終了に失敗しました");
      toast.success("PlotJuggler を終了しました");
    } catch (err) {
      toast.error((err as Error).message);
    }
  };

  return (
    <Card className="flex flex-1 flex-row overflow-hidden">
      <div className="flex w-56 shrink-0 flex-col overflow-hidden border-r border-border">
        <div className="flex items-center justify-between p-2">
          <span className="text-sm font-medium">ログファイル</span>
          <div className="flex gap-1">
            <Button size="sm" variant="ghost" onClick={() => void openLogsFolder()}>
              フォルダを開く
            </Button>
            <Button size="sm" variant="ghost" onClick={() => void refreshFiles()}>
              更新
            </Button>
          </div>
        </div>
        <Separator />
        <ScrollArea className="min-h-0 flex-1">
          <div className="flex flex-col p-1">
            {files.map((f) => (
              <div
                key={f.name}
                role="button"
                tabIndex={0}
                onClick={() => setSelected(f.name)}
                onDoubleClick={() => void openPlotJuggler(f.name)}
                onKeyDown={(e) => {
                  if (e.key === "Enter" || e.key === " ") setSelected(f.name);
                }}
                className={`group flex items-center justify-between gap-1 rounded px-2 py-1.5 text-left text-xs transition-colors ${
                  selected === f.name ? "bg-primary text-primary-foreground" : "hover:bg-muted"
                }`}
              >
                <span className="flex min-w-0 flex-col">
                  <span className="truncate font-medium">{f.name}</span>
                  <span className={selected === f.name ? "text-primary-foreground/70" : "text-muted-foreground"}>
                    {formatDate(f.mtimeMs)}
                  </span>
                </span>
                <Button
                  size="icon-xs"
                  variant="ghost"
                  className={`shrink-0 opacity-0 group-hover:opacity-100 ${
                    selected === f.name ? "hover:bg-primary-foreground/20" : ""
                  }`}
                  onClick={(e) => {
                    e.stopPropagation();
                    void copyFileName(f.name);
                  }}
                  title="ファイル名をコピー"
                >
                  <CopyIcon />
                </Button>
              </div>
            ))}
            {files.length === 0 && (
              <span className="px-2 py-1 text-xs text-muted-foreground">ログファイルがありません</span>
            )}
          </div>
        </ScrollArea>
      </div>

      <div className="flex flex-1 flex-col overflow-hidden">
        <div className="flex items-center gap-2 p-2">
          <label className="flex items-center gap-1 text-xs">
            <input type="checkbox" checked={showLeft45} onChange={(e) => setShowLeft45(e.target.checked)} />
            Left45
          </label>
          <label className="flex items-center gap-1 text-xs">
            <input type="checkbox" checked={showRight45} onChange={(e) => setShowRight45(e.target.checked)} />
            Right45
          </label>
          <div className="flex-1" />
          <Button size="sm" variant="outline" disabled={!selected || pjBusy} onClick={() => void openPlotJuggler()}>
            PlotJugglerで開く
          </Button>
          <Button size="sm" variant="outline" onClick={killPlotJuggler}>
            PJを終了
          </Button>
        </div>
        <Separator />
        <div className="flex flex-wrap items-center gap-x-4 gap-y-1 p-2 text-xs">
          <label className="flex items-center gap-1">
            <input type="checkbox" checked={dropEnabled} onChange={(e) => setDropEnabled(e.target.checked)} />
            センサードロップ解析
          </label>
          {dropEnabled && (
            <>
              <label className="flex items-center gap-1">
                motion_state
                <input
                  type="number"
                  className="w-14 rounded border border-border bg-background px-1"
                  value={dropMotionState}
                  onChange={(e) => setDropMotionState(parseFloat(e.target.value))}
                />
              </label>
              <label className="flex items-center gap-1">
                low
                <input
                  type="number"
                  className="w-14 rounded border border-border bg-background px-1"
                  value={dropLow}
                  onChange={(e) => setDropLow(parseFloat(e.target.value))}
                />
              </label>
              <label className="flex items-center gap-1">
                high
                <input
                  type="number"
                  className="w-14 rounded border border-border bg-background px-1"
                  value={dropHigh}
                  onChange={(e) => setDropHigh(parseFloat(e.target.value))}
                />
              </label>
              <label className="flex items-center gap-1">
                <input type="checkbox" checked={dropColLeft} onChange={(e) => setDropColLeft(e.target.checked)} />
                left45_d
              </label>
              <label className="flex items-center gap-1">
                <input type="checkbox" checked={dropColRight} onChange={(e) => setDropColRight(e.target.checked)} />
                right45_d
              </label>
            </>
          )}
        </div>
        <div className="flex flex-wrap items-center gap-x-4 gap-y-1 px-2 pb-2 text-xs">
          <label className="flex items-center gap-1">
            <input
              type="checkbox"
              checked={transitionEnabled}
              onChange={(e) => setTransitionEnabled(e.target.checked)}
            />
            状態遷移解析
          </label>
          {transitionEnabled && (
            <label className="flex items-center gap-1">
              states
              <input
                type="text"
                className="w-24 rounded border border-border bg-background px-1"
                value={transitionStates}
                onChange={(e) => setTransitionStates(e.target.value)}
              />
            </label>
          )}
        </div>
        <div className="flex flex-wrap items-center gap-x-4 gap-y-1 px-2 pb-2 text-xs">
          <label className="flex items-center gap-1">
            <input type="checkbox" checked={troughEnabled} onChange={(e) => setTroughEnabled(e.target.checked)} />
            センサートラフ解析
          </label>
          {troughEnabled && (
            <>
              <label className="flex items-center gap-1">
                motion_state終了
                <input
                  type="number"
                  className="w-14 rounded border border-border bg-background px-1"
                  value={troughMotionState}
                  onChange={(e) => setTroughMotionState(parseFloat(e.target.value))}
                />
              </label>
              <label className="flex items-center gap-1">
                探索区間states
                <input
                  type="text"
                  className="w-16 rounded border border-border bg-background px-1"
                  value={troughStates}
                  onChange={(e) => setTroughStates(e.target.value)}
                />
              </label>
              <label className="flex items-center gap-1">
                eps
                <input
                  type="number"
                  className="w-14 rounded border border-border bg-background px-1"
                  value={troughEps}
                  onChange={(e) => setTroughEps(parseFloat(e.target.value))}
                />
              </label>
              <label className="flex items-center gap-1">
                median窓
                <input
                  type="number"
                  min={1}
                  step={2}
                  className="w-14 rounded border border-border bg-background px-1"
                  value={troughMedianWindow}
                  onChange={(e) => setTroughMedianWindow(parseInt(e.target.value, 10))}
                />
              </label>
              {TROUGH_COLUMNS.map((col) => (
                <label key={col} className="flex items-center gap-1">
                  <input
                    type="checkbox"
                    checked={troughColumns[col]}
                    onChange={(e) => setTroughColumns((prev) => ({ ...prev, [col]: e.target.checked }))}
                  />
                  {col}
                </label>
              ))}
            </>
          )}
        </div>
        <Separator />
        <div className="min-h-0 flex-1">
          <TrajectoryPlot
            data={trajectoryData}
            showLeft45={showLeft45}
            showRight45={showRight45}
            markers={analysisEvents}
            onPointClick={(p) => setClickInfo(p ? formatClickInfo(p) : null)}
          />
        </div>
        <Separator />
        <div className="p-2 font-mono text-xs text-muted-foreground">
          {clickInfo ?? (trajectoryData ? "点をクリックすると詳細を表示します" : "x/y列を含むログを選択してください")}
        </div>
        {analysisEvents.length > 0 && (
          <>
            <Separator />
            <ScrollArea className="max-h-32 min-h-0">
              <div className="flex flex-col gap-0.5 p-2 font-mono text-xs">
                {analysisEvents.map((ev, i) => (
                  <span key={i} className={EVENT_COLOR[ev.kind]}>
                    {ev.label}
                  </span>
                ))}
              </div>
            </ScrollArea>
          </>
        )}
      </div>
    </Card>
  );
}
