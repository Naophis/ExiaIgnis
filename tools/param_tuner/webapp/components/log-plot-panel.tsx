"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useRouter } from "next/navigation";
import { CopyIcon } from "lucide-react";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { ResizableHandle, ResizablePanel, ResizablePanelGroup } from "@/components/ui/resizable";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { SensorTimeseriesPlot, type TimeSeries } from "@/components/sensor-timeseries-plot";
import { TrajectoryPlot, type TrajectoryHighlight } from "@/components/trajectory-plot";
import { AnalysisToggles } from "@/components/analysis-toggles";
import {
  EVENT_COLOR,
  TROUGH_COLUMN_COLOR,
  TROUGH_COLUMNS,
  TROUGH_COMPANION_COLUMN,
  useAnalysisEvents,
  useAnalysisSettings,
} from "@/lib/use-analysis";
import { DEFAULT_POST_TICKS, formatTurnExit, turnKey, type TurnExitRow } from "@/lib/turn-exit";
import {
  TipLayer,
  TurnExitSummaryTable,
  TurnExitTable,
  useTip,
  type TurnExitSummaryData,
} from "@/components/turn-exit-table";
import { buildTrajectoryData, DEFAULT_X_OFFSET, parseCsv, type TrajectoryPoint } from "@/lib/trajectory";

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

export function LogPlotPanel({
  autoOpen,
  onAutoOpenHandled,
}: {
  autoOpen?: AutoOpenRequest | null;
  onAutoOpenHandled?: () => void;
}) {
  const [files, setFiles] = useState<LogFileInfo[]>([]);
  const [selected, setSelected] = useState<string | null>(null);
  const [csvText, setCsvText] = useState<string | null>(null);
  const [showLeft45, setShowLeft45] = useState(true);
  const [showRight45, setShowRight45] = useState(true);
  // WALL_OFF中の4kHz相当サンプル(hf_*列、2026-09-15以降のfirmware)の点群と
  // 壁切れ検出マーカー
  const [showHf, setShowHf] = useState(true);
  // trajectory.ts's world-space x origin - depends on where this robot's
  // sensor/frame zero sits relative to the maze grid, so it's a user field
  // rather than a fixed constant.
  const [xOffset, setXOffset] = useState(DEFAULT_X_OFFSET);
  const [clickInfo, setClickInfo] = useState<string | null>(null);
  const [pjBusy, setPjBusy] = useState(false);

  // 解析の設定は詳細ログ解析ページ(/logs)と共有(lib/use-analysis.ts)
  const analysis = useAnalysisSettings();
  const [turnExitSummary, setTurnExitSummary] = useState<TurnExitSummaryData | null>(null);
  const [turnExitBusy, setTurnExitBusy] = useState(false);
  // 通常の横軸=index の時系列折れ線グラフ(空間プロットとは別物)。この画面だけの表示切替。
  const [chartEnabled, setChartEnabled] = useState(false);
  // 旋回テーブルで選択中の行(プロット上で旋回区間と旋回後の窓を強調する)
  const [selectedTurnKey, setSelectedTurnKey] = useState<string | null>(null);
  const router = useRouter();
  const { tip: turnTip, show: showTurnTip, hide: hideTurnTip } = useTip();

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

  const trajectoryData = useMemo(() => buildTrajectoryData(rawRows, xOffset), [rawRows, xOffset]);

  // buildTrajectoryData() reuses row objects verbatim as TrajectoryPoint.raw,
  // so this identity-keyed map lets drop/rise markers re-anchor to the
  // sensor's projected wall position (same as the left45/right45 dots)
  // instead of the robot's own (x, y).
  const pointByRow = useMemo(
    () => new Map(trajectoryData?.allPoints.map((p) => [p.raw, p]) ?? []),
    [trajectoryData]
  );

  const { events: analysisEvents, chartMarkers, turnExitRows } = useAnalysisEvents(analysis, {
    rawRows,
    pointByRow,
    xOffset,
    showHf,
    logName: selected?.replace(/\.csv$/, ""),
  });

  // 選択中の旋回: SLALOM 区間と、出口から DEFAULT_POST_TICKS(次の旋回で打ち切り)を
  // 生行オブジェクトの集合として渡す(TrajectoryPoint.raw と同一なので identity で引ける)。
  const turnHighlight = useMemo<TrajectoryHighlight | null>(() => {
    if (!selectedTurnKey) return null;
    const i = turnExitRows.findIndex((r) => turnKey(r) === selectedTurnKey);
    if (i < 0) return null;
    const t = turnExitRows[i];
    const nextTurnIdx = turnExitRows[i + 1]?.idx ?? rawRows.length;
    const postEnd = Math.min(t.exitIdx + DEFAULT_POST_TICKS, nextTurnIdx, rawRows.length);
    return {
      turn: new Set(rawRows.slice(t.idx, t.endIdx + 1)),
      post: new Set(rawRows.slice(t.exitIdx, postEnd)),
    };
  }, [selectedTurnKey, turnExitRows, rawRows]);

  const selectTurn = useCallback((t: TurnExitRow) => {
    setSelectedTurnKey((prev) => (prev === turnKey(t) ? null : turnKey(t)));
    setClickInfo(formatTurnExit(t));
  }, []);

  // 下のイベント一覧に出す分。turn-exit は旋回テーブルにあるので二重に出さない
  const listedEvents = useMemo(() => analysisEvents.filter((ev) => ev.kind !== "turn-exit"), [analysisEvents]);

  const fetchTurnExitSummary = useCallback(async (limit: number) => {
    setTurnExitBusy(true);
    try {
      const res = await fetch(`/api/logs/turn-exit?limit=${limit}`);
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "集計に失敗しました");
      setTurnExitSummary(data as TurnExitSummaryData);
    } catch (err) {
      toast.error((err as Error).message);
    } finally {
      setTurnExitBusy(false);
    }
  }, []);

  // 有効中は新しいログが保存されるたびに集計し直す。files[0] は常に latest.csv
  // (複製、mtime が最新)なので、その次のファイル名の変化を新ログの合図にする。
  const newestLog = files.find((f) => f.name !== "latest.csv")?.name ?? null;
  useEffect(() => {
    if (!analysis.turnExit.enabled) return;
    // fetch-on-change: 結果は非同期に setState する(CLAUDE.md「既知のハマりどころ」参照)
    // eslint-disable-next-line react-hooks/set-state-in-effect
    void fetchTurnExitSummary(analysis.turnExit.limit);
  }, [analysis.turnExit.enabled, analysis.turnExit.limit, newestLog, fetchTurnExitSummary]);

  const chartSeries = useMemo<TimeSeries[]>(() => {
    if (!chartEnabled || rawRows.length === 0) return [];
    const toPoints = (col: string) =>
      rawRows
        .map((r) => ({ x: r.index, y: r[col] }))
        .filter((p) => Number.isFinite(p.x) && Number.isFinite(p.y) && p.y > 0);

    const series: TimeSeries[] = [];
    for (const col of TROUGH_COLUMNS) {
      if (!analysis.trough.columns[col]) continue;
      series.push({ column: col, color: TROUGH_COLUMN_COLOR[col], points: toPoints(col) });

      const companion = TROUGH_COMPANION_COLUMN[col];
      if (companion && companion in rawRows[0]) {
        series.push({ column: companion, color: TROUGH_COLUMN_COLOR[col], dash: true, points: toPoints(companion) });
      }
    }
    return series;
  }, [chartEnabled, rawRows, analysis.trough.columns]);

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
  //
  // The request must be strictly one-shot. Effects also run on every mount
  // (and on every Fast Refresh in dev), not just when `nonce` changes, and
  // this panel is unmounted whenever the editor/templates/matrix view is
  // showing - so a request left behind in the parent's state used to relaunch
  // PlotJuggler with the same stale file each time one of those views was
  // closed. Hand the request back to the parent to clear once consumed; the
  // ref covers StrictMode's double-invoked mount effect, where both runs see
  // the same props before the parent's state update lands.
  const handledNonceRef = useRef<number | null>(null);
  useEffect(() => {
    if (!autoOpen) return;
    if (handledNonceRef.current === autoOpen.nonce) return;
    handledNonceRef.current = autoOpen.nonce;
    setSelected(autoOpen.file);
    void openPlotJuggler(autoOpen.file);
    onAutoOpenHandled?.();
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
      <ResizablePanelGroup direction="horizontal" autoSaveId="param-console-logplot">
      <ResizablePanel defaultSize={22} minSize={12} maxSize={45} className="min-w-0">
      <div className="flex h-full flex-col overflow-hidden border-r border-border">
        <div className="flex items-center justify-between px-2 py-1">
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
          <div className="flex flex-col p-0.5">
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
                className={`group flex items-center justify-between gap-1 rounded px-1.5 py-1 text-left text-xs transition-colors ${
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
              <span className="px-1.5 py-0.5 text-xs text-muted-foreground">ログファイルがありません</span>
            )}
          </div>
        </ScrollArea>
      </div>
      </ResizablePanel>
      <ResizableHandle withHandle />
      <ResizablePanel defaultSize={78} minSize={30} className="min-w-0">
      <div className="flex h-full flex-col overflow-hidden">
        {/* 表示トグル・解析トグル(チップ)・PlotJuggler ボタンを1行にまとめる。
            有効化した解析だけパラメータが横に展開し、足りなければ折り返す。 */}
        <div className="flex flex-wrap items-center gap-x-2 gap-y-0.5 px-1.5 py-0.5 text-xs">
          <label className="flex items-center gap-1 text-xs">
            <input type="checkbox" checked={showLeft45} onChange={(e) => setShowLeft45(e.target.checked)} />
            Left45
          </label>
          <label className="flex items-center gap-1 text-xs">
            <input type="checkbox" checked={showRight45} onChange={(e) => setShowRight45(e.target.checked)} />
            Right45
          </label>
          <label
            className="flex items-center gap-1 text-xs"
            title="WALL_OFF中の4kHz相当サンプル(hf_*列)を行の姿勢から位置内挿して投影した点群と、firmwareの壁切れ検出位置(◇/□)"
          >
            <input type="checkbox" checked={showHf} onChange={(e) => setShowHf(e.target.checked)} />
            hf点群
          </label>
          <label className="flex items-center gap-1 text-xs" title="プロットの原点Xオフセット(mm)。ロボットのセンサー/座標系の原点とグリッドのズレを補正する">
            原点Xオフセット
            <input
              type="number"
              step={0.1}
              className="w-16 rounded border border-border bg-background px-1"
              value={xOffset}
              onChange={(e) => setXOffset(parseFloat(e.target.value))}
            />
          </label>
        <AnalysisToggles
          settings={analysis}
          showTurnExitSummary
          summaryBusy={turnExitBusy}
          onRefreshSummary={() => void fetchTurnExitSummary(analysis.turnExit.limit)}
        />
        <div className="flex flex-wrap items-center gap-x-2 gap-y-0.5 rounded border border-border/60 px-1 py-0">
          <label
            className="flex items-center gap-1"
            title="センサートラフ解析のチェック列を index 軸で表示。壁切れエッジ/hf のマーカーも重畳"
          >
            <input type="checkbox" checked={chartEnabled} onChange={(e) => setChartEnabled(e.target.checked)} />
            時系列
          </label>
        </div>
          <div className="flex-1" />
          {/* 1本を画面いっぱいで精査する別ページ(/logs)。このパネルは走った直後に
              その場で見る用として残してある。 */}
          <Button
            size="xs"
            variant="outline"
            disabled={!selected}
            title="このログを詳細ログ解析ページ(軌跡 + 連動カーソルの時系列グラフ)で開く"
            onClick={() => selected && router.push(`/logs?file=${encodeURIComponent(selected)}`)}
          >
            詳細解析
          </Button>
          <Button size="xs" variant="outline" disabled={!selected || pjBusy} onClick={() => void openPlotJuggler()}>
            PlotJugglerで開く
          </Button>
          <Button size="xs" variant="outline" onClick={killPlotJuggler}>
            PJを終了
          </Button>
        </div>
        <Separator />
        {/* 迷路プロットは正方形で横に余るので、旋回出口解析の表はプロットの右に
            横分割で置く(既定はプロット 55%、ハンドルでドラッグ可)。表側は
            collapsible なので、ハンドルを右端まで寄せれば畳める。 */}
        <ResizablePanelGroup direction="horizontal" className="min-h-0 flex-1">
          <ResizablePanel order={1} defaultSize={52} minSize={25} className="min-w-0 min-h-0">
            <div className="flex h-full flex-col overflow-hidden">
              <div className="min-h-0 flex-1">
                <TrajectoryPlot
                  data={trajectoryData}
                  showLeft45={showLeft45}
                  showRight45={showRight45}
                  showHf={showHf}
                  markers={analysisEvents}
                  highlight={analysis.turnExit.enabled ? turnHighlight : null}
                  onPointClick={(p) => setClickInfo(p ? formatClickInfo(p) : null)}
                />
              </div>
              {chartEnabled && chartSeries.length > 0 && (
                <>
                  <Separator />
                  <div className="h-56 shrink-0">
                    <SensorTimeseriesPlot series={chartSeries} markers={chartMarkers} />
                  </div>
                </>
              )}
            </div>
          </ResizablePanel>
          {analysis.turnExit.enabled && (
            <>
              <ResizableHandle withHandle />
              <ResizablePanel order={2} defaultSize={48} minSize={10} collapsible className="min-w-0 min-h-0">
                <div className="flex h-full flex-col overflow-hidden">
                  <ScrollArea className="min-h-0 flex-1">
                    <div className="flex flex-col gap-1.5 p-1.5">
                      {turnExitRows.length > 0 && (
                        <TurnExitTable
                          rows={turnExitRows}
                          selectedKey={selectedTurnKey}
                          onSelect={selectTurn}
                          onTip={showTurnTip}
                          onTipHide={hideTurnTip}
                        />
                      )}
                      {turnExitRows.length === 0 && rawRows.length > 0 && (
                        <span className="text-xs text-muted-foreground">
                          このログに SLALOM 区間がないか、必要な列(kim_theta / s_pid_p 等)がありません
                        </span>
                      )}
                      {turnExitSummary && (
                        <TurnExitSummaryTable data={turnExitSummary} onTip={showTurnTip} onTipHide={hideTurnTip} />
                      )}
                    </div>
                  </ScrollArea>
                </div>
              </ResizablePanel>
            </>
          )}
        </ResizablePanelGroup>
        <Separator />
        <div className="px-2 py-1 font-mono text-xs text-muted-foreground">
          {clickInfo ?? (trajectoryData ? "点をクリックすると詳細を表示します" : "x/y列を含むログを選択してください")}
        </div>
        {listedEvents.length > 0 && (
          <>
            <Separator />
            <ScrollArea className="max-h-32 min-h-0">
              <div className="flex flex-col gap-0.5 px-2 py-1 font-mono text-xs">
                {listedEvents.map((ev, i) => (
                  <span key={i} className={EVENT_COLOR[ev.kind]}>
                    {ev.label}
                  </span>
                ))}
              </div>
            </ScrollArea>
          </>
        )}
      </div>
      </ResizablePanel>
      </ResizablePanelGroup>
      <TipLayer tip={turnTip} />
    </Card>
  );
}
