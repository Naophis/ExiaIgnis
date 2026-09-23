"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useRouter } from "next/navigation";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { ResizableHandle, ResizablePanel, ResizablePanelGroup } from "@/components/ui/resizable";
import { ScrollArea } from "@/components/ui/scroll-area";
import { AnalysisToggles } from "@/components/analysis-toggles";
import {
  SensorTimeseriesPlot,
  type ChartBand,
  type Domain,
  type TimeSeries,
} from "@/components/sensor-timeseries-plot";
import { TrajectoryPlot, type TrajectoryHighlight } from "@/components/trajectory-plot";
import { TipLayer, TurnExitTable, useTip } from "@/components/turn-exit-table";
import { buildTrajectoryData, DEFAULT_X_OFFSET, parseCsv, type TrajectoryPoint } from "@/lib/trajectory";
import { DEFAULT_POST_TICKS, formatTurnExit, turnKey, type TurnExitRow } from "@/lib/turn-exit";
import { EVENT_COLOR, useAnalysisEvents, useAnalysisSettings } from "@/lib/use-analysis";
import {
  CHART_VIEWS,
  columnSpecLabel,
  CURSOR_READOUT_COLUMNS,
  DEFAULT_CHART_HEIGHT,
  DEFAULT_VIEW_KEY,
  MOTION_STATE_BAND,
  motionStateBlocks,
  motionStateLabel,
  parseColumnSpec,
  SERIES_COLORS,
  usableColumns,
  viewByKey,
  type ChartSpec,
} from "@/lib/log-columns";

// 表示設定の保存先。グラフの構成(列・高さ・並び順)と表示トグルをまとめて覚える。
// 左右の分割幅は ResizablePanelGroup の autoSaveId が別に保存する。
const PREFS_KEY = "exia-log-detail-prefs-v1";
// 旋回へジャンプしたときに前後に確保する tick(入口の助走と出口の収束を含めて見る)
const TURN_ZOOM_PRE = 40;
const TURN_ZOOM_POST = 90;

interface LogFileInfo {
  name: string;
  mtimeMs: number;
  size: number;
}

interface ViewPrefs {
  /** 観点(CHART_VIEWS のキー)ごとのグラフ構成。編集した観点だけ入る。 */
  chartsByView: Record<string, ChartSpec[]>;
  viewKey: string;
  xOffset: number;
  showLeft45: boolean;
  showRight45: boolean;
  showHf: boolean;
  showBands: boolean;
  showTurnTable: boolean;
  showEvents: boolean;
}

function newId(): string {
  return Math.random().toString(36).slice(2, 9);
}

// プリセットの id は観点キー+並び順から決める(毎回ランダムにすると React の key と
// 系列キャッシュが毎描画で変わってしまう)。ユーザーが足したグラフだけランダム id。
function presetCharts(viewKey: string): ChartSpec[] {
  return viewByKey(viewKey).charts.map((c, i) => ({ ...c, id: `${viewKey}#${i}` }));
}

/** 列を選ぶポップオーバー。150列超あるので絞り込み必須。 */
function ColumnPicker({
  columns,
  selected,
  onToggle,
}: {
  columns: string[];
  selected: string[];
  onToggle: (column: string) => void;
}) {
  const [open, setOpen] = useState(false);
  const [q, setQ] = useState("");
  const hits = useMemo(() => {
    const needle = q.trim().toLowerCase();
    const list = needle ? columns.filter((c) => c.toLowerCase().includes(needle)) : columns;
    return list.slice(0, 300);
  }, [columns, q]);
  return (
    <span className="relative">
      <Button size="xs" variant="outline" onClick={() => setOpen((v) => !v)}>
        列 +
      </Button>
      {open && (
        <>
          <div className="fixed inset-0 z-40" onClick={() => setOpen(false)} />
          <div className="absolute right-0 z-50 mt-1 w-64 rounded border border-border bg-popover p-1.5 shadow-lg">
            <input
              autoFocus
              type="text"
              placeholder="列名で絞り込み"
              className="mb-1 w-full rounded border border-border bg-background px-1.5 py-1 text-xs"
              value={q}
              onChange={(e) => setQ(e.target.value)}
            />
            <div className="max-h-64 overflow-y-auto">
              {hits.map((c) => (
                <label
                  key={c}
                  className="flex cursor-pointer items-center gap-1.5 rounded px-1 py-0.5 text-xs hover:bg-muted"
                >
                  <input type="checkbox" checked={selected.includes(c)} onChange={() => onToggle(c)} />
                  <span className="truncate font-mono">{c}</span>
                </label>
              ))}
              {hits.length === 0 && <span className="px-1 text-xs text-muted-foreground">該当なし</span>}
            </div>
          </div>
        </>
      )}
    </span>
  );
}

export function LogDetailView({ initialFile }: { initialFile?: string }) {
  const router = useRouter();
  const [files, setFiles] = useState<LogFileInfo[]>([]);
  const [selected, setSelected] = useState<string | null>(initialFile ?? null);
  const [csvText, setCsvText] = useState<string | null>(null);
  const [viewKey, setViewKey] = useState(DEFAULT_VIEW_KEY);
  const [chartsByView, setChartsByView] = useState<Record<string, ChartSpec[]>>({});
  const [domain, setDomain] = useState<Domain | null>(null);
  const [cursorIndex, setCursorIndex] = useState<number | null>(null);
  const [xOffset, setXOffset] = useState(DEFAULT_X_OFFSET);
  const [showLeft45, setShowLeft45] = useState(true);
  const [showRight45, setShowRight45] = useState(true);
  const [showHf, setShowHf] = useState(true);
  const [showBands, setShowBands] = useState(true);
  const [showTurnTable, setShowTurnTable] = useState(false);
  const [showEvents, setShowEvents] = useState(false);
  const [selectedTurnKey, setSelectedTurnKey] = useState<string | null>(null);
  const [clickInfo, setClickInfo] = useState<string | null>(null);
  const { tip, show: showTip, hide: hideTip } = useTip();

  // 解析の設定はプロットタブ(log-plot-panel.tsx)と共有(lib/use-analysis.ts)。
  // このページは旋回ストリップが主な移動手段なので旋回出口だけ既定で ON。
  const analysis = useAnalysisSettings({ turnExitEnabled: true });

  // 表示設定の読み書き。読めない/壊れていれば既定値のまま進む。
  const prefsLoaded = useRef(false);
  useEffect(() => {
    if (prefsLoaded.current) return;
    prefsLoaded.current = true;
    try {
      const raw = window.localStorage.getItem(PREFS_KEY);
      if (!raw) return;
      const p = JSON.parse(raw) as Partial<ViewPrefs>;
      // マウント時の localStorage 読み出し(外部システムからの初期同期)。
      // このルールはこのパターンに対しては過検知(webapp/CLAUDE.md 参照)。
      /* eslint-disable react-hooks/set-state-in-effect */
      if (p.chartsByView && typeof p.chartsByView === "object") {
        const clean: Record<string, ChartSpec[]> = {};
        for (const [k, v] of Object.entries(p.chartsByView)) {
          if (Array.isArray(v) && v.every((c) => c && Array.isArray(c.columns))) {
            clean[k] = v.map((c, i) => ({ ...c, id: c.id || `${k}#${i}` }));
          }
        }
        setChartsByView(clean);
      }
      if (typeof p.viewKey === "string" && CHART_VIEWS.some((v) => v.key === p.viewKey)) setViewKey(p.viewKey);
      if (typeof p.xOffset === "number" && Number.isFinite(p.xOffset)) setXOffset(p.xOffset);
      if (typeof p.showLeft45 === "boolean") setShowLeft45(p.showLeft45);
      if (typeof p.showRight45 === "boolean") setShowRight45(p.showRight45);
      if (typeof p.showHf === "boolean") setShowHf(p.showHf);
      if (typeof p.showBands === "boolean") setShowBands(p.showBands);
      if (typeof p.showTurnTable === "boolean") setShowTurnTable(p.showTurnTable);
      if (typeof p.showEvents === "boolean") setShowEvents(p.showEvents);
      /* eslint-enable react-hooks/set-state-in-effect */
    } catch {
      // ignore
    }
  }, []);
  useEffect(() => {
    if (!prefsLoaded.current) return;
    const prefs: ViewPrefs = {
      chartsByView,
      viewKey,
      xOffset,
      showLeft45,
      showRight45,
      showHf,
      showBands,
      showTurnTable,
      showEvents,
    };
    try {
      window.localStorage.setItem(PREFS_KEY, JSON.stringify(prefs));
    } catch {
      // ignore
    }
  }, [chartsByView, viewKey, xOffset, showLeft45, showRight45, showHf, showBands, showTurnTable, showEvents]);

  useEffect(() => {
    let cancelled = false;
    void fetch("/api/logs")
      .then((res) => res.json())
      .then((data) => {
        if (cancelled) return;
        const next = data.files as LogFileInfo[];
        setFiles(next);
        setSelected((prev) => (prev && next.some((f) => f.name === prev) ? prev : (next[0]?.name ?? null)));
      });
    return () => {
      cancelled = true;
    };
  }, []);

  useEffect(() => {
    if (!selected) return;
    let cancelled = false;
    void fetch(`/api/logs/content?name=${encodeURIComponent(selected)}`)
      .then((res) => res.text())
      .then((text) => {
        if (cancelled) return;
        setCsvText(text);
        setDomain(null);
        setCursorIndex(null);
        setSelectedTurnKey(null);
        setClickInfo(null);
      });
    return () => {
      cancelled = true;
    };
  }, [selected]);

  // 表示中の観点のグラフ構成。編集していない観点はプリセットをそのまま使う。
  const charts = useMemo(() => chartsByView[viewKey] ?? presetCharts(viewKey), [chartsByView, viewKey]);
  const setCharts = useCallback(
    (updater: (prev: ChartSpec[]) => ChartSpec[]) => {
      setChartsByView((prev) => ({ ...prev, [viewKey]: updater(prev[viewKey] ?? presetCharts(viewKey)) }));
    },
    [viewKey]
  );

  const rawRows = useMemo(() => (csvText ? parseCsv(csvText) : []), [csvText]);
  const trajectoryData = useMemo(() => buildTrajectoryData(rawRows, xOffset), [rawRows, xOffset]);
  const columns = useMemo(() => usableColumns(rawRows), [rawRows]);
  const pointByRow = useMemo(
    () => new Map(trajectoryData?.allPoints.map((p) => [p.raw, p]) ?? []),
    [trajectoryData]
  );

  const {
    events: analysisEvents,
    chartMarkers,
    turnExitRows: turns,
  } = useAnalysisEvents(analysis, {
    rawRows,
    pointByRow,
    xOffset,
    showHf,
    logName: selected?.replace(/\.csv$/, ""),
  });

  const listedEvents = useMemo(() => analysisEvents.filter((ev) => ev.kind !== "turn-exit"), [analysisEvents]);

  const fullDomain = useMemo<Domain | null>(
    () => (rawRows.length > 0 ? { xMin: rawRows[0].index, xMax: rawRows[rawRows.length - 1].index } : null),
    [rawRows]
  );
  const rowByIndex = useMemo(() => new Map(rawRows.map((r) => [r.index, r])), [rawRows]);

  const bands = useMemo<ChartBand[]>(() => {
    if (!showBands || rawRows.length === 0) return [];
    return motionStateBlocks(rawRows)
      .map((b) => ({ x0: b.x0, x1: b.x1, color: MOTION_STATE_BAND[b.state] ?? "" }))
      .filter((b) => b.color !== "");
  }, [rawRows, showBands]);

  const seriesByChart = useMemo(() => {
    const out = new Map<string, TimeSeries[]>();
    for (const chart of charts) {
      const list: TimeSeries[] = [];
      chart.columns.forEach((spec, i) => {
        const { column, scale } = parseColumnSpec(spec);
        if (!(rawRows.length > 0 && column in rawRows[0])) return;
        const points: { x: number; y: number }[] = [];
        for (const r of rawRows) {
          if (Number.isFinite(r[column])) points.push({ x: r.index, y: r[column] * scale });
        }
        list.push({ column: columnSpecLabel(spec), color: SERIES_COLORS[i % SERIES_COLORS.length], points });
      });
      out.set(chart.id, list);
    }
    return out;
  }, [charts, rawRows]);

  const cursorRow = cursorIndex === null ? null : (rowByIndex.get(cursorIndex) ?? null);
  const cursorPoint: TrajectoryPoint | null = cursorRow ? (pointByRow.get(cursorRow) ?? null) : null;

  const turnHighlight = useMemo<TrajectoryHighlight | null>(() => {
    if (!selectedTurnKey) return null;
    const i = turns.findIndex((t) => turnKey(t) === selectedTurnKey);
    if (i < 0) return null;
    const t = turns[i];
    const nextTurnIdx = turns[i + 1]?.idx ?? rawRows.length;
    const postEnd = Math.min(t.exitIdx + DEFAULT_POST_TICKS, nextTurnIdx, rawRows.length);
    return {
      turn: new Set(rawRows.slice(t.idx, t.endIdx + 1)),
      post: new Set(rawRows.slice(t.exitIdx, postEnd)),
    };
  }, [selectedTurnKey, turns, rawRows]);

  const jumpToTurn = useCallback(
    (t: TurnExitRow) => {
      if (rawRows.length === 0) return;
      setSelectedTurnKey((prev) => (prev === turnKey(t) ? null : turnKey(t)));
      setDomain({
        xMin: rawRows[Math.max(0, t.idx - TURN_ZOOM_PRE)].index,
        xMax: rawRows[Math.min(rawRows.length - 1, t.exitIdx + TURN_ZOOM_POST)].index,
      });
      setCursorIndex(rawRows[t.idx].index);
      setClickInfo(formatTurnExit(t));
    },
    [rawRows]
  );

  const patchChart = (id: string, patch: Partial<ChartSpec>) =>
    setCharts((prev) => prev.map((c) => (c.id === id ? { ...c, ...patch } : c)));

  const toggleColumn = (chartId: string, column: string) =>
    setCharts((prev) =>
      prev.map((c) =>
        c.id === chartId
          ? {
              ...c,
              columns: c.columns.includes(column)
                ? c.columns.filter((x) => x !== column)
                : [...c.columns, column],
            }
          : c
      )
    );

  const moveChart = (id: string, delta: number) =>
    setCharts((prev) => {
      const i = prev.findIndex((c) => c.id === id);
      const j = i + delta;
      if (i < 0 || j < 0 || j >= prev.length) return prev;
      const next = [...prev];
      [next[i], next[j]] = [next[j], next[i]];
      return next;
    });

  // グラフの高さはドラッグで決めて localStorage に残す。
  const startResize = (e: React.PointerEvent, id: string, current: number) => {
    e.preventDefault();
    const startY = e.clientY;
    const onMove = (ev: PointerEvent) => {
      patchChart(id, { height: Math.max(80, Math.min(700, current + (ev.clientY - startY))) });
    };
    const onUp = () => {
      window.removeEventListener("pointermove", onMove);
      window.removeEventListener("pointerup", onUp);
    };
    window.addEventListener("pointermove", onMove);
    window.addEventListener("pointerup", onUp);
  };

  const effectiveDomain = domain ?? fullDomain;
  const zoomed = !!(domain && fullDomain && (domain.xMin !== fullDomain.xMin || domain.xMax !== fullDomain.xMax));

  return (
    <div className="flex h-screen flex-col gap-1.5 p-2">
      <div className="flex shrink-0 flex-wrap items-center gap-x-3 gap-y-1 rounded-xl border-l-2 border-l-accent-gold bg-card px-3 py-1 text-xs ring-1 ring-primary/20">
        <button
          type="button"
          onClick={() => router.push("/")}
          className="font-semibold tracking-wide text-accent-gold hover:underline"
        >
          ← Exia PARAM CONSOLE
        </button>
        <span className="text-muted-foreground">詳細ログ解析</span>
        <select
          className="max-w-72 rounded border border-border bg-background px-1.5 py-1 font-mono text-xs"
          value={selected ?? ""}
          onChange={(e) => setSelected(e.target.value)}
        >
          {files.map((f) => (
            <option key={f.name} value={f.name}>
              {f.name}
            </option>
          ))}
        </select>
        <label className="flex items-center gap-1">
          <input type="checkbox" checked={showLeft45} onChange={(e) => setShowLeft45(e.target.checked)} />
          Left45
        </label>
        <label className="flex items-center gap-1">
          <input type="checkbox" checked={showRight45} onChange={(e) => setShowRight45(e.target.checked)} />
          Right45
        </label>
        <label className="flex items-center gap-1">
          <input type="checkbox" checked={showHf} onChange={(e) => setShowHf(e.target.checked)} />
          hf点群
        </label>
        <label
          className="flex items-center gap-1"
          title="motion_state の区間をグラフ背景に色で敷く(旋回=紫、前後の繋ぎ=水色、壁切れ=緑、ピボット=桃)"
        >
          <input type="checkbox" checked={showBands} onChange={(e) => setShowBands(e.target.checked)} />
          状態帯
        </label>
        <label className="flex items-center gap-1" title="プロットの原点Xオフセット(mm)">
          原点X
          <input
            type="number"
            step={0.1}
            className="w-14 rounded border border-border bg-background px-1"
            value={xOffset}
            onChange={(e) => setXOffset(parseFloat(e.target.value))}
          />
        </label>
        <div className="flex-1" />
        {zoomed && (
          <Button size="xs" variant="outline" onClick={() => setDomain(null)}>
            全体に戻す
          </Button>
        )}
      </div>

      {/* 解析トグル(プロットタブと同じ設定・同じ計算) */}
      <div className="flex shrink-0 flex-wrap items-center gap-x-2 gap-y-0.5 px-1 text-xs">
        <AnalysisToggles settings={analysis} />
        {analysis.turnExit.enabled && (
          <label className="flex items-center gap-1" title="旋回出口の詳細テーブルを出す(行クリックでその旋回へズーム)">
            <input type="checkbox" checked={showTurnTable} onChange={(e) => setShowTurnTable(e.target.checked)} />
            旋回表
          </label>
        )}
        <label className="flex items-center gap-1" title="検出したイベントのラベル一覧を下に出す">
          <input type="checkbox" checked={showEvents} onChange={(e) => setShowEvents(e.target.checked)} />
          イベント一覧
        </label>
      </div>

      {analysis.turnExit.enabled && turns.length > 0 && (
        <div className="flex shrink-0 items-center gap-1 overflow-x-auto rounded border border-border/60 px-2 py-1">
          <span className="shrink-0 text-xs text-muted-foreground">旋回へ:</span>
          {turns.map((t) => (
            <button
              key={t.idx}
              type="button"
              onClick={() => jumpToTurn(t)}
              title={`idx ${t.idx}-${t.endIdx} / wide=${Number.isFinite(t.wide) ? t.wide.toFixed(1) : "–"}mm yaw0=${t.yaw0.toFixed(1)}° sat=${t.sat}`}
              className={`shrink-0 rounded border px-1.5 py-0.5 font-mono text-[11px] transition-colors ${
                selectedTurnKey === turnKey(t)
                  ? "border-fuchsia-400 bg-fuchsia-500/25"
                  : "border-border/60 hover:bg-muted"
              }`}
            >
              {t.idx} {t.kind} {t.dir}
            </button>
          ))}
        </div>
      )}

      {analysis.turnExit.enabled && showTurnTable && turns.length > 0 && (
        <div className="max-h-44 shrink-0 overflow-auto rounded border border-border/60 p-1">
          <TurnExitTable rows={turns} selectedKey={selectedTurnKey} onSelect={jumpToTurn} onTip={showTip} onTipHide={hideTip} />
        </div>
      )}

      <ResizablePanelGroup direction="horizontal" autoSaveId="exia-log-detail" className="min-h-0 flex-1">
        <ResizablePanel defaultSize={45} minSize={20} className="min-h-0 min-w-0">
          <Card className="flex h-full flex-col overflow-hidden p-0">
            <div className="min-h-0 flex-1">
              <TrajectoryPlot
                data={trajectoryData}
                showLeft45={showLeft45}
                showRight45={showRight45}
                showHf={showHf}
                markers={analysisEvents}
                highlight={turnHighlight}
                cursorPoint={cursorPoint}
                onPointClick={(p) => setCursorIndex(p ? p.raw.index : null)}
              />
            </div>
          </Card>
        </ResizablePanel>
        <ResizableHandle withHandle />
        <ResizablePanel defaultSize={55} minSize={25} className="min-h-0 min-w-0">
          <Card className="flex h-full flex-col overflow-hidden p-0">
            <div className="flex shrink-0 items-center gap-2 border-b border-border/60 px-1.5 py-1 text-xs">
              <label className="flex items-center gap-1" title="PlotJuggler のレイアウト(profile.xml)のタブと同じ組み合わせ。観点ごとに編集内容を覚える">
                観点
                <select
                  className="rounded border border-border bg-background px-1.5 py-0.5 text-xs"
                  value={viewKey}
                  onChange={(e) => setViewKey(e.target.value)}
                >
                  {CHART_VIEWS.map((v) => (
                    <option key={v.key} value={v.key}>
                      {v.title}
                    </option>
                  ))}
                </select>
              </label>
              <span className="text-muted-foreground">
                {charts.length} グラフ{chartsByView[viewKey] ? "(編集済み)" : ""}
              </span>
            </div>
            <ScrollArea className="min-h-0 flex-1">
              <div className="flex flex-col gap-1 p-1">
                {charts.map((chart, ci) => (
                  <div key={chart.id} className="rounded border border-border/60">
                    <div className="flex flex-wrap items-center gap-1 px-1.5 py-0.5 text-xs">
                      <span className="font-medium">{chart.title}</span>
                      {chart.columns.map((col, i) => (
                        <button
                          key={col}
                          type="button"
                          title="クリックでこの列を外す"
                          onClick={() => toggleColumn(chart.id, col)}
                          className="rounded px-1 font-mono text-[11px] hover:line-through"
                          style={{ color: SERIES_COLORS[i % SERIES_COLORS.length] }}
                        >
                          {columnSpecLabel(col)}
                        </button>
                      ))}
                      <div className="flex-1" />
                      <Button
                        size="xs"
                        variant="ghost"
                        disabled={ci === 0}
                        title="上へ"
                        onClick={() => moveChart(chart.id, -1)}
                      >
                        ↑
                      </Button>
                      <Button
                        size="xs"
                        variant="ghost"
                        disabled={ci === charts.length - 1}
                        title="下へ"
                        onClick={() => moveChart(chart.id, 1)}
                      >
                        ↓
                      </Button>
                      <ColumnPicker
                        columns={columns}
                        selected={chart.columns}
                        onToggle={(c) => toggleColumn(chart.id, c)}
                      />
                      <Button
                        size="xs"
                        variant="ghost"
                        title="このグラフを消す"
                        onClick={() => setCharts((prev) => prev.filter((c) => c.id !== chart.id))}
                      >
                        ×
                      </Button>
                    </div>
                    <div style={{ height: chart.height ?? DEFAULT_CHART_HEIGHT }}>
                      <SensorTimeseriesPlot
                        series={seriesByChart.get(chart.id) ?? []}
                        markers={chartMarkers}
                        bands={bands}
                        domain={effectiveDomain}
                        onDomainChange={setDomain}
                        cursorIndex={cursorIndex}
                        onCursorChange={setCursorIndex}
                        compact
                      />
                    </div>
                    {/* 下端ドラッグで高さを変える(保存される) */}
                    <div
                      onPointerDown={(e) => startResize(e, chart.id, chart.height ?? DEFAULT_CHART_HEIGHT)}
                      title="ドラッグで高さを変える"
                      className="h-1.5 cursor-ns-resize rounded-b bg-border/40 hover:bg-primary/40"
                    />
                  </div>
                ))}
                <div className="flex gap-1 p-1">
                  <Button
                    size="xs"
                    variant="outline"
                    onClick={() =>
                      setCharts((prev) => [
                        ...prev,
                        { id: newId(), title: "グラフ", columns: [], height: DEFAULT_CHART_HEIGHT },
                      ])
                    }
                  >
                    グラフ追加
                  </Button>
                  <Button
                    size="xs"
                    variant="ghost"
                    title="この観点のグラフ構成を profile.xml 由来の初期状態へ戻す"
                    onClick={() => setChartsByView((prev) => ({ ...prev, [viewKey]: presetCharts(viewKey) }))}
                  >
                    この観点を初期状態に戻す
                  </Button>
                </div>
              </div>
            </ScrollArea>
          </Card>
        </ResizablePanel>
      </ResizablePanelGroup>

      {showEvents && listedEvents.length > 0 && (
        <div className="max-h-28 shrink-0 overflow-auto rounded border border-border/60 p-1">
          <div className="flex flex-col gap-0.5 font-mono text-[11px]">
            {listedEvents.map((ev, i) => (
              <span key={i} className={EVENT_COLOR[ev.kind]}>
                {ev.label}
              </span>
            ))}
          </div>
        </div>
      )}

      <div className="shrink-0 px-1 font-mono text-[11px] text-muted-foreground">
        {clickInfo ? (
          <span className="text-foreground">{clickInfo}</span>
        ) : cursorRow ? (
          <span className="flex flex-wrap gap-x-3">
            <span className="text-foreground">index={cursorRow.index}</span>
            {CURSOR_READOUT_COLUMNS.map((c) =>
              Number.isFinite(cursorRow[c]) ? (
                <span key={c}>
                  {c}=
                  {c === "motion_state"
                    ? motionStateLabel(cursorRow[c])
                    : cursorRow[c].toFixed(Math.abs(cursorRow[c]) < 10 ? 2 : 1)}
                </span>
              ) : null
            )}
          </span>
        ) : (
          "グラフにマウスを載せるか軌跡の点をクリックすると値が出ます。グラフはホイール=ズーム、ドラッグ=パン(全グラフ連動)"
        )}
      </div>
      <TipLayer tip={tip} />
    </div>
  );
}
