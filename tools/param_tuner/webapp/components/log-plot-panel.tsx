"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { CopyIcon } from "lucide-react";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { ResizableHandle, ResizablePanel, ResizablePanelGroup } from "@/components/ui/resizable";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { SensorTimeseriesPlot, type TimeSeries } from "@/components/sensor-timeseries-plot";
import { TrajectoryPlot, type TrajectoryHighlight } from "@/components/trajectory-plot";
import { Table, TableBody, TableCell, TableHead, TableHeader, TableRow } from "@/components/ui/table";
import {
  computeHfEdgeEvents,
  computeMotionTransitionEvents,
  computeSensorDropEvents,
  computeSensorTroughEvents,
  computeWallOffEdgeEvents,
  type AnalysisEvent,
} from "@/lib/log-analysis";
import {
  analyzeTurnExits,
  DEFAULT_POST_TICKS,
  turnSensitivity,
  type Stat,
  type TurnExitRow,
  type TurnExitSummaryRow,
} from "@/lib/turn-exit";
import { buildTrajectoryData, DEFAULT_X_OFFSET, parseCsv, type TrajectoryPoint } from "@/lib/trajectory";

const TRANSITION_COLUMNS = ["left45_d", "left45_2_d", "left45_3_d", "right45_d", "right45_2_d", "right45_3_d"];
const TROUGH_COLUMNS = ["left45_d", "right45_d", "left90_d", "right90_d"] as const;

const COLUMN_COLOR: Record<(typeof TROUGH_COLUMNS)[number], string> = {
  left45_d: "#e06c75",
  right45_d: "#61afef",
  left90_d: "#98c379",
  right90_d: "#d19a66",
};

// left45_d/right45_d はセンサー生値からの変換後の距離。同時に、変換元の
// 生距離(sen_dist_l45/sen_dist_r45)も破線で重ねて見比べられるようにする。
const COMPANION_COLUMN: Partial<Record<(typeof TROUGH_COLUMNS)[number], string>> = {
  left45_d: "sen_dist_l45",
  right45_d: "sen_dist_r45",
};

const EVENT_COLOR: Record<AnalysisEvent["kind"], string> = {
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

interface TurnExitSummaryData {
  files: string[];
  skipped: string[];
  rows: TurnExitRow[];
  summary: TurnExitSummaryRow[];
}

const f1 = (x: number, digits = 1) => (Number.isFinite(x) ? x.toFixed(digits) : "–");
// API 経由の Stat は NaN が JSON で null になる(n=0)。表示・色付けとも n===0 を先に見る
const fmtStat = (st: Stat, digits = 1) => (st.n === 0 ? "–" : `${st.mean.toFixed(digits)}±${st.std.toFixed(digits)}`);
const statFlag = (st: Stat, cond: (mean: number) => boolean) => st.n > 0 && cond(st.mean);

// テーブル行の選択キー。ログが切り替わると自動的に一致しなくなる(idx だけだと
// 別ログの同じ行番号に当たりうる)。
const turnKey = (t: TurnExitRow) => `${t.log}|${t.idx}`;

function turnExitLabel(t: TurnExitRow): string {
  return `turn-exit idx=${t.idx} ${t.kind} ${t.dir} v=${t.v} wide=${f1(t.wide)} yaw0=${f1(t.yaw0)}° sat=${t.sat} v_in=${f1(t.vIn, 2)}`;
}

function formatTurnExit(t: TurnExitRow): string {
  return [
    `${t.kind} ${t.dir} v=${t.v} idx=${t.idx}-${t.endIdx} exit=${t.exitIdx}`,
    `wmax=${f1(t.wmax)} (${f1(t.latg)}G)`,
    `w+${f1(t.wOver)}/${f1(t.wUnder)}`,
    `vc_min=${f1(t.vcMin, 2)} v_in=${f1(t.vIn, 2)} sat=${t.sat}`,
    `lag=${f1(t.lag)}° yaw0=${f1(t.yaw0)}°`,
    `off0=${f1(t.off0)} off=${f1(t.off)} yaw=${f1(t.yaw)}° off_c=${f1(t.offC)} wide=${f1(t.wide)}`,
    `dsen40=${f1(t.dsen40, 0)}° ey40=${f1(t.ey40)}`,
  ].join(" | ");
}

function sensitivityTitle(kind: string): string {
  const sens = turnSensitivity(kind);
  if (!sens) return kind;
  return `${kind} の幾何感度: rad 1mm → 出口横ずれ ${sens.rad.toFixed(2)}mm、front 1mm → ${sens.front.toFixed(2)}mm(外側正。back は横に効かない)`;
}

// 列の説明(turn_exit_check.py の docstring と同じ意味)。見出し・値セルのどちらに
// マウスを載せても TipLayer で表示する(native の title は表示まで1秒待つ上に
// 小さいので使わない)。
const COLUMN_HELP: Record<string, string> = {
  idx: "旋回(SLALOM)が始まる CSV の行番号。PlotJuggler や --dump で同じ区間を追うときの目印",
  kind: "旋回の種類。角度と「斜め区間にいるか」で判定。\ndia45 / dia135 = 直線→斜め、dia45_2 / dia135_2 = 斜め→直線、dia90 = 斜め→斜め、large90 / orval180 = 直線→直線。\nホバーでその角度の幾何感度(rad / front 1mm あたりの出口横ずれ)も表示",
  dir: "L = 左旋回、R = 右旋回",
  v: "その旋回の設定速度 ideal_v [mm/s]。どの t_XXXX.yaml のプロファイルかに対応",
  wmax: "旋回中の目標角速度の最大 |ideal_w| [rad/s]。v ÷ wmax が実効半径 [mm]",
  latg: "横加速度 v × wmax を重力加速度で割った値 [G]。大きいほどタイヤに厳しく、飽和やスリップが出やすい",
  vcMin: "旋回中の実測速度 v_c の最小値 ÷ ideal_v。0.75 未満(赤)は接触かスリップで失速している",
  vIn: "旋回中の内輪速度の最小値 ÷ ideal_v。半径 38mm 級でも 0.3〜0.4 は残る。0.1 未満(赤)は内輪が止まった = 壁や柱に接触",
  sat: "旋回中に左右どちらかの duty が 99% を超えた tick 数。多い(黄: >20)ほどモーターが飽和して目標角速度に追従できていない",
  lag: "旋回終了 tick での 目標角度 ideal_ang − 実測角度 ang [deg]。+ は回り足りない、− は回り過ぎ。|lag| > 3° は黄",
  yaw0: "旋回後 2tick 目の kim_theta [deg] = 出口での向きのずれ(出口の向きを 0 とする座標系)。+ は左を向いている。|yaw0| > 3° は黄",
  off0: "旋回後、最初に左右の 45° 壁が両方見えた tick の (左45 − 右45) ÷ 2 [mm]。+ は右壁寄り。出口直後の生の横ずれで、ヨーの見かけ分を含む。斜めへ抜ける旋回は柱を見るので空欄",
  wide: "旋回後 25〜50mm 走った区間の横ずれから、ヨーの見かけ分(0.96mm/°)を引き、旋回の外側を + にした値 [mm]。+ は大回り、− は内側。これを ±3mm(黄の境)に入れるのが rad / front 調整の目標",
  dsen40: "旋回後 40tick の壁 PD 出力 |duty_sen| の最大 [deg]。大きい(黄: >30)ほど旋回直後に壁制御が暴れている。片壁×2 とヨーで誇張されるので目安",
  n: "集計した旋回の本数。4 本未満(薄字)はばらつきの判断には足りない",
};
const STAT_HELP = "\n(表示は 平均±σ、σ は母標準偏差)";

const TURN_EXIT_COLUMNS: Array<{ key: keyof typeof COLUMN_HELP; label: string }> = [
  { key: "idx", label: "idx" },
  { key: "kind", label: "種別" },
  { key: "dir", label: "向き" },
  { key: "v", label: "v" },
  { key: "wmax", label: "wmax" },
  { key: "latg", label: "G" },
  { key: "vcMin", label: "vc" },
  { key: "vIn", label: "v_in" },
  { key: "sat", label: "sat" },
  { key: "lag", label: "lag" },
  { key: "yaw0", label: "yaw0" },
  { key: "off0", label: "off0" },
  { key: "wide", label: "wide" },
  { key: "dsen40", label: "dsen" },
];
const SUMMARY_COLUMNS: Array<{ key: keyof typeof COLUMN_HELP; label: string; stat: boolean }> = [
  { key: "kind", label: "種別", stat: false },
  { key: "dir", label: "向き", stat: false },
  { key: "v", label: "v", stat: false },
  { key: "n", label: "n", stat: false },
  { key: "wide", label: "wide", stat: true },
  { key: "off0", label: "off0", stat: true },
  { key: "yaw0", label: "yaw0", stat: true },
  { key: "lag", label: "lag", stat: true },
  { key: "sat", label: "sat", stat: true },
  { key: "vcMin", label: "vc", stat: true },
  { key: "vIn", label: "v_in", stat: true },
  { key: "dsen40", label: "dsen", stat: true },
];

const CELL = "h-6 px-1 py-0 whitespace-nowrap";
const HEAD = `${CELL} cursor-help underline decoration-dotted underline-offset-2`;

interface TipState {
  x: number;
  y: number;
  text: string;
}

// 表の見出し/セル用の即時ツールチップ。ScrollArea の overflow に切られないよう
// position: fixed で、要素の直下に出す。
function useTip() {
  const [tip, setTip] = useState<TipState | null>(null);
  const show = useCallback((e: React.MouseEvent<HTMLElement>, text: string) => {
    const r = e.currentTarget.getBoundingClientRect();
    setTip({ x: r.left, y: r.bottom + 4, text });
  }, []);
  const hide = useCallback(() => setTip(null), []);
  return { tip, show, hide };
}

function TipLayer({ tip }: { tip: TipState | null }) {
  if (!tip) return null;
  const maxW = 380;
  const x = Math.min(tip.x, Math.max(8, window.innerWidth - maxW - 8));
  return (
    <div
      className="pointer-events-none fixed z-50 rounded border border-border bg-popover px-2 py-1 font-sans text-xs whitespace-pre-line text-popover-foreground shadow-md"
      style={{ left: x, top: tip.y, maxWidth: maxW }}
    >
      {tip.text}
    </div>
  );
}

type TipShow = (e: React.MouseEvent<HTMLElement>, text: string) => void;

function helpFor(key: string, label: string, stat = false): string {
  return `${label}: ${COLUMN_HELP[key] ?? ""}${stat ? STAT_HELP : ""}`;
}

function TurnExitTable({
  rows,
  selectedKey,
  onSelect,
  onTip,
  onTipHide,
}: {
  rows: TurnExitRow[];
  selectedKey: string | null;
  onSelect: (t: TurnExitRow) => void;
  onTip: TipShow;
  onTipHide: () => void;
}) {
  const cellValue = (t: TurnExitRow, key: string): { text: string; cls: string; tip?: string } => {
    switch (key) {
      case "idx":
        return { text: String(t.idx), cls: "" };
      case "kind":
        return { text: t.kind, cls: "", tip: `${helpFor("kind", "種別")}\n${sensitivityTitle(t.kind)}` };
      case "dir":
        return { text: t.dir, cls: "" };
      case "v":
        return { text: String(t.v), cls: "" };
      case "wmax":
        return { text: f1(t.wmax), cls: "" };
      case "latg":
        return { text: f1(t.latg), cls: "" };
      case "vcMin":
        return { text: f1(t.vcMin, 2), cls: t.vcMin < 0.75 ? "text-red-400" : "" };
      case "vIn":
        return { text: f1(t.vIn, 2), cls: t.vIn < 0.1 ? "bg-red-500/30 text-red-200" : "" };
      case "sat":
        return { text: String(t.sat), cls: t.sat > 20 ? "text-amber-400" : "" };
      case "lag":
        return { text: f1(t.lag), cls: Math.abs(t.lag) > 3 ? "text-amber-400" : "" };
      case "yaw0":
        return { text: f1(t.yaw0), cls: Math.abs(t.yaw0) > 3 ? "text-amber-400" : "" };
      case "off0":
        return { text: f1(t.off0), cls: "" };
      case "wide":
        return { text: f1(t.wide), cls: Math.abs(t.wide) > 3 ? "text-amber-400" : "" };
      case "dsen40":
        return { text: f1(t.dsen40, 0), cls: t.dsen40 > 30 ? "text-amber-400" : "" };
      default:
        return { text: "", cls: "" };
    }
  };
  return (
    <Table className="w-auto font-mono text-[11px]">
      <TableHeader>
        <TableRow>
          {TURN_EXIT_COLUMNS.map((c) => (
            <TableHead
              key={c.key}
              className={HEAD}
              onMouseEnter={(e) => onTip(e, helpFor(c.key, c.label))}
              onMouseLeave={onTipHide}
            >
              {c.label}
            </TableHead>
          ))}
        </TableRow>
      </TableHeader>
      <TableBody>
        {rows.map((t) => (
          <TableRow
            key={t.idx}
            className={`cursor-pointer ${turnKey(t) === selectedKey ? "bg-fuchsia-500/25 hover:bg-fuchsia-500/30" : ""}`}
            onClick={() => onSelect(t)}
          >
            {TURN_EXIT_COLUMNS.map((c) => {
              const v = cellValue(t, c.key);
              return (
                <TableCell
                  key={c.key}
                  className={`${CELL} ${v.cls}`}
                  onMouseEnter={(e) => onTip(e, v.tip ?? helpFor(c.key, c.label))}
                  onMouseLeave={onTipHide}
                >
                  {v.text}
                </TableCell>
              );
            })}
          </TableRow>
        ))}
      </TableBody>
    </Table>
  );
}

function TurnExitSummaryTable({
  data,
  onTip,
  onTipHide,
}: {
  data: TurnExitSummaryData;
  onTip: TipShow;
  onTipHide: () => void;
}) {
  const oldest = data.files[data.files.length - 1];
  const cellValue = (g: TurnExitSummaryRow, key: string): { text: string; cls: string; tip?: string } => {
    switch (key) {
      case "kind":
        return { text: g.kind, cls: "", tip: `${helpFor("kind", "種別")}\n${sensitivityTitle(g.kind)}` };
      case "dir":
        return { text: g.dir, cls: "" };
      case "v":
        return { text: String(g.v), cls: "" };
      case "n":
        return { text: String(g.n), cls: g.n < 4 ? "text-muted-foreground" : "" };
      case "wide":
        return { text: fmtStat(g.wide), cls: statFlag(g.wide, (m) => Math.abs(m) > 3) ? "text-amber-400" : "" };
      case "off0":
        return { text: fmtStat(g.off0), cls: "" };
      case "yaw0":
        return { text: fmtStat(g.yaw0), cls: statFlag(g.yaw0, (m) => Math.abs(m) > 3) ? "text-amber-400" : "" };
      case "lag":
        return { text: fmtStat(g.lag), cls: "" };
      case "sat":
        return { text: fmtStat(g.sat), cls: statFlag(g.sat, (m) => m > 20) ? "text-amber-400" : "" };
      case "vcMin":
        return { text: fmtStat(g.vcMin, 2), cls: statFlag(g.vcMin, (m) => m < 0.75) ? "text-red-400" : "" };
      case "vIn":
        return { text: fmtStat(g.vIn, 2), cls: statFlag(g.vIn, (m) => m < 0.1) ? "text-red-400" : "" };
      case "dsen40":
        return { text: fmtStat(g.dsen40, 0), cls: "" };
      default:
        return { text: "", cls: "" };
    }
  };
  return (
    <div className="flex flex-col gap-1">
      <span className="text-xs text-muted-foreground">
        集計 {data.files.length} 本 ({oldest} 〜 {data.files[0]}) 平均±σ、wide&gt;0 は外側/大回り
        {data.skipped.length > 0 && ` / 読めず: ${data.skipped.join(", ")}`}
      </span>
      <Table className="w-auto font-mono text-[11px]">
        <TableHeader>
          <TableRow>
            {SUMMARY_COLUMNS.map((c) => (
              <TableHead
                key={c.key}
                className={HEAD}
                onMouseEnter={(e) => onTip(e, helpFor(c.key, c.label, c.stat))}
                onMouseLeave={onTipHide}
              >
                {c.label}
              </TableHead>
            ))}
          </TableRow>
        </TableHeader>
        <TableBody>
          {data.summary.map((g) => (
            <TableRow key={`${g.kind}|${g.dir}|${g.v}`}>
              {SUMMARY_COLUMNS.map((c) => {
                const v = cellValue(g, c.key);
                return (
                  <TableCell
                    key={c.key}
                    className={`${CELL} ${v.cls}`}
                    onMouseEnter={(e) => onTip(e, v.tip ?? helpFor(c.key, c.label, c.stat))}
                    onMouseLeave={onTipHide}
                  >
                    {v.text}
                  </TableCell>
                );
              })}
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </div>
  );
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

  // 通常の横軸=index の時系列折れ線グラフ(空間プロットとは別物)。
  // troughColumns で選んだ列を表示し、センサートラフ解析が有効なら
  // trough/rise マーカーも重畳する。
  const [chartEnabled, setChartEnabled] = useState(false);

  // wall_off_edge_check.py 相当のオーバーレイ設定
  const [wallOffEnabled, setWallOffEnabled] = useState(false);
  const [wallOffMotionStates, setWallOffMotionStates] = useState("6,13");
  const [wallOffBaselineN, setWallOffBaselineN] = useState(5);
  const [wallOffArmDelta, setWallOffArmDelta] = useState(1.0);
  const [wallOffFitLo, setWallOffFitLo] = useState(1.0);
  const [wallOffFitHi, setWallOffFitHi] = useState(5.0);

  // turn_exit_check.py 相当: 旋回ごとの追従状態と出口残差(lib/turn-exit.ts)。
  // 選択中の1本はブラウザ側で解析、複数本の集計は /api/logs/turn-exit。
  const [turnExitEnabled, setTurnExitEnabled] = useState(false);
  const [turnExitLimit, setTurnExitLimit] = useState(6);
  const [turnExitSummary, setTurnExitSummary] = useState<TurnExitSummaryData | null>(null);
  const [turnExitBusy, setTurnExitBusy] = useState(false);
  // 旋回テーブルで選択中の行(プロット上で旋回区間と旋回後の窓を強調する)
  const [selectedTurnKey, setSelectedTurnKey] = useState<string | null>(null);
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
      xOffset,
    });
  }, [rawRows, dropEnabled, dropMotionState, dropLow, dropHigh, dropColLeft, dropColRight, pointByRow, xOffset]);

  const transitionEvents = useMemo<AnalysisEvent[]>(() => {
    if (!transitionEnabled || rawRows.length === 0) return [];
    const states = transitionStates
      .split(",")
      .map((s) => parseFloat(s.trim()))
      .filter((n) => !Number.isNaN(n));
    if (states.length === 0) return [];
    return computeMotionTransitionEvents(rawRows, { states, columns: TRANSITION_COLUMNS, pointByRow, xOffset });
  }, [rawRows, transitionEnabled, transitionStates, pointByRow, xOffset]);

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
      xOffset,
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
    xOffset,
  ]);

  const wallOffEvents = useMemo<AnalysisEvent[]>(() => {
    if (!wallOffEnabled || rawRows.length === 0) return [];
    const motionStates = wallOffMotionStates
      .split(",")
      .map((s) => parseFloat(s.trim()))
      .filter((n) => !Number.isNaN(n));
    if (motionStates.length === 0) return [];
    return computeWallOffEdgeEvents(rawRows, {
      motionStates,
      baselineN: wallOffBaselineN,
      armDelta: wallOffArmDelta,
      fitLo: wallOffFitLo,
      fitHi: wallOffFitHi,
      pointByRow,
      xOffset,
    });
  }, [
    rawRows,
    wallOffEnabled,
    wallOffMotionStates,
    wallOffBaselineN,
    wallOffArmDelta,
    wallOffFitLo,
    wallOffFitHi,
    pointByRow,
    xOffset,
  ]);

  const hfEvents = useMemo<AnalysisEvent[]>(() => {
    if (!showHf || rawRows.length === 0 || !("hf_edge_rel" in rawRows[0])) return [];
    return computeHfEdgeEvents(rawRows, { pointByRow, xOffset });
  }, [showHf, rawRows, pointByRow, xOffset]);

  const turnExitRows = useMemo<TurnExitRow[]>(() => {
    if (!turnExitEnabled || rawRows.length === 0) return [];
    return analyzeTurnExits(rawRows, { log: selected?.replace(/\.csv$/, "") ?? "" });
  }, [turnExitEnabled, rawRows, selected]);

  const turnExitEvents = useMemo<AnalysisEvent[]>(() => {
    const out: AnalysisEvent[] = [];
    for (const t of turnExitRows) {
      const r = rawRows[t.exitIdx];
      if (!r || !Number.isFinite(r.x) || !Number.isFinite(r.y)) continue;
      out.push({ x: r.x, y: r.y, anchored: "robot", kind: "turn-exit", label: turnExitLabel(t) });
    }
    return out;
  }, [turnExitRows, rawRows]);

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

  const analysisEvents = useMemo(
    () => [...dropEvents, ...transitionEvents, ...troughEvents, ...wallOffEvents, ...hfEvents, ...turnExitEvents],
    [dropEvents, transitionEvents, troughEvents, wallOffEvents, hfEvents, turnExitEvents]
  );

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
    if (!turnExitEnabled) return;
    // fetch-on-change: 結果は非同期に setState する(CLAUDE.md「既知のハマりどころ」参照)
    // eslint-disable-next-line react-hooks/set-state-in-effect
    void fetchTurnExitSummary(turnExitLimit);
  }, [turnExitEnabled, turnExitLimit, newestLog, fetchTurnExitSummary]);

  const chartSeries = useMemo<TimeSeries[]>(() => {
    if (!chartEnabled || rawRows.length === 0) return [];
    const toPoints = (col: string) =>
      rawRows
        .map((r) => ({ x: r.index, y: r[col] }))
        .filter((p) => Number.isFinite(p.x) && Number.isFinite(p.y) && p.y > 0);

    const series: TimeSeries[] = [];
    for (const col of TROUGH_COLUMNS) {
      if (!troughColumns[col]) continue;
      series.push({ column: col, color: COLUMN_COLOR[col], points: toPoints(col) });

      const companion = COMPANION_COLUMN[col];
      if (companion && companion in rawRows[0]) {
        series.push({ column: companion, color: COLUMN_COLOR[col], dash: true, points: toPoints(companion) });
      }
    }
    return series;
  }, [chartEnabled, rawRows, troughColumns]);

  const chartMarkers = useMemo(
    () =>
      [...(troughEnabled ? troughEvents : []), ...(wallOffEnabled ? wallOffEvents : []), ...hfEvents].filter(
        (e) => e.seriesIndex !== undefined
      ),
    [troughEnabled, troughEvents, wallOffEnabled, wallOffEvents, hfEvents]
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
        <div className="flex flex-wrap items-center gap-x-2 gap-y-0.5 rounded border border-border/60 px-1 py-0">
          <label className="flex items-center gap-1" title="analyze_sensor_drop.py 相当: 指定 motion_state 中のセンサー距離の急落(low/high しきい値)を検出">
            <input type="checkbox" checked={dropEnabled} onChange={(e) => setDropEnabled(e.target.checked)} />
            ドロップ
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
        <div className="flex flex-wrap items-center gap-x-2 gap-y-0.5 rounded border border-border/60 px-1 py-0">
          <label className="flex items-center gap-1" title="analyze_motion_state_transitions.py 相当: 指定 motion_state の開始/終了行と、その時の45°読みの壁面点">
            <input
              type="checkbox"
              checked={transitionEnabled}
              onChange={(e) => setTransitionEnabled(e.target.checked)}
            />
            状態遷移
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
        <div className="flex flex-wrap items-center gap-x-2 gap-y-0.5 rounded border border-border/60 px-1 py-0">
          <label className="flex items-center gap-1" title="analyze_sensor_trough.py 相当: 指定区間のセンサー距離の谷(最接近)と復帰">
            <input type="checkbox" checked={troughEnabled} onChange={(e) => setTroughEnabled(e.target.checked)} />
            トラフ
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
        <div className="flex flex-wrap items-center gap-x-2 gap-y-0.5 rounded border border-border/60 px-1 py-0">
          <label className="flex items-center gap-1" title="wall_off_edge_check.py 相当: 壁切れ検出のレベル判定/気づいた点/逆算エッジの3段を表示">
            <input type="checkbox" checked={wallOffEnabled} onChange={(e) => setWallOffEnabled(e.target.checked)} />
            壁切れエッジ
          </label>
          {wallOffEnabled && (
            <>
              <label className="flex items-center gap-1">
                motion_states
                <input
                  type="text"
                  className="w-16 rounded border border-border bg-background px-1"
                  value={wallOffMotionStates}
                  onChange={(e) => setWallOffMotionStates(e.target.value)}
                />
              </label>
              <label className="flex items-center gap-1">
                baseline N
                <input
                  type="number"
                  min={1}
                  className="w-14 rounded border border-border bg-background px-1"
                  value={wallOffBaselineN}
                  onChange={(e) => setWallOffBaselineN(parseInt(e.target.value, 10))}
                />
              </label>
              <label className="flex items-center gap-1">
                arm delta
                <input
                  type="number"
                  step={0.1}
                  className="w-16 rounded border border-border bg-background px-1"
                  value={wallOffArmDelta}
                  onChange={(e) => setWallOffArmDelta(parseFloat(e.target.value))}
                />
              </label>
              <label className="flex items-center gap-1">
                fit lo
                <input
                  type="number"
                  step={0.1}
                  className="w-16 rounded border border-border bg-background px-1"
                  value={wallOffFitLo}
                  onChange={(e) => setWallOffFitLo(parseFloat(e.target.value))}
                />
              </label>
              <label className="flex items-center gap-1">
                fit hi
                <input
                  type="number"
                  step={0.1}
                  className="w-16 rounded border border-border bg-background px-1"
                  value={wallOffFitHi}
                  onChange={(e) => setWallOffFitHi(parseFloat(e.target.value))}
                />
              </label>
            </>
          )}
        </div>
        <div className="flex flex-wrap items-center gap-x-2 gap-y-0.5 rounded border border-border/60 px-1 py-0">
          <label
            className="flex items-center gap-1"
            title="turn_exit_check.py 相当: 旋回(SLALOM)ごとの追従状態と出口残差。duty_sen は片壁×2とヨーの見かけ横ずれで誇張されるので、壁読み値から出した wide(外側正)と yaw0 で判定する"
          >
            <input type="checkbox" checked={turnExitEnabled} onChange={(e) => setTurnExitEnabled(e.target.checked)} />
            旋回出口
          </label>
          {turnExitEnabled && (
            <>
              <label
                className="flex items-center gap-1"
                title="直近 N 本のログをサーバー側で集計(latest.csv は除外)。新しいログが保存されると自動で更新"
              >
                集計 直近
                <input
                  type="number"
                  min={1}
                  max={40}
                  className="w-14 rounded border border-border bg-background px-1"
                  value={turnExitLimit}
                  onChange={(e) => setTurnExitLimit(Math.min(40, Math.max(1, parseInt(e.target.value, 10) || 1)))}
                />
                本
              </label>
              <Button
                size="sm"
                variant="ghost"
                disabled={turnExitBusy}
                onClick={() => void fetchTurnExitSummary(turnExitLimit)}
              >
                再集計
              </Button>
            </>
          )}
        </div>
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
                  highlight={turnExitEnabled ? turnHighlight : null}
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
          {turnExitEnabled && (
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
