"use client";

// 旋回出口解析(lib/turn-exit.ts)の表示。プロットタブ(log-plot-panel.tsx)と
// 詳細ログ解析ページ(log-detail-view.tsx)の両方から使うのでここに置く。
// 列の意味は COLUMN_HELP に日本語で持ち、見出し・値セルのどちらにホバーしても
// TipLayer で出す(native の title は表示まで1秒待つ上に ScrollArea に切られる)。

import { useCallback, useState } from "react";
import { Table, TableBody, TableCell, TableHead, TableHeader, TableRow } from "@/components/ui/table";
import {
  fmtNum as f1,
  turnKey,
  turnSensitivity,
  type Stat,
  type TurnExitRow,
  type TurnExitSummaryRow,
} from "@/lib/turn-exit";

export interface TurnExitSummaryData {
  files: string[];
  skipped: string[];
  rows: TurnExitRow[];
  summary: TurnExitSummaryRow[];
}

// API 経由の Stat は NaN が JSON で null になる(n=0)。表示・色付けとも n===0 を先に見る
const fmtStat = (st: Stat, digits = 1) => (st.n === 0 ? "–" : `${st.mean.toFixed(digits)}±${st.std.toFixed(digits)}`);
const statFlag = (st: Stat, cond: (mean: number) => boolean) => st.n > 0 && cond(st.mean);

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
export function useTip() {
  const [tip, setTip] = useState<TipState | null>(null);
  const show = useCallback((e: React.MouseEvent<HTMLElement>, text: string) => {
    const r = e.currentTarget.getBoundingClientRect();
    setTip({ x: r.left, y: r.bottom + 4, text });
  }, []);
  const hide = useCallback(() => setTip(null), []);
  return { tip, show, hide };
}

export function TipLayer({ tip }: { tip: TipState | null }) {
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

export function TurnExitTable({
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

export function TurnExitSummaryTable({
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
