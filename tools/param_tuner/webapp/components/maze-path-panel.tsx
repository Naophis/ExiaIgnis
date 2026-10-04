"use client";

import { useState } from "react";
import { Button } from "@/components/ui/button";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Table, TableBody, TableCell, TableHead, TableHeader, TableRow } from "@/components/ui/table";
import type { PathGeometry } from "@/lib/maze-path";
import type { ExecOption, PathDirection, PathSimResult, TurnParam, TurnParamSet } from "@/lib/path-sim";

// 迷路タブの右に出す「経路」パネル。計算は usePathSim、軌跡の重ね描きは MazePanel。

interface Props {
  options: ExecOption[];
  exec: number;
  onExecChange: (v: number) => void;
  direction: PathDirection;
  onDirectionChange: (v: PathDirection) => void;
  busy: boolean;
  result: PathSimResult | null;
  geometry: PathGeometry | null; // 最終経路の軌跡(ターン名の表示に使う)
  shownCandidate: number | null;
  onShowCandidate: (type: number | null) => void;
  hoverSeg: number | null;
  onHoverSeg: (i: number | null) => void;
}

// 番号は run_prf.yaml の exec_prof の並び順(0 始まり、ユーザー指定)。機体のメインモードの
// mode_num はこれ + 2(0 = 探索, 1 = 片側/帰還探索)。
function execDetail(o: ExecOption): string {
  const speed = [o.turnFile, o.vMax !== null ? `直線${o.vMax}` : null].filter(Boolean).join(" ");
  return `f${o.fast} n${o.normal} s${o.slow}${speed ? `  ${speed}` : ""}`;
}

// 機体のモード選択の LED(MainTask::select_mode: lbit.byte = mode_num + 1)と同じ点灯パターン。
// 左から b5 b4 b3 / b2 b1 b0(UserInterface::LED_bit の配線で、左の 3 個が b5 b4 b3 の順)。
// mode 0 = ○○○ ○○●、このシミュレータの 0 番(mode 2)= ○○○ ○●●。点灯色は実機と同じライトグリーン。
function LedPattern({ value }: { value: number }) {
  const bits = [5, 4, 3, 2, 1, 0].map((b) => ((value >> b) & 1) === 1);
  return (
    <span className="flex items-center gap-[2px]">
      {bits.map((on, i) => (
        <span
          key={i}
          className={`inline-block size-[7px] rounded-full ${
            on ? "bg-[oklch(0.78_0.17_142)] shadow-[0_0_3px_oklch(0.78_0.17_142/0.7)]" : "bg-muted-foreground/20"
          } ${i === 3 ? "ml-[5px]" : ""}`}
        />
      ))}
    </span>
  );
}

const ledText = (value: number) =>
  [5, 4, 3, 2, 1, 0].map((b, i) => `${i === 3 ? " " : ""}${(value >> b) & 1 ? "●" : "○"}`).join("");

const SETS: { key: TurnParamSet; label: string; title: string }[] = [
  { key: "fast", label: "f", title: "fast: このターンの後に直線があり、その次のターンが Large / Orval のとき(calc_goal_time の fast_turn_mode)" },
  { key: "normal", label: "n", title: "normal: 通常" },
  { key: "slow", label: "s", title: "slow: 走り出しの直線が 0 で、いきなり曲がるとき(start_turn)" },
];

const num = (v: number | undefined, digits = 0) => (v === undefined ? "-" : v.toFixed(digits));
// 左右が同じなら 1 つだけ出す(表の幅を抑える)。
const lr = (l: number | undefined, r: number | undefined) =>
  l === undefined || r === undefined ? "-" : l === r ? l.toFixed(1) : `${l.toFixed(1)}/${r.toFixed(1)}`;

function TurnParamTable({ result, geometry }: { result: PathSimResult; geometry: PathGeometry | null }) {
  const count = new Map<string, number>();
  for (const t of geometry?.turns ?? []) count.set(t.name.toLowerCase(), (count.get(t.name.toLowerCase()) ?? 0) + 1);
  const types = (result.turn_params?.normal ?? []).map((t) => t.type);
  const byType = (set: TurnParamSet, type: string): TurnParam | undefined =>
    result.turn_params?.[set]?.find((t) => t.type === type);
  const sp = result.straight_params ?? {};
  return (
    <>
      <div className="px-1.5 pb-1 font-mono text-[11px] text-muted-foreground" title="vel_prof の normal の番号の直線パラメータ(v_max / accl / decel)">
        直線 {Object.entries(sp)
          .map(([k, v]) => `${k} ${v.v_max.toFixed(0)} (${v.accl.toFixed(0)}/${v.decel.toFixed(0)})`)
          .join(" · ")}
      </div>
      <Table className="font-mono text-[11px]">
        <TableHeader>
          <TableRow>
            <TableHead className="px-1" title="×N は経路での回数">ターン</TableHead>
            <TableHead className="px-1" title="組(f = fast / n = normal / s = slow)と、profiles.yaml の list のファイル(t_ を省略)">
              組 file
            </TableHead>
            <TableHead className="px-1">v</TableHead>
            <TableHead className="px-1">rad</TableHead>
            <TableHead className="px-1" title="pow_n">n</TableHead>
            <TableHead className="px-1" title="旋回時間 s(読込時に rad・ang・v・pow_n から計算。Orval の time2 はマウスを載せると出る)">
              time
            </TableHead>
            <TableHead className="px-1" title="front 左/右(同じなら 1 つ)">front</TableHead>
            <TableHead className="px-1" title="back 左/右(同じなら 1 つ)">back</TableHead>
          </TableRow>
        </TableHeader>
        <TableBody>
          {types.map((type) => {
            const n = count.get(type) ?? 0;
            return SETS.map((set, k) => {
              const p = byType(set.key, type);
              return (
                <TableRow key={`${type}-${set.key}`} className={`${n === 0 ? "opacity-45" : ""} ${k === 0 ? "border-t border-border" : "border-0"}`}>
                  <TableCell className="px-1">{k === 0 ? `${type}${n > 0 ? ` ×${n}` : ""}` : ""}</TableCell>
                  <TableCell className="px-1" title={`${set.title}\n${p?.file ?? ""}`}>
                    {set.label} {p?.file?.replace(/^t_/, "").replace(/\.hf$/, "") ?? "-"}
                  </TableCell>
                  <TableCell className="px-1">{num(p?.v)}</TableCell>
                  <TableCell className="px-1">{num(p?.rad, 1)}</TableCell>
                  <TableCell className="px-1">{num(p?.pow_n)}</TableCell>
                  <TableCell className="px-1" title={p?.time2 ? `time2 = ${p.time2.toFixed(3)}` : undefined}>
                    {num(p?.time, 3)}
                  </TableCell>
                  <TableCell className="px-1">{lr(p?.front_l, p?.front_r)}</TableCell>
                  <TableCell className="px-1">{lr(p?.back_l, p?.back_r)}</TableCell>
                </TableRow>
              );
            });
          })}
        </TableBody>
      </Table>
    </>
  );
}

export function MazePathPanel({
  options,
  exec,
  onExecChange,
  direction,
  onDirectionChange,
  busy,
  result,
  geometry,
  shownCandidate,
  onShowCandidate,
  hoverSeg,
  onHoverSeg,
}: Props) {
  const [view, setView] = useState<"segments" | "turns">("segments");
  const ok = result?.ok === true;
  const segs = result?.segments ?? [];
  const turnAt = new Map((geometry?.turns ?? []).map((t) => [t.index, t]));

  return (
    <div className="flex h-full min-w-0 flex-col overflow-hidden border-l border-border text-xs">
      {/* 走行パラメータは 1 クリックで切り替える(プルダウンは操作が面倒と言われた)。 */}
      {/* 同じ幅の格子に並べて、点灯パターンが縦にそろうようにする。選択中も LED の色は変えない。 */}
      <div className="grid grid-cols-[repeat(auto-fill,minmax(4.75rem,1fr))] gap-1 px-1.5 pt-1">
        {options.map((o) => {
          const selected = o.index === exec;
          return (
            <button
              key={o.index}
              type="button"
              onClick={() => onExecChange(o.index)}
              title={`run_prf[${o.index}]: ${execDetail(o)}\n機体の mode ${o.index + 2}  LED ${ledText(o.index + 3)}`}
              className={`flex h-6 items-center justify-between gap-1 rounded border px-1.5 font-mono text-[11px] ${
                selected ? "border-primary-bright bg-primary-bright/15 ring-1 ring-primary-bright" : "border-border hover:bg-muted"
              }`}
            >
              <LedPattern value={o.index + 3} />
              <span className={selected ? "font-bold text-primary-bright" : "text-muted-foreground"}>{o.index}</span>
            </button>
          );
        })}
      </div>
      <div className="flex flex-wrap items-center gap-1 px-1.5 py-0.5">
        <div className="flex gap-0.5">
          <Button
            size="xs"
            variant={direction === "right" ? "default" : "outline"}
            onClick={() => onDirectionChange("right")}
            title="機体で右を選んだとき: タイム最小の経路探索(calc_goal_time と同じ計算を重みにした最短経路)。使えないときだけ、重みパターン 1〜5 の候補の比較へ戻る"
          >
            右: タイム最小
          </Button>
          <Button
            size="xs"
            variant={direction === "left" ? "default" : "outline"}
            onClick={() => onDirectionChange("left")}
            title="機体で左を選んだとき: path_create の経路をそのまま使う"
          >
            左: 単純
          </Button>
        </div>
        {options[exec] && (
          <span className="font-mono text-muted-foreground" title={`機体の mode ${exec + 2}  LED ${ledText(exec + 3)}`}>
            run_prf[{exec}] {execDetail(options[exec])}
          </span>
        )}
        {busy && <span className="text-muted-foreground">計算中…</span>}
      </div>

      {result && !ok && (
        <div className="px-1.5 py-1 whitespace-pre-wrap text-destructive">
          経路を作れません: {result.error ?? "不明なエラー"}
        </div>
      )}

      {ok && (
        <div className="flex flex-col gap-0.5 px-1.5 pb-1">
          <div className="flex flex-wrap items-baseline gap-x-2">
            <span className="text-sm font-semibold">{result.goal_time?.toFixed(3)} s</span>
            <span
              className="text-muted-foreground"
              title="PathCreator::calc_goal_time() の値(実機が走行前に出す仮想タイムと同じ計算)。最後の直線(最後のターン〜ゴール)を含む"
            >
              ゴールタイム(見積もり)
            </span>
            <span className="text-muted-foreground">
              吸引 {result.suction} · ターン {geometry?.turns.length ?? 0}
            </span>
            {direction === "right" && result.planner && result.method_used === "time" && (
              <span
                className="text-muted-foreground"
                title={`タイム最小の経路探索(TimePathPlanner)。計算は PC 上の時間\n節点 ${result.planner.nodes} / 辺 ${result.planner.edges} / ヒープ最大 ${result.planner.heap_max} / 覚えた区間 ${result.planner.seg_cached}\n機体の節点の上限は空きメモリから 1024〜8192`}
              >
                探索 {result.planner.ms.toFixed(1)} ms · 節点 {result.planner.nodes}
              </span>
            )}
            {direction === "right" && result.planner && result.method_used !== "time" && (
              <span className="text-accent-gold" title="機体も同じ条件なら、従来の重みパターン 1〜5 の比較で経路を作る">
                タイム最小の探索が使えず({result.planner.result})→ 従来の方法
              </span>
            )}
          </div>
          {direction === "right" && (result.candidates?.length ?? 0) > 0 && (
            <div className="flex flex-wrap items-center gap-1">
              <span
                className="text-muted-foreground"
                title="従来の方法で作った候補: set_param_num の番号と、その候補の calc_goal_time。クリックで迷路に重ねる"
              >
                候補
              </span>
              {result.candidates!.map((c) => {
                const selected = c.type === result.selected_type;
                const shown = shownCandidate === c.type;
                return (
                  <button
                    key={c.type}
                    type="button"
                    disabled={!c.result}
                    onClick={() => onShowCandidate(shown ? null : c.type)}
                    className={`rounded border px-1 font-mono ${
                      shown ? "border-accent-gold text-accent-gold" : "border-border"
                    } ${selected ? "font-bold" : ""} ${c.result ? "hover:bg-muted" : "opacity-50"}`}
                    title={selected ? "採用された候補" : c.result ? "クリックで迷路に重ねる" : "この候補は経路を作れなかった"}
                  >
                    {selected ? "✓" : ""}#{c.type} {c.result ? c.time.toFixed(3) : "失敗"}
                  </button>
                );
              })}
              {result.selected_type === -1 && (
                <span className="text-accent-gold">候補がすべて失敗 → 単純な経路</span>
              )}
            </div>
          )}
        </div>
      )}

      {ok && (
        <div className="flex gap-0.5 px-1.5 pb-0.5">
          <Button size="xs" variant={view === "segments" ? "default" : "outline"} onClick={() => setView("segments")}>
            区間
          </Button>
          <Button
            size="xs"
            variant={view === "turns" ? "default" : "outline"}
            onClick={() => setView("turns")}
            title="load_slalom_param() が読んだターンごとのパラメータ(fast / normal / slow)"
          >
            ターン設定
          </Button>
        </div>
      )}

      {ok && view === "turns" && (
        <ScrollArea className="min-h-0 flex-1">
          <TurnParamTable result={result} geometry={geometry} />
        </ScrollArea>
      )}

      {ok && view === "segments" && (
        <ScrollArea className="min-h-0 flex-1">
          <Table className="font-mono text-[11px]">
            <TableHeader>
              <TableRow>
                <TableHead title="path_s / path_t の添字">#</TableHead>
                <TableHead title="0.5 × path_s − 1(print_path と同じ)/ 実際に走る距離 mm">直線</TableHead>
                <TableHead>ターン</TableHead>
                <TableHead title="直線の時間 s">t直</TableHead>
                <TableHead title="ターンの時間 s(slalom_dummy)">tタ</TableHead>
                <TableHead title="累計 s">累計</TableHead>
                <TableHead title="直線の最高 / 終わりの速度 mm/s。終わり = 次のターンに入る速度(始めは前の区間の終わりと同じ)">v</TableHead>
              </TableRow>
            </TableHeader>
            <TableBody>
              {segs.map((sg, i) => {
                const s = result.path_s?.[i] ?? 0;
                const t = result.path_t?.[i] ?? 0;
                const turn = turnAt.get(i);
                return (
                  <TableRow
                    key={i}
                    className={hoverSeg === i ? "bg-muted" : ""}
                    onMouseEnter={() => onHoverSeg(i)}
                    onMouseLeave={() => onHoverSeg(null)}
                  >
                    <TableCell>{i}</TableCell>
                    <TableCell>
                      {(0.5 * s - 1).toFixed(1)} / {sg.dist.toFixed(0)}
                    </TableCell>
                    <TableCell>{t === 255 ? "Finish" : turn ? `${turn.name} ${turn.right ? "R" : "L"}` : t}</TableCell>
                    <TableCell>{sg.str_time.toFixed(3)}</TableCell>
                    <TableCell>{sg.turn_time.toFixed(3)}</TableCell>
                    <TableCell>{sg.total_time.toFixed(3)}</TableCell>
                    <TableCell>
                      {sg.v_max.toFixed(0)}/{sg.v_end.toFixed(0)}
                    </TableCell>
                  </TableRow>
                );
              })}
            </TableBody>
          </Table>
          <details className="px-1.5 py-1">
            <summary className="cursor-pointer text-muted-foreground">ファームの出力(実機のコンソールと同じ)</summary>
            <pre className="mt-1 text-[10px] leading-tight whitespace-pre-wrap text-muted-foreground">{result.log}</pre>
          </details>
        </ScrollArea>
      )}

      {result && !ok && result.log && (
        <ScrollArea className="min-h-0 flex-1">
          <pre className="px-1.5 text-[10px] leading-tight whitespace-pre-wrap text-muted-foreground">{result.log}</pre>
        </ScrollArea>
      )}
    </div>
  );
}
