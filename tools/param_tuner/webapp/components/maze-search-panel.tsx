"use client";

import { useEffect, useMemo, useRef } from "react";
import { Button } from "@/components/ui/button";
import { ScrollArea } from "@/components/ui/scroll-area";
import type { SearchSimResult, SearchStep } from "@/lib/search-sim";

// 迷路タブの右に出す「探索」パネル。計算は useSearchSim、地図・軌跡・ロボットの重ね描きは
// MazePanel。ステップ = 足立法の判断 1 回。

export const MOTION_LABEL: Record<SearchStep["m"], string> = {
  S: "探索直進",
  F: "既知の直進",
  D: "既知→ターン",
  R: "右ターン",
  L: "左ターン",
  B: "後退(超信地)",
};
const MOTION_TITLE: Record<SearchStep["m"], string> = {
  S: "未踏の区画へ 1 区画(探索速度)",
  F: "踏破済みで次も直進: FastRun の速度で 1 区画",
  D: "踏破済みで次はターン: FastRunDia の加速で、終わりは探索速度",
  R: "スラローム(Normal)。judge2() の pivot90 は選ばない",
  L: "スラローム(Normal)。judge2() の pivot90 は選ばない",
  B: "pivot(): 中央へ → 前壁合わせ → 超信地 → 後退 → 半区画",
};
const END_LABEL: Record<string, string> = {
  home: "スタートへ帰還",
  timeup: "時間切れ(seach_timer)",
  back4: "後退が 4 回続いて中止",
  none: "足立法が NONE を返して終了",
  step_limit: "判断回数の上限で打ち切り",
};
// Direction の値 → 矢印
export const DIR_ARROW: Record<number, string> = { 1: "↑", 2: "→", 4: "←", 8: "↓" };
const SPEEDS = [2, 5, 20, 60];

export function fmtTime(s: number): string {
  const m = Math.floor(s / 60);
  const r = s - m * 60;
  return m > 0 ? `${s.toFixed(1)} s (${m}:${r.toFixed(1).padStart(4, "0")})` : `${s.toFixed(2)} s`;
}

interface Props {
  result: SearchSimResult | null;
  busy: boolean;
  step: number;
  onStep: (i: number) => void;
  playing: boolean;
  onTogglePlay: () => void;
  speed: number;
  onSpeed: (v: number) => void;
  knownCells: number; // 現在のステップで 4 方向とも分かっている区画の数
  totalCells: number;
  showRoute: boolean; // 候補の経路とサブゴールを迷路に重ねるか
  onShowRoute: (v: boolean) => void;
  routeLen: number; // 現在のステップの候補の経路の長さ(区画。0 = 候補なし)
  routeUnknown: number; // そのうち、通る壁が未知の区間の数
}

export function MazeSearchPanel({
  result,
  busy,
  step,
  onStep,
  playing,
  onTogglePlay,
  speed,
  onSpeed,
  knownCells,
  totalCells,
  showRoute,
  onShowRoute,
  routeLen,
  routeUnknown,
}: Props) {
  const ok = result?.ok === true;
  const steps = useMemo(() => (ok ? (result.steps ?? []) : []), [ok, result]);
  const last = steps.length - 1;
  const cur = steps[step];

  const breakdown = useMemo(() => {
    const m = new Map<SearchStep["m"], { n: number; t: number }>();
    for (const st of steps) {
      const e = m.get(st.m) ?? { n: 0, t: 0 };
      e.n += 1;
      e.t += st.t1 - st.t0;
      m.set(st.m, e);
    }
    return (Object.keys(MOTION_LABEL) as SearchStep["m"][]).filter((k) => m.has(k)).map((k) => ({ k, ...m.get(k)! }));
  }, [steps]);

  // 一覧の行は結果が変わったときだけ作る(再生中に 1000 行を描き直さない)。
  // 現在行の強調とスクロールは下の effect で直接切り替える。
  const rows = useMemo(
    () =>
      steps.map((st, i) => (
        <tr
          key={i}
          data-i={i}
          onClick={() => onStep(i)}
          className="cursor-pointer border-b border-border/40 hover:bg-muted/60"
        >
          <td className="px-1 text-right text-muted-foreground">{i}</td>
          <td className="px-1 text-right">{st.t0.toFixed(2)}</td>
          <td className="px-1">
            ({st.f[0]},{st.f[1]}){DIR_ARROW[st.f[2]]}
          </td>
          <td className="px-1" title={MOTION_TITLE[st.m]}>
            {MOTION_LABEL[st.m]}
          </td>
          <td className="px-1 text-right">{((st.t1 - st.t0) * 1000).toFixed(0)}</td>
          <td className="px-1 text-muted-foreground">{st.g ? `G後 ${st.sg}` : ""}</td>
        </tr>
      )),
    // onStep は親の setState なので変わらない
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [steps],
  );
  const tbodyRef = useRef<HTMLTableSectionElement>(null);
  const prevRow = useRef<Element | null>(null);
  useEffect(() => {
    prevRow.current?.classList.remove("bg-primary/20");
    const row = tbodyRef.current?.querySelector(`[data-i="${step}"]`) ?? null;
    row?.classList.add("bg-primary/20");
    row?.scrollIntoView({ block: "nearest" });
    prevRow.current = row;
  }, [step, rows]);

  return (
    <div className="flex h-full min-w-0 flex-col overflow-hidden border-l border-border text-xs">
      <div className="flex flex-wrap items-center gap-1 px-1.5 pt-1">
        <Button size="xs" variant="outline" onClick={() => onStep(0)} disabled={!ok} title="最初へ (Home)">
          ⏮
        </Button>
        <Button size="xs" variant="outline" onClick={() => onStep(Math.max(0, step - 1))} disabled={!ok} title="1 つ戻る (←、Shift で 10)">
          ◀
        </Button>
        <Button size="xs" variant={playing ? "default" : "outline"} onClick={onTogglePlay} disabled={!ok} title="再生 / 停止">
          {playing ? "⏸" : "▶"}
        </Button>
        <Button size="xs" variant="outline" onClick={() => onStep(Math.min(last, step + 1))} disabled={!ok} title="1 つ進む (→、Shift で 10)">
          ▶|
        </Button>
        <Button size="xs" variant="outline" onClick={() => onStep(last)} disabled={!ok} title="最後へ (End)">
          ⏭
        </Button>
        <div className="flex gap-0.5" title="再生の速さ(判断 / 秒)">
          {SPEEDS.map((s) => (
            <button
              key={s}
              type="button"
              onClick={() => onSpeed(s)}
              className={`rounded border px-1 font-mono text-[10px] ${speed === s ? "border-primary text-primary" : "border-border text-muted-foreground hover:bg-muted"}`}
            >
              {s}/s
            </button>
          ))}
        </div>
        <button
          type="button"
          onClick={() => onShowRoute(!showRoute)}
          title={
            "足立法がいま見ている候補の経路を重ねる(ゴール後)。未知の壁は無いものとした重みパターン 1 の最短経路で、この上の未知区画がサブゴールになる\n" +
            "実線 = 既知の区間 / 点線 = 通る壁が未知の区間 / ◆ = サブゴール(前の経路から持ち越したものも含む)"
          }
          className={`rounded border px-1 text-[10px] ${showRoute ? "border-[oklch(0.74_0.19_330)] text-[oklch(0.74_0.19_330)]" : "border-border text-muted-foreground hover:bg-muted"}`}
        >
          候補
        </button>
        {busy && <span className="text-muted-foreground">計算中…</span>}
      </div>
      {ok && steps.length > 0 && (
        <input
          type="range"
          min={0}
          max={last}
          value={step}
          onChange={(e) => onStep(Number(e.target.value))}
          className="mx-1.5 my-1 accent-[var(--primary)]"
        />
      )}

      {result && !ok && (
        <div className="px-1.5 py-1 whitespace-pre-wrap text-destructive">探索を再現できません: {result.error ?? "不明なエラー"}</div>
      )}

      {ok && cur && (
        <div className="px-1.5 pb-1 font-mono">
          <span className="text-sm font-semibold">
            #{step} / {last}
          </span>{" "}
          t={cur.t0.toFixed(2)} s ({cur.f[0]},{cur.f[1]}){DIR_ARROW[cur.f[2]]} → {MOTION_LABEL[cur.m]} → ({cur.to[0]},{cur.to[1]})
          {DIR_ARROW[cur.to[2]]}
          <span className="text-muted-foreground">
            {" "}
            · 既知 {knownCells}/{totalCells} 区画{cur.g ? ` · ゴール後(残りサブゴール ${cur.sg})` : ""}
            {cur.g && routeLen > 0 ? ` · 候補 ${routeLen}(未知 ${routeUnknown})` : ""}
          </span>
        </div>
      )}

      {ok && (
        <div className="flex flex-col gap-0.5 border-t border-border/60 px-1.5 py-1">
          <div className="flex flex-wrap items-baseline gap-x-2">
            <span className="text-sm font-semibold">{fmtTime(result.total_time ?? 0)}</span>
            <span
              className="text-muted-foreground"
              title="理想のセンサーで、ターンは常にスラローム・壁切れ補正なし・前壁合わせは即収束として、各モーションの時間を足したもの。実機の補正動作やセンサーの読み違いは入らない"
            >
              探索時間(試算)
            </span>
            <span className={result.end_reason === "home" ? "text-muted-foreground" : "text-accent-gold"}>
              {END_LABEL[result.end_reason ?? ""] ?? result.end_reason}
            </span>
          </div>
          <div className="text-muted-foreground">
            ゴール到達 {result.goal_time !== undefined && result.goal_time >= 0 ? fmtTime(result.goal_time) : "なし"}
            {result.goal_by === "known" && (
              <span title="残り 1 区画のゴールは 4 辺が分かった時点で入らずに到達扱い(Adachi::goal_step_check)">(入らずに確定)</span>
            )}{" "}
            · 判断{" "}
            {steps.length} 回 · 上限 {result.search_timer} s
          </div>
          {result.params && (
            <div className="font-mono text-[11px] text-muted-foreground" title="run_main_mode() の mode_num == 0 と同じく load_slalom_param(0, 0, 0)">
              探索 v{result.params.search_v} a{result.params.search_accl} · 既知 v{result.params.fast_v} · ターン{" "}
              {result.params.turn_file?.replace(/\.hf$/, "")} v{result.params.turn_v} ({(result.params.turn_time * 1000).toFixed(0)}ms) · 超信地 w
              {result.params.w_max} α{result.params.alpha}
            </div>
          )}
          <table className="mt-0.5 w-auto font-mono text-[11px]">
            <tbody>
              {breakdown.map((b) => (
                <tr key={b.k} title={MOTION_TITLE[b.k]}>
                  <td className="pr-2">{MOTION_LABEL[b.k]}</td>
                  <td className="pr-2 text-right">{b.n} 回</td>
                  <td className="pr-2 text-right">{b.t.toFixed(1)} s</td>
                  <td className="pr-2 text-right text-muted-foreground">{((b.t / b.n) * 1000).toFixed(0)} ms/回</td>
                  <td className="text-right text-muted-foreground">{((b.t / (result.total_time || 1)) * 100).toFixed(0)}%</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}

      {ok && (
        <ScrollArea className="min-h-0 flex-1 border-t border-border/60">
          <table className="w-full font-mono text-[11px]">
            <thead className="sticky top-0 bg-card text-muted-foreground">
              <tr>
                <th className="px-1 text-right font-normal">#</th>
                <th className="px-1 text-right font-normal" title="判断した時刻 s">t</th>
                <th className="px-1 text-left font-normal" title="判断した区画と向き">区画</th>
                <th className="px-1 text-left font-normal">動作</th>
                <th className="px-1 text-right font-normal" title="この動作にかかった時間 ms">ms</th>
                <th className="px-1 text-left font-normal" title="ゴール到達後の残りサブゴール数">ゴール後</th>
              </tr>
            </thead>
            <tbody ref={tbodyRef}>{rows}</tbody>
          </table>
        </ScrollArea>
      )}
    </div>
  );
}
