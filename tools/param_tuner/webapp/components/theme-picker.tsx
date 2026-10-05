"use client";

import { useEffect, useRef, useState } from "react";
import { MachineChip } from "@/components/machine-chip";
import { Button } from "@/components/ui/button";
import type { Machine } from "@/lib/machine-shared";
import { DEFAULT_THEME_HEX, THEME_PRESETS, type ThemePrefs } from "@/lib/theme";

interface Props {
  prefs: ThemePrefs;
  onChange: (prefs: ThemePrefs) => void;
  // 表示中の機体(未登録なら null)
  machine: Machine | null;
  // 「機体ごとの色」のときに色を選んだ: その機体の色(machines.yaml の color)を変える
  onMachineColor: (id: string, color: string) => void;
}

// ヘッダーバーの「テーマ」。画面全体の主色(ボタン・バッジ・選択中のタブ・フォーカスの輪・
// 枠線の色味)を選ぶ。
//   機体ごとの色: 選んだ色は表示中の機体の色になる(機体設定の色と同じもの)。機体を
//                 切り替えると画面全体の色も変わるので、どの機体を見ているかが色で分かる。
//   全機体で 1 色: 選んだ色はこのブラウザに覚える。機体を切り替えても変わらない。
export function ThemePicker({ prefs, onChange, machine, onMachineColor }: Props) {
  const [open, setOpen] = useState(false);
  const rootRef = useRef<HTMLDivElement>(null);
  const perMachine = prefs.mode === "machine" && machine !== null;
  // いま当たっている色
  const current = (perMachine ? machine.color : (prefs.color ?? DEFAULT_THEME_HEX)).toLowerCase();

  useEffect(() => {
    if (!open) return;
    const onDown = (e: MouseEvent) => {
      if (rootRef.current && !rootRef.current.contains(e.target as Node)) setOpen(false);
    };
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") setOpen(false);
    };
    window.addEventListener("mousedown", onDown);
    window.addEventListener("keydown", onKey);
    return () => {
      window.removeEventListener("mousedown", onDown);
      window.removeEventListener("keydown", onKey);
    };
  }, [open]);

  // color = null は既定(GN グリーン)
  const pick = (color: string | null) => {
    if (perMachine) onMachineColor(machine.id, color ?? DEFAULT_THEME_HEX);
    else onChange({ mode: "fixed", color });
  };

  const isPreset = THEME_PRESETS.some((p) => (p.color ?? DEFAULT_THEME_HEX) === current);

  return (
    <div ref={rootRef} className="relative" data-theme-picker>
      <Button
        size="sm"
        variant={open ? "secondary" : "ghost"}
        className="gap-1.5 px-2"
        title="画面全体のテーマカラーを変える"
        onClick={() => setOpen((v) => !v)}
        data-theme-button
      >
        <span className="size-3 rounded-full bg-primary ring-1 ring-foreground/30" />
        テーマ
      </Button>
      {open && (
        <div
          className="cb-frame absolute top-full right-0 z-50 mt-1.5 flex w-80 flex-col gap-2 rounded-xl bg-popover p-3 text-sm shadow-xl ring-1 ring-foreground/20"
          data-theme-popover
        >
          <div className="flex items-center gap-1.5 font-medium">
            テーマカラー
            {perMachine && <MachineChip machine={machine} title="いま選ぶ色は、この機体の色になる" />}
          </div>
          <div className="flex flex-wrap gap-1.5">
            {THEME_PRESETS.map((p) => {
              const hex = p.color ?? DEFAULT_THEME_HEX;
              const selected = hex === current;
              return (
                <button
                  key={p.name}
                  type="button"
                  title={p.name}
                  data-theme-preset={p.color ?? "default"}
                  data-selected={selected}
                  onClick={() => pick(p.color)}
                  style={{ backgroundColor: hex }}
                  className={`size-7 rounded-md transition-transform hover:scale-110 ${
                    selected ? "ring-2 ring-foreground ring-offset-2 ring-offset-popover" : "ring-1 ring-foreground/20"
                  }`}
                />
              );
            })}
          </div>
          <label className="flex cursor-pointer items-center gap-2 text-xs text-muted-foreground">
            <input
              type="color"
              value={current}
              className={`h-7 w-9 shrink-0 cursor-pointer rounded border-0 bg-transparent p-0 ${
                isPreset ? "" : "ring-2 ring-foreground ring-offset-2 ring-offset-popover"
              }`}
              onChange={(e) => pick(e.target.value)}
              data-theme-custom
            />
            好きな色を選ぶ(ごく暗い色は、同じ色合いのまま見える明るさにします)
          </label>
          <div className="h-px bg-border" />
          <label className="flex cursor-pointer items-start gap-2 text-xs">
            <input
              type="checkbox"
              className="mt-0.5"
              checked={prefs.mode === "machine"}
              onChange={(e) =>
                onChange(
                  e.target.checked
                    ? { ...prefs, mode: "machine" }
                    : // いまの見た目のまま「全機体で 1 色」へ移る
                      { mode: "fixed", color: current === DEFAULT_THEME_HEX ? null : current },
                )
              }
              data-theme-per-machine
            />
            <span>
              機体ごとに色を変える
              <span className="block text-muted-foreground">
                {prefs.mode === "machine"
                  ? "機体を切り替えると画面全体の色も変わる。色は機体ごとに machines.yaml に残る(機体設定の色と同じもの)"
                  : "外してあるので、どの機体でも同じ色(このブラウザに覚える)"}
              </span>
            </span>
          </label>
        </div>
      )}
    </div>
  );
}
