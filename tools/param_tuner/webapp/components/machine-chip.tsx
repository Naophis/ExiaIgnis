"use client";

import type { CSSProperties, ReactNode } from "react";
import { contrastText, type Machine } from "@/lib/machine-shared";
import { cn } from "@/lib/utils";

// 機体の名前を、その機体の色で出す。どの機体のファイルを見ている・書いている・
// 送っているかを、画面のどこでも同じ色で示すための部品。
export function MachineChip({
  machine,
  fallback,
  solid = true,
  className,
  children,
  title,
}: {
  machine: Machine | null;
  fallback?: string; // 登録簿に無い id のとき出す文字
  solid?: boolean; // false = 枠だけ
  className?: string;
  children?: ReactNode; // 名前の後ろに足すもの
  title?: string;
}) {
  const color = machine?.color ?? "#64748b";
  const style: CSSProperties = solid
    ? { backgroundColor: color, color: contrastText(color), borderColor: color }
    : { borderColor: color, color };
  return (
    <span
      title={title}
      style={style}
      className={cn(
        "inline-flex h-5 shrink-0 items-center gap-1 rounded-md border px-1.5 text-xs font-semibold whitespace-nowrap",
        className,
      )}
    >
      {machine?.label ?? fallback ?? "?"}
      {children}
    </span>
  );
}

export function MachineDot({ machine, className }: { machine: Machine | null; className?: string }) {
  return (
    <span
      className={cn("inline-block size-2 shrink-0 rounded-full", className)}
      style={{ backgroundColor: machine?.color ?? "#64748b" }}
    />
  );
}
