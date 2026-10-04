"use client";

import { useEffect, type ReactNode } from "react";
import { Button } from "@/components/ui/button";

export interface Choice {
  key: string;
  label: string;
  variant?: "default" | "outline" | "secondary" | "destructive" | "ghost";
}

export interface ChoiceRequest {
  title: string;
  body: ReactNode;
  choices: Choice[];
  resolve: (key: string | null) => void; // null = やめる(Esc・外側クリック)
}

// 選択肢を並べた確認の窓。window.confirm では「はい / いいえ」しか出せず、
// 何が起きるかをボタンに書けないので自前で持つ。
export function ChoiceDialog({ request }: { request: ChoiceRequest | null }) {
  useEffect(() => {
    if (!request) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") request.resolve(null);
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [request]);

  if (!request) return null;
  return (
    <div
      className="fixed inset-0 z-50 flex items-center justify-center bg-black/60 p-4"
      onMouseDown={(e) => {
        if (e.target === e.currentTarget) request.resolve(null);
      }}
    >
      <div
        role="dialog"
        aria-modal="true"
        data-choice-dialog
        className="flex w-full max-w-lg flex-col gap-3 rounded-xl bg-card p-4 text-sm shadow-xl ring-1 ring-foreground/20"
      >
        <div className="text-base font-semibold">{request.title}</div>
        <div className="flex flex-col gap-1.5 text-muted-foreground">{request.body}</div>
        <div className="flex flex-col gap-1.5">
          {request.choices.map((c) => (
            <Button
              key={c.key}
              variant={c.variant ?? "outline"}
              className="h-auto min-h-8 justify-start py-1.5 text-left whitespace-normal"
              onClick={() => request.resolve(c.key)}
            >
              {c.label}
            </Button>
          ))}
          <Button variant="ghost" onClick={() => request.resolve(null)}>
            やめる
          </Button>
        </div>
      </div>
    </div>
  );
}
