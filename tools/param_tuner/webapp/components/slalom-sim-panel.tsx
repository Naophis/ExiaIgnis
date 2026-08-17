"use client";

import { useEffect, useMemo, useState } from "react";
import { load as loadYaml } from "js-yaml";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Separator } from "@/components/ui/separator";
import {
  simulateSlalom,
  TURN_DEFAULTS,
  TURN_TYPES,
  TURN_TYPE_LABELS,
  type SlalomSimResult,
  type TurnType,
} from "@/lib/slalom-sim";
import { SlalomSimPlot } from "@/components/slalom-sim-plot";
import { cn } from "@/lib/utils";

interface Props {
  file: string;
  draft: string;
}

interface Fields {
  v: string;
  rad: string;
  n: string;
  ang: string;
}

function parseFileSpeed(file: string): number | null {
  const m = file.match(/^t_(\d+)\.yaml$/i);
  return m ? Number(m[1]) : null;
}

// Reads the currently-edited draft (not the saved file) so the panel always
// reflects what's on screen, including unsaved changes.
function readBlockFromDraft(draft: string, type: TurnType): Partial<Fields> | null {
  let doc: unknown;
  try {
    doc = loadYaml(draft);
  } catch {
    return null;
  }
  if (typeof doc !== "object" || doc === null) return null;
  const block = (doc as Record<string, unknown>)[type];
  if (typeof block !== "object" || block === null) return null;
  const b = block as Record<string, unknown>;
  const out: Partial<Fields> = {};
  if (typeof b.v === "number") out.v = String(b.v);
  if (typeof b.rad === "number") out.rad = String(b.rad);
  if (typeof b.pow_n === "number") out.n = String(b.pow_n);
  if (typeof b.ang === "number") out.ang = String(b.ang);
  return out;
}

function defaultFields(type: TurnType, fileSpeed: number | null): Fields {
  const d = TURN_DEFAULTS[type];
  return { v: String(fileSpeed ?? 1200), rad: String(d.rad), n: String(d.n), ang: String(d.ang) };
}

// Draft values win field-by-field over the type defaults, so a file that
// only has (say) `rad` hand-tuned still gets sane v/n/ang.
function resolveFields(draft: string, type: TurnType, fileSpeed: number | null): Fields {
  const defaults = defaultFields(type, fileSpeed);
  const fromDraft = readBlockFromDraft(draft, type);
  return { ...defaults, ...fromDraft };
}

export function SlalomSimPanel({ file, draft }: Props) {
  const fileSpeed = useMemo(() => parseFileSpeed(file), [file]);
  const [type, setType] = useState<TurnType>("normal");
  const [fields, setFields] = useState<Fields>(() => resolveFields(draft, "normal", fileSpeed));

  const reloadFromDraft = (nextType: TurnType) => {
    setFields(resolveFields(draft, nextType, fileSpeed));
  };

  // Adjust state during render when `type` changes (react.dev's "adjusting
  // state when a prop changes" pattern) instead of an effect, so switching
  // turn types re-syncs fields from the draft without an extra render pass.
  const [prevType, setPrevType] = useState(type);
  if (type !== prevType) {
    setPrevType(type);
    setFields(resolveFields(draft, type, fileSpeed));
  }

  const [result, setResult] = useState<SlalomSimResult | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const handle = setTimeout(() => {
      const v = parseFloat(fields.v);
      const rad = parseFloat(fields.rad);
      const n = parseFloat(fields.n);
      const ang = parseFloat(fields.ang);
      if (![v, rad, n, ang].every(Number.isFinite) || v <= 0 || rad === 0 || n <= 0 || ang <= 0) {
        setError("v / rad / pow_n / ang に有効な数値を入力してください");
        setResult(null);
        return;
      }
      try {
        setResult(simulateSlalom({ type, v, rad, n, ang }));
        setError(null);
      } catch (err) {
        setError((err as Error).message);
        setResult(null);
      }
    }, 150);
    return () => clearTimeout(handle);
  }, [type, fields]);

  const setField = (key: keyof Fields) => (e: React.ChangeEvent<HTMLInputElement>) =>
    setFields((prev) => ({ ...prev, [key]: e.target.value }));

  return (
    <Card className="flex w-[34rem] min-w-[20rem] shrink flex-col overflow-hidden">
      <CardHeader className="gap-1">
        <div className="flex items-center justify-between">
          <CardTitle>スラロームシミュレータ</CardTitle>
          <Button size="sm" variant="outline" onClick={() => reloadFromDraft(type)}>
            編集中のYAMLから再読込
          </Button>
        </div>
      </CardHeader>
      <CardContent className="flex min-h-0 flex-1 flex-col gap-3 overflow-y-auto">
        <div className="flex flex-col gap-1">
          <span className="text-xs text-muted-foreground">ターン種別</span>
          <div className="flex min-w-0 flex-wrap gap-1">
            {TURN_TYPES.map((t) => {
              const active = t === type;
              return (
                <button
                  key={t}
                  type="button"
                  onClick={() => setType(t)}
                  className={cn(
                    "rounded-md px-2.5 py-1.5 text-xs font-medium whitespace-nowrap ring-1 ring-border transition-colors",
                    active
                      ? "bg-primary text-primary-foreground ring-primary"
                      : "bg-transparent text-muted-foreground hover:bg-muted hover:text-foreground"
                  )}
                >
                  {TURN_TYPE_LABELS[t]}
                </button>
              );
            })}
          </div>
        </div>

        <div className="grid grid-cols-4 gap-2">
          <label className="flex flex-col gap-1 text-xs text-muted-foreground">
            v (mm/s)
            <Input inputMode="decimal" value={fields.v} onChange={setField("v")} />
          </label>
          <label className="flex flex-col gap-1 text-xs text-muted-foreground">
            rad (mm)
            <Input inputMode="decimal" value={fields.rad} onChange={setField("rad")} />
          </label>
          <label className="flex flex-col gap-1 text-xs text-muted-foreground">
            pow_n
            <Input inputMode="decimal" value={fields.n} onChange={setField("n")} />
          </label>
          <label className="flex flex-col gap-1 text-xs text-muted-foreground">
            ang (deg)
            <Input inputMode="decimal" value={fields.ang} onChange={setField("ang")} />
          </label>
        </div>

        <Separator />

        {error ? (
          <span className="text-sm text-destructive">{error}</span>
        ) : result ? (
          <div className="grid grid-cols-2 gap-x-4 gap-y-1 text-sm">
            <span className="text-muted-foreground">所要時間</span>
            <span>{(result.time * 1000).toFixed(1)} ms</span>
            <span className="text-muted-foreground">front</span>
            <span>{result.offsetSupported ? `${result.front.toFixed(2)} mm` : "未実装 (0)"}</span>
            <span className="text-muted-foreground">back</span>
            <span>{result.offsetSupported ? `${result.back.toFixed(2)} mm` : "未実装 (0)"}</span>
            <span className="text-muted-foreground">最大遠心加速度</span>
            <span>{result.maxAccG.toFixed(2)} G</span>
          </div>
        ) : null}
        {result && !result.offsetSupported && (
          <span className="text-xs text-muted-foreground">
            orval は元の Python ツールでも前後オフセット計算が未実装のため、front/back は常に 0 になります。
          </span>
        )}

        <div className="h-80 shrink-0 overflow-hidden rounded-md border">
          <SlalomSimPlot result={result} />
        </div>

        {result && (
          <pre className="overflow-x-auto rounded-md border bg-muted/30 p-2 text-xs">
            {`${type}:
  v: ${fields.v}
  ang: ${fields.ang}
  pow_n: ${fields.n}
  rad: ${fields.rad}${
              result.offsetSupported
                ? `
  front: { left: ${result.front.toFixed(4)}, right: ${result.front.toFixed(4)} }
  back: { left: ${result.back.toFixed(4)}, right: ${result.back.toFixed(4)} }`
                : ""
            }`}
          </pre>
        )}
      </CardContent>
    </Card>
  );
}
