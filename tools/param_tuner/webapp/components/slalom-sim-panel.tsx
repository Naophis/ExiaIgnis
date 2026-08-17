"use client";

import { useEffect, useMemo, useState } from "react";
import { load as loadYaml } from "js-yaml";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Separator } from "@/components/ui/separator";
import {
  DEFAULT_SLIP_K,
  DEFAULT_SLIP_KY,
  simulateSlalom,
  TURN_DEFAULTS,
  TURN_TYPES,
  TURN_TYPE_LABELS,
  type SlalomSimResult,
  type TurnType,
} from "@/lib/slalom-sim";
import { APPLY_ALL_SELECTION, applySimResultToYaml, type SlalomApplySelection } from "@/lib/slalom-yaml-patch";
import {
  DEFAULT_LAYER_VISIBILITY,
  SIM_PLOT_STYLE,
  SlalomSimPlot,
  type LayerVisibility,
  type SimLayerKey,
} from "@/components/slalom-sim-plot";
import { cn } from "@/lib/utils";

interface Props {
  file: string;
  draft: string;
  onApply: (nextDraft: string) => void;
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

export function SlalomSimPanel({ file, draft, onApply }: Props) {
  const fileSpeed = useMemo(() => parseFileSpeed(file), [file]);
  const [type, setType] = useState<TurnType>("normal");
  const [fields, setFields] = useState<Fields>(() => resolveFields(draft, "normal", fileSpeed));
  // Slip params aren't part of the yaml turn blocks - they're a shared,
  // hand-tuned starting point (hardware.yaml's slip_param_k2/slip_param_K),
  // so they persist across turn-type switches instead of resetting.
  const [slipK, setSlipK] = useState(String(DEFAULT_SLIP_K));
  const [slipKy, setSlipKy] = useState(String(DEFAULT_SLIP_KY));
  // Which fields "適用" actually writes - lets e.g. only `rad` be pushed
  // into the yaml while the rest of the block stays as hand-tuned.
  const [applySelection, setApplySelection] = useState<SlalomApplySelection>(APPLY_ALL_SELECTION);
  const toggleApplyField = (key: keyof SlalomApplySelection) =>
    setApplySelection((prev) => ({ ...prev, [key]: !prev[key] }));
  // Show/hide individual plot layers - the 4 lines can overlap and be hard
  // to pick apart, so let the user declutter instead of chasing colors.
  const [layerVisible, setLayerVisible] = useState<LayerVisibility>(DEFAULT_LAYER_VISIBILITY);
  const toggleLayer = (key: SimLayerKey) => setLayerVisible((prev) => ({ ...prev, [key]: !prev[key] }));

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
      const K = parseFloat(slipK);
      const Ky = parseFloat(slipKy);
      if (
        ![v, rad, n, ang, K, Ky].every(Number.isFinite) ||
        v <= 0 ||
        rad === 0 ||
        n <= 0 ||
        ang <= 0
      ) {
        setError("v / rad / pow_n / ang / K / K_y に有効な数値を入力してください");
        setResult(null);
        return;
      }
      try {
        setResult(simulateSlalom({ type, v, rad, n, ang, K, Ky }));
        setError(null);
      } catch (err) {
        setError((err as Error).message);
        setResult(null);
      }
    }, 150);
    return () => clearTimeout(handle);
  }, [type, fields, slipK, slipKy]);

  const setField = (key: keyof Fields) => (e: React.ChangeEvent<HTMLInputElement>) =>
    setFields((prev) => ({ ...prev, [key]: e.target.value }));

  const nothingSelected = Object.values(applySelection).every((v) => !v);

  const applyToYaml = () => {
    if (!result || nothingSelected) return;
    try {
      const nextDraft = applySimResultToYaml(draft, type, fields, result, applySelection);
      onApply(nextDraft);
      toast.success(`${type}: 編集中のYAMLに反映しました（保存はCtrl+Sで）`);
    } catch (err) {
      toast.error((err as Error).message);
    }
  };

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

        <div className="grid grid-cols-4 gap-2">
          <label className="flex flex-col gap-1 text-xs text-muted-foreground">
            K (slip_param_k2)
            <Input inputMode="decimal" value={slipK} onChange={(e) => setSlipK(e.target.value)} />
          </label>
          <label className="flex flex-col gap-1 text-xs text-muted-foreground">
            K_y (slip_param_K)
            <Input inputMode="decimal" value={slipKy} onChange={(e) => setSlipKy(e.target.value)} />
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
          <SlalomSimPlot result={result} visible={layerVisible} />
        </div>
        <div className="flex flex-wrap gap-x-1 gap-y-1 text-xs">
          {(
            [
              ["idealPath", "理想軌道（スリップ未考慮）", SIM_PLOT_STYLE.idealPath.color, false],
              ["idealOffset", "理想軌道の前後オフセット", SIM_PLOT_STYLE.idealOffset.color, false],
              ["slipPath", "スリップ考慮軌道", SIM_PLOT_STYLE.slipPath.color, true],
              ["slipOffset", "スリップ軌道の前後オフセット", SIM_PLOT_STYLE.slipOffset.color, true],
            ] as const
          ).map(([key, label, color, dashed]) => {
            const on = layerVisible[key];
            return (
              <button
                key={key}
                type="button"
                onClick={() => toggleLayer(key)}
                title="クリックで表示/非表示を切り替え"
                className={cn(
                  "flex items-center gap-1.5 rounded-md px-1.5 py-1 transition-colors hover:bg-muted",
                  on ? "text-foreground" : "text-muted-foreground/50 line-through"
                )}
              >
                <span
                  className={cn("inline-block h-0.5 w-4 shrink-0", dashed && "border-t-2 border-dashed")}
                  style={{
                    backgroundColor: dashed ? undefined : color,
                    borderColor: dashed ? color : undefined,
                    opacity: on ? 1 : 0.4,
                  }}
                />
                {label}
              </button>
            );
          })}
        </div>

        {result && (
          <div className="flex flex-col gap-1.5">
            <span className="text-xs text-muted-foreground">適用する項目</span>
            <div className="flex flex-wrap gap-1">
              {(
                [
                  ["v", "v"],
                  ["ang", "ang"],
                  ["n", "pow_n"],
                  ["rad", "rad"],
                  ...(result.offsetSupported ? ([["front", "front"], ["back", "back"]] as const) : []),
                ] as const
              ).map(([key, label]) => {
                const active = applySelection[key];
                return (
                  <button
                    key={key}
                    type="button"
                    onClick={() => toggleApplyField(key)}
                    className={cn(
                      "rounded-md px-2.5 py-1 text-xs font-medium whitespace-nowrap ring-1 ring-border transition-colors",
                      active
                        ? "bg-primary text-primary-foreground ring-primary"
                        : "bg-transparent text-muted-foreground hover:bg-muted hover:text-foreground"
                    )}
                  >
                    {label}
                  </button>
                );
              })}
            </div>
            <Button size="sm" disabled={nothingSelected} onClick={applyToYaml} className="self-start">
              選択した項目を編集中のYAMLに適用
            </Button>
          </div>
        )}

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
