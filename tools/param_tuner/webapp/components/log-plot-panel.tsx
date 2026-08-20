"use client";

import { useCallback, useEffect, useMemo, useState } from "react";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { TrajectoryPlot } from "@/components/trajectory-plot";
import { buildTrajectoryData, parseCsv, type TrajectoryPoint } from "@/lib/trajectory";

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

interface AutoOpenRequest {
  file: string;
  nonce: number;
}

export function LogPlotPanel({ autoOpen }: { autoOpen?: AutoOpenRequest | null }) {
  const [files, setFiles] = useState<LogFileInfo[]>([]);
  const [selected, setSelected] = useState<string | null>(null);
  const [csvText, setCsvText] = useState<string | null>(null);
  const [showLeft45, setShowLeft45] = useState(true);
  const [showRight45, setShowRight45] = useState(true);
  const [clickInfo, setClickInfo] = useState<string | null>(null);
  const [pjBusy, setPjBusy] = useState(false);

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

  const trajectoryData = useMemo(() => {
    if (!csvText) return null;
    return buildTrajectoryData(parseCsv(csvText));
  }, [csvText]);

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
  useEffect(() => {
    if (!autoOpen) return;
    // eslint-disable-next-line react-hooks/set-state-in-effect
    setSelected(autoOpen.file);
    void openPlotJuggler(autoOpen.file);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [autoOpen?.nonce]);

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
      <div className="flex w-56 shrink-0 flex-col overflow-hidden border-r border-border">
        <div className="flex items-center justify-between p-2">
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
          <div className="flex flex-col p-1">
            {files.map((f) => (
              <button
                key={f.name}
                type="button"
                onClick={() => setSelected(f.name)}
                onDoubleClick={() => void openPlotJuggler(f.name)}
                className={`flex flex-col rounded px-2 py-1.5 text-left text-xs transition-colors ${
                  selected === f.name ? "bg-primary text-primary-foreground" : "hover:bg-muted"
                }`}
              >
                <span className="truncate font-medium">{f.name}</span>
                <span className={selected === f.name ? "text-primary-foreground/70" : "text-muted-foreground"}>
                  {formatDate(f.mtimeMs)}
                </span>
              </button>
            ))}
            {files.length === 0 && (
              <span className="px-2 py-1 text-xs text-muted-foreground">ログファイルがありません</span>
            )}
          </div>
        </ScrollArea>
      </div>

      <div className="flex flex-1 flex-col overflow-hidden">
        <div className="flex items-center gap-2 p-2">
          <label className="flex items-center gap-1 text-xs">
            <input type="checkbox" checked={showLeft45} onChange={(e) => setShowLeft45(e.target.checked)} />
            Left45
          </label>
          <label className="flex items-center gap-1 text-xs">
            <input type="checkbox" checked={showRight45} onChange={(e) => setShowRight45(e.target.checked)} />
            Right45
          </label>
          <div className="flex-1" />
          <Button size="sm" variant="outline" disabled={!selected || pjBusy} onClick={() => void openPlotJuggler()}>
            PlotJugglerで開く
          </Button>
          <Button size="sm" variant="outline" onClick={killPlotJuggler}>
            PJを終了
          </Button>
        </div>
        <Separator />
        <div className="min-h-0 flex-1">
          <TrajectoryPlot
            data={trajectoryData}
            showLeft45={showLeft45}
            showRight45={showRight45}
            onPointClick={(p) => setClickInfo(p ? formatClickInfo(p) : null)}
          />
        </div>
        <Separator />
        <div className="p-2 font-mono text-xs text-muted-foreground">
          {clickInfo ?? (trajectoryData ? "点をクリックすると詳細を表示します" : "x/y列を含むログを選択してください")}
        </div>
      </div>
    </Card>
  );
}
