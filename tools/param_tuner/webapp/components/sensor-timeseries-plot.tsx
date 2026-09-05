"use client";

import { useEffect, useRef, useState } from "react";
import type { AnalysisEvent } from "@/lib/log-analysis";

export interface TimeSeries {
  column: string;
  color: string;
  dash?: boolean; // draw as a dashed line (e.g. a companion/raw column plotted alongside its processed counterpart)
  points: { x: number; y: number }[]; // x = CSV "index" column, y = sensor value
}

interface Props {
  series: TimeSeries[];
  markers: AnalysisEvent[]; // filtered to events carrying seriesIndex/seriesValue
}

const MARKER_COLOR: Partial<Record<AnalysisEvent["kind"], string>> = {
  drop: "#ff5555",
  rise: "#3ddc84",
  trough: "#c678f5",
  "trough-rise": "#4ad4d4",
  "wall-off-actual": "#ff8c42",
  "wall-off-arm": "#eab308",
  "wall-off-edge": "#22c55e",
};

const PAD_LEFT = 48;
const PAD_RIGHT = 12;
const PAD_TOP = 12;
const PAD_BOTTOM = 24;
const MIN_DOMAIN = 10; // samples; zoom can't shrink the visible x-range below this
const DRAG_CLICK_THRESHOLD = 4;

interface Domain {
  xMin: number;
  xMax: number;
}

function dataExtentX(series: TimeSeries[]): Domain | null {
  let xMin = Infinity;
  let xMax = -Infinity;
  for (const s of series) {
    for (const p of s.points) {
      if (p.x < xMin) xMin = p.x;
      if (p.x > xMax) xMax = p.x;
    }
  }
  if (!Number.isFinite(xMin) || !Number.isFinite(xMax)) return null;
  return { xMin, xMax };
}

function yExtentInDomain(series: TimeSeries[], domain: Domain): { yMin: number; yMax: number } {
  let yMin = Infinity;
  let yMax = -Infinity;
  for (const s of series) {
    for (const p of s.points) {
      if (p.x < domain.xMin || p.x > domain.xMax) continue;
      if (p.y < yMin) yMin = p.y;
      if (p.y > yMax) yMax = p.y;
    }
  }
  if (!Number.isFinite(yMin) || !Number.isFinite(yMax)) return { yMin: 0, yMax: 1 };
  if (yMin === yMax) return { yMin: yMin - 1, yMax: yMax + 1 };
  const pad = (yMax - yMin) * 0.08;
  return { yMin: yMin - pad, yMax: yMax + pad };
}

export function SensorTimeseriesPlot({ series, markers }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });
  const fullDomain = dataExtentX(series);
  const [domain, setDomain] = useState<Domain | null>(fullDomain);
  const domainRef = useRef(domain);
  useEffect(() => {
    domainRef.current = domain;
  }, [domain]);
  const dragRef = useRef<{ startX: number; startDomain: Domain; moved: boolean } | null>(null);
  const [hoverInfo, setHoverInfo] = useState<string | null>(null);

  // New data (file switch, or series/columns toggled) -> refit to full extent.
  const fullKey = fullDomain ? `${fullDomain.xMin}:${fullDomain.xMax}:${series.length}` : "none";
  const [prevKey, setPrevKey] = useState(fullKey);
  if (fullKey !== prevKey) {
    setPrevKey(fullKey);
    setDomain(fullDomain);
  }

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const ro = new ResizeObserver((entries) => {
      const box = entries[0].contentRect;
      setSize({ width: box.width, height: box.height });
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !fullDomain) return;
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const rect = canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const availW = Math.max(rect.width - PAD_LEFT - PAD_RIGHT, 1);
      setDomain((d) => {
        const cur = d ?? fullDomain;
        const span = cur.xMax - cur.xMin;
        const factor = Math.exp(-e.deltaY * 0.0015);
        const fullSpan = fullDomain.xMax - fullDomain.xMin;
        const newSpan = Math.min(fullSpan, Math.max(MIN_DOMAIN, span / factor));
        if (newSpan === span) return cur;
        const mouseFrac = (mx - PAD_LEFT) / availW;
        const mouseX = cur.xMin + mouseFrac * span;
        let xMin = mouseX - mouseFrac * newSpan;
        let xMax = xMin + newSpan;
        if (xMin < fullDomain.xMin) {
          xMin = fullDomain.xMin;
          xMax = xMin + newSpan;
        }
        if (xMax > fullDomain.xMax) {
          xMax = fullDomain.xMax;
          xMin = xMax - newSpan;
        }
        return { xMin, xMax };
      });
    };
    canvas.addEventListener("wheel", onWheel, { passive: false });
    return () => canvas.removeEventListener("wheel", onWheel);
  }, [fullDomain]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || size.width === 0 || size.height === 0) return;
    const dpr = window.devicePixelRatio || 1;
    canvas.width = size.width * dpr;
    canvas.height = size.height * dpr;
    canvas.style.width = `${size.width}px`;
    canvas.style.height = `${size.height}px`;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = "#0d1117";
    ctx.fillRect(0, 0, size.width, size.height);

    if (!domain || series.length === 0) return;
    const availW = Math.max(size.width - PAD_LEFT - PAD_RIGHT, 1);
    const availH = Math.max(size.height - PAD_TOP - PAD_BOTTOM, 1);
    const { yMin, yMax } = yExtentInDomain(series, domain);
    const scaleX = availW / Math.max(domain.xMax - domain.xMin, 1e-6);
    const scaleY = availH / Math.max(yMax - yMin, 1e-6);
    const toX = (x: number) => PAD_LEFT + (x - domain.xMin) * scaleX;
    const toY = (y: number) => PAD_TOP + availH - (y - yMin) * scaleY;

    // y gridlines + labels
    ctx.strokeStyle = "rgba(255,255,255,0.08)";
    ctx.fillStyle = "rgba(255,255,255,0.5)";
    ctx.font = "10px monospace";
    ctx.lineWidth = 1;
    const yTicks = 4;
    for (let t = 0; t <= yTicks; t++) {
      const yv = yMin + ((yMax - yMin) * t) / yTicks;
      const cy = toY(yv);
      ctx.beginPath();
      ctx.moveTo(PAD_LEFT, cy);
      ctx.lineTo(size.width - PAD_RIGHT, cy);
      ctx.stroke();
      ctx.fillText(yv.toFixed(1), 2, cy + 3);
    }
    // x ticks
    const xTicks = 5;
    for (let t = 0; t <= xTicks; t++) {
      const xv = domain.xMin + ((domain.xMax - domain.xMin) * t) / xTicks;
      const cx = toX(xv);
      ctx.fillText(Math.round(xv).toString(), cx - 10, size.height - 6);
    }

    // series lines
    for (const s of series) {
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1.3;
      ctx.setLineDash(s.dash ? [5, 3] : []);
      ctx.beginPath();
      let started = false;
      for (const p of s.points) {
        if (p.x < domain.xMin || p.x > domain.xMax) continue;
        const cx = toX(p.x);
        const cy = toY(p.y);
        if (!started) {
          ctx.moveTo(cx, cy);
          started = true;
        } else {
          ctx.lineTo(cx, cy);
        }
      }
      if (started) ctx.stroke();
    }
    ctx.setLineDash([]);

    // markers
    for (const m of markers) {
      if (m.seriesIndex === undefined || m.seriesValue === undefined) continue;
      if (m.seriesIndex < domain.xMin || m.seriesIndex > domain.xMax) continue;
      const color = MARKER_COLOR[m.kind];
      if (!color) continue;
      const cx = toX(m.seriesIndex);
      const cy = toY(m.seriesValue);
      ctx.fillStyle = color;
      ctx.strokeStyle = "#0d1117";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.arc(cx, cy, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }

    // legend (dashed series get an outlined swatch instead of filled, to read
    // as "companion of the solid line above" rather than an unrelated series)
    let lx = PAD_LEFT;
    ctx.font = "11px sans-serif";
    for (const s of series) {
      if (s.dash) {
        ctx.strokeStyle = s.color;
        ctx.lineWidth = 1.5;
        ctx.strokeRect(lx + 1, 3, 8, 8);
      } else {
        ctx.fillStyle = s.color;
        ctx.fillRect(lx, 2, 10, 10);
      }
      ctx.fillStyle = "rgba(255,255,255,0.8)";
      ctx.fillText(s.column, lx + 14, 11);
      lx += 14 + ctx.measureText(s.column).width + 14;
    }
  }, [series, markers, size, domain]);

  const handlePointerDown = (e: React.PointerEvent<HTMLCanvasElement>) => {
    if (!domain) return;
    (e.target as HTMLCanvasElement).setPointerCapture(e.pointerId);
    dragRef.current = { startX: e.clientX, startDomain: domain, moved: false };
  };

  const handlePointerMove = (e: React.PointerEvent<HTMLCanvasElement>) => {
    const drag = dragRef.current;
    const canvas = canvasRef.current;
    if (drag && canvas) {
      const dx = e.clientX - drag.startX;
      if (!drag.moved && Math.abs(dx) < DRAG_CLICK_THRESHOLD) return;
      drag.moved = true;
      const availW = Math.max(size.width - PAD_LEFT - PAD_RIGHT, 1);
      const span = drag.startDomain.xMax - drag.startDomain.xMin;
      const shift = (-dx / availW) * span;
      let xMin = drag.startDomain.xMin + shift;
      let xMax = drag.startDomain.xMax + shift;
      if (fullDomain) {
        if (xMin < fullDomain.xMin) {
          xMax += fullDomain.xMin - xMin;
          xMin = fullDomain.xMin;
        }
        if (xMax > fullDomain.xMax) {
          xMin -= xMax - fullDomain.xMax;
          xMax = fullDomain.xMax;
        }
      }
      setDomain({ xMin, xMax });
      return;
    }
    // hover readout when not dragging
    if (!canvas || !domainRef.current || series.length === 0) return;
    const rect = canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const availW = Math.max(rect.width - PAD_LEFT - PAD_RIGHT, 1);
    const d = domainRef.current;
    const xAt = d.xMin + ((mx - PAD_LEFT) / availW) * (d.xMax - d.xMin);
    const parts = [`index≈${xAt.toFixed(0)}`];
    for (const s of series) {
      let nearest: { x: number; y: number } | null = null;
      let best = Infinity;
      for (const p of s.points) {
        const dd = Math.abs(p.x - xAt);
        if (dd < best) {
          best = dd;
          nearest = p;
        }
      }
      if (nearest) parts.push(`${s.column}=${nearest.y.toFixed(1)}`);
    }
    setHoverInfo(parts.join(" | "));
  };

  const handlePointerUp = () => {
    dragRef.current = null;
  };

  const resetDomain = () => setDomain(fullDomain);

  const zoomed = !!(domain && fullDomain && (domain.xMin !== fullDomain.xMin || domain.xMax !== fullDomain.xMax));

  return (
    <div className="flex h-full w-full flex-col">
      <div ref={containerRef} className="relative min-h-0 flex-1">
        <canvas
          ref={canvasRef}
          onPointerDown={handlePointerDown}
          onPointerMove={handlePointerMove}
          onPointerUp={handlePointerUp}
          onPointerCancel={handlePointerUp}
          onPointerLeave={() => setHoverInfo(null)}
          className={zoomed ? "cursor-grab active:cursor-grabbing" : "cursor-crosshair"}
        />
        {zoomed && (
          <button
            type="button"
            className="absolute bottom-1.5 right-1.5 rounded bg-black/60 px-1.5 py-0.5 text-[10px] text-white/80 underline hover:text-white"
            onClick={resetDomain}
          >
            リセット
          </button>
        )}
      </div>
      <div className="p-1 font-mono text-[11px] text-muted-foreground">
        {hoverInfo ?? "グラフ上でホイール=ズーム、ドラッグ=パン"}
      </div>
    </div>
  );
}
