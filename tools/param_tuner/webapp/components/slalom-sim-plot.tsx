"use client";

import { useEffect, useRef, useState } from "react";
import type { SlalomSimResult } from "@/lib/slalom-sim";

export type SimLayerKey = "idealPath" | "idealOffset" | "slipPath" | "slipOffset";
export type LayerVisibility = Record<SimLayerKey, boolean>;
export const DEFAULT_LAYER_VISIBILITY: LayerVisibility = {
  idealPath: true,
  idealOffset: true,
  slipPath: true,
  slipOffset: true,
};

interface Props {
  result: SlalomSimResult | null;
  visible?: LayerVisibility;
}

const PADDING = 20;
const WALL_LINES = [-45, 45, 135, 225];
const CENTER_LINES = [-90, 0, 90, 180];

// Warm family = idealized (path thick+solid = the main line, offset
// thinner+solid but still bold), cool family = slip overlay (both dashed,
// distinct dash pitch). Deliberately avoids red/vermillion entirely - the
// wall lines below are already a translucent red, so a red trajectory would
// wash out exactly where it crosses a wall. Exported so the panel's legend
// swatches match exactly instead of hand-copied hex values drifting out of
// sync.
export const SIM_PLOT_STYLE = {
  idealPath: { color: "#fde047", width: 5, dash: [] as number[] }, // yellow-300 - thick, the main line
  idealOffset: { color: "#fb923c", width: 3, dash: [] as number[] }, // orange-400
  slipPath: { color: "#38bdf8", width: 3, dash: [8, 4] }, // sky-400
  slipOffset: { color: "#c084fc", width: 2.5, dash: [3, 3] }, // purple-400
};

type Bounds = { xMin: number; xMax: number; yMin: number; yMax: number };

const FALLBACK_BOUNDS: Bounds = { xMin: -60, xMax: 150, yMin: -60, yMax: 150 };
const CONTENT_MARGIN = 15; // mm of breathing room around the trajectory
const MIN_SPAN = 60; // mm - keeps very small/short turns from zooming in absurdly

// Unlike plot.py's Plot.exe (one fixed -60..240 frame shared by every turn
// type, sized for the largest one), fit the view to *this* result's actual
// extent: most turns only use a fraction of that frame, leaving the plot
// mostly empty grid. Square (via the shared max-span), so aspect stays 1:1.
function computeBounds(result: SlalomSimResult | null, visible: LayerVisibility): Bounds {
  if (!result) return FALLBACK_BOUNDS;
  const points: { x: number; y: number }[] = [];
  if (visible.idealPath) points.push(...result.path);
  if (visible.slipPath) points.push(...result.slipPath);
  if (visible.idealOffset) points.push(...result.prevPath, ...result.afterPath);
  if (visible.slipOffset) points.push(...result.slipPrevPath, ...result.slipAfterPath);
  if (points.length === 0) return FALLBACK_BOUNDS;

  let xMin = Infinity;
  let xMax = -Infinity;
  let yMin = Infinity;
  let yMax = -Infinity;
  for (const p of points) {
    xMin = Math.min(xMin, p.x);
    xMax = Math.max(xMax, p.x);
    yMin = Math.min(yMin, p.y);
    yMax = Math.max(yMax, p.y);
  }
  xMin -= CONTENT_MARGIN;
  xMax += CONTENT_MARGIN;
  yMin -= CONTENT_MARGIN;
  yMax += CONTENT_MARGIN;

  const span = Math.max(xMax - xMin, yMax - yMin, MIN_SPAN);
  const cx = (xMin + xMax) / 2;
  const cy = (yMin + yMax) / 2;
  return { xMin: cx - span / 2, xMax: cx + span / 2, yMin: cy - span / 2, yMax: cy + span / 2 };
}

function makeTransform(bounds: Bounds, w: number, h: number) {
  const worldW = bounds.xMax - bounds.xMin;
  const worldH = bounds.yMax - bounds.yMin;
  const availW = Math.max(w - PADDING * 2, 1);
  const availH = Math.max(h - PADDING * 2, 1);
  const scale = Math.min(availW / worldW, availH / worldH);
  const offsetX = PADDING + (availW - worldW * scale) / 2;
  const offsetY = PADDING + (availH - worldH * scale) / 2;
  return (wx: number, wy: number): [number, number] => [
    offsetX + (wx - bounds.xMin) * scale,
    h - offsetY - (wy - bounds.yMin) * scale,
  ];
}

export function SlalomSimPlot({ result, visible = DEFAULT_LAYER_VISIBILITY }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });

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
    if (!canvas || size.width === 0 || size.height === 0) return;

    const dpr = window.devicePixelRatio || 1;
    canvas.width = size.width * dpr;
    canvas.height = size.height * dpr;
    canvas.style.width = `${size.width}px`;
    canvas.style.height = `${size.height}px`;

    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    // Darker than before (was #3a3a3a) - the mid-gray was competing with the
    // bright path colors for contrast; a near-black ground makes every line
    // below pop regardless of hue.
    ctx.fillStyle = "#18181b";
    ctx.fillRect(0, 0, size.width, size.height);

    const toCanvas = makeTransform(computeBounds(result, visible), size.width, size.height);
    const range: [number, number] = [-1000, 1000];

    ctx.lineWidth = 5;
    ctx.strokeStyle = "rgba(255,60,60,0.25)";
    for (const v of WALL_LINES) {
      ctx.beginPath();
      let [x1, y1] = toCanvas(range[0], v);
      let [x2, y2] = toCanvas(range[1], v);
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
      ctx.beginPath();
      [x1, y1] = toCanvas(v, range[0]);
      [x2, y2] = toCanvas(v, range[1]);
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    }

    ctx.lineWidth = 0.75;
    ctx.strokeStyle = "rgba(192,192,192,0.5)";
    ctx.setLineDash([4, 4]);
    for (const v of CENTER_LINES) {
      ctx.beginPath();
      let [x1, y1] = toCanvas(range[0], v);
      let [x2, y2] = toCanvas(range[1], v);
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
      ctx.beginPath();
      [x1, y1] = toCanvas(v, range[0]);
      [x2, y2] = toCanvas(v, range[1]);
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    }
    ctx.setLineDash([]);

    if (!result) return;

    // Every line gets a dark halo stroked first (wider, same dash) so it
    // reads clearly no matter what's directly behind it - a wall line, a
    // center line, or another trajectory it happens to cross.
    const strokeStyled = (points: { x: number; y: number }[], style: (typeof SIM_PLOT_STYLE)[keyof typeof SIM_PLOT_STYLE]) => {
      const tracePath = () => {
        ctx.beginPath();
        points.forEach((p, i) => {
          const [cx, cy] = toCanvas(p.x, p.y);
          if (i === 0) ctx.moveTo(cx, cy);
          else ctx.lineTo(cx, cy);
        });
      };
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      ctx.setLineDash(style.dash);

      ctx.strokeStyle = "rgba(0,0,0,0.7)";
      ctx.lineWidth = style.width + 2.5;
      tracePath();
      ctx.stroke();

      ctx.strokeStyle = style.color;
      ctx.lineWidth = style.width;
      tracePath();
      ctx.stroke();

      ctx.setLineDash([]);
    };

    if (result.offsetSupported) {
      // Idealized lead-in/lead-out vs. the slip path's own, independently-
      // solved lead-in/lead-out - these two segments generally don't
      // coincide, which is the point: it shows how far off the planned cell
      // alignment the slip-affected path lands.
      if (visible.idealOffset) {
        strokeStyled(result.prevPath, SIM_PLOT_STYLE.idealOffset);
        strokeStyled(result.afterPath, SIM_PLOT_STYLE.idealOffset);
      }
      if (visible.slipOffset) {
        strokeStyled(result.slipPrevPath, SIM_PLOT_STYLE.slipOffset);
        strokeStyled(result.slipAfterPath, SIM_PLOT_STYLE.slipOffset);
      }
    }

    // Idealized (no-slip) path vs. the tire-slip overlay, both drawn in the
    // same cell-aligned frame so the gap between them directly shows how
    // far slip would drag the robot off-line.
    if (visible.idealPath) strokeStyled(result.path, SIM_PLOT_STYLE.idealPath);
    if (visible.slipPath) strokeStyled(result.slipPath, SIM_PLOT_STYLE.slipPath);

    const drawDot = (p: { x: number; y: number }, color: string) => {
      const [cx, cy] = toCanvas(p.x, p.y);
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(cx, cy, 3.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "rgba(0,0,0,0.7)";
      ctx.lineWidth = 1.5;
      ctx.stroke();
    };
    if (visible.idealPath) {
      drawDot(result.path[0], "white");
      drawDot(result.path[result.path.length - 1], "white");
    }
  }, [result, size, visible]);

  return (
    <div ref={containerRef} className="h-full w-full">
      <canvas ref={canvasRef} />
    </div>
  );
}
