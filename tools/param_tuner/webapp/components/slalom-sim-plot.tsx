"use client";

import { useEffect, useRef, useState } from "react";
import type { SlalomSimResult } from "@/lib/slalom-sim";

interface Props {
  result: SlalomSimResult | null;
}

const PADDING = 20;
// Matches plot.py's Plot.exe: a fixed world frame (mm) shared by every turn
// type so the visual scale stays comparable when switching between them.
const BOUNDS = { xMin: -60, xMax: 240, yMin: -60, yMax: 240 };
const WALL_LINES = [-45, 45, 135, 225];
const CENTER_LINES = [-90, 0, 90, 180];

function makeTransform(w: number, h: number) {
  const worldW = BOUNDS.xMax - BOUNDS.xMin;
  const worldH = BOUNDS.yMax - BOUNDS.yMin;
  const availW = Math.max(w - PADDING * 2, 1);
  const availH = Math.max(h - PADDING * 2, 1);
  const scale = Math.min(availW / worldW, availH / worldH);
  const offsetX = PADDING + (availW - worldW * scale) / 2;
  const offsetY = PADDING + (availH - worldH * scale) / 2;
  return (wx: number, wy: number): [number, number] => [
    offsetX + (wx - BOUNDS.xMin) * scale,
    h - offsetY - (wy - BOUNDS.yMin) * scale,
  ];
}

export function SlalomSimPlot({ result }: Props) {
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
    ctx.fillStyle = "#3a3a3a";
    ctx.fillRect(0, 0, size.width, size.height);

    const toCanvas = makeTransform(size.width, size.height);
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

    const drawSegment = (p0: { x: number; y: number }, p1: { x: number; y: number }, color: string) => {
      ctx.strokeStyle = color;
      ctx.lineWidth = 3;
      ctx.beginPath();
      const [x1, y1] = toCanvas(p0.x, p0.y);
      const [x2, y2] = toCanvas(p1.x, p1.y);
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    };

    if (result.offsetSupported) {
      drawSegment(result.prevPath[0], result.prevPath[1], "coral");
      drawSegment(result.afterPath[0], result.afterPath[1], "coral");
    }

    const drawPath = (points: { x: number; y: number }[], color: string, dashed: boolean) => {
      ctx.strokeStyle = color;
      ctx.lineWidth = dashed ? 2 : 3;
      ctx.setLineDash(dashed ? [5, 4] : []);
      ctx.beginPath();
      points.forEach((p, i) => {
        const [cx, cy] = toCanvas(p.x, p.y);
        if (i === 0) ctx.moveTo(cx, cy);
        else ctx.lineTo(cx, cy);
      });
      ctx.stroke();
      ctx.setLineDash([]);
    };

    // Idealized (no-slip) path in solid gold, tire-slip overlay dashed cyan -
    // both drawn in the same cell-aligned frame so the gap between them
    // directly shows how far slip would drag the robot off-line.
    drawPath(result.path, "gold", false);
    drawPath(result.slipPath, "deepskyblue", true);

    const drawDot = (p: { x: number; y: number }, color: string) => {
      const [cx, cy] = toCanvas(p.x, p.y);
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(cx, cy, 3.5, 0, Math.PI * 2);
      ctx.fill();
    };
    drawDot(result.path[0], "white");
    drawDot(result.path[result.path.length - 1], "white");
  }, [result, size]);

  return (
    <div ref={containerRef} className="h-full w-full">
      <canvas ref={canvasRef} />
    </div>
  );
}
