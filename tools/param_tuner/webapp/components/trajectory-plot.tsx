"use client";

import { useEffect, useRef, useState } from "react";
import { TRAJECTORY_POS_OFFSET_X, type TrajectoryData, type TrajectoryPoint } from "@/lib/trajectory";

interface Props {
  data: TrajectoryData | null;
  showLeft45: boolean;
  showRight45: boolean;
  onPointClick: (point: TrajectoryPoint | null) => void;
}

const PADDING = 24;

function makeTransform(bounds: TrajectoryData["worldBounds"], w: number, h: number) {
  const worldW = Math.max(bounds.xMax - bounds.xMin, 1e-6);
  const worldH = Math.max(bounds.yMax - bounds.yMin, 1e-6);
  const availW = Math.max(w - PADDING * 2, 1);
  const availH = Math.max(h - PADDING * 2, 1);
  const scale = Math.min(availW / worldW, availH / worldH);
  const offsetX = PADDING + (availW - worldW * scale) / 2;
  const offsetY = PADDING + (availH - worldH * scale) / 2;

  const toCanvas = (wx: number, wy: number): [number, number] => [
    offsetX + (wx - bounds.xMin) * scale,
    h - offsetY - (wy - bounds.yMin) * scale,
  ];
  const toWorld = (cx: number, cy: number): [number, number] => [
    bounds.xMin + (cx - offsetX) / scale,
    bounds.yMin + (h - offsetY - cy) / scale,
  ];
  return { toCanvas, toWorld, scale };
}

export function TrajectoryPlot({ data, showLeft45, showRight45, onPointClick }: Props) {
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
    ctx.fillStyle = "#0d1117";
    ctx.fillRect(0, 0, size.width, size.height);

    if (!data) return;
    const { toCanvas } = makeTransform(data.worldBounds, size.width, size.height);

    ctx.strokeStyle = "rgba(255,60,60,0.25)";
    ctx.lineWidth = 1.5;
    for (const line of data.gridLines) {
      const [x1, y1] = toCanvas(line.x1, line.y1);
      const [x2, y2] = toCanvas(line.x2, line.y2);
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    }

    for (const group of data.groups) {
      ctx.fillStyle = group.color;
      for (const p of group.points) {
        const [cx, cy] = toCanvas(p.x + TRAJECTORY_POS_OFFSET_X, p.y);
        ctx.beginPath();
        ctx.arc(cx, cy, 1.8, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    const drawWall = (points: { x: number; y: number }[], square: boolean) => {
      ctx.fillStyle = "rgba(230,230,230,0.65)";
      ctx.strokeStyle = "rgba(255,255,255,0.5)";
      ctx.lineWidth = 0.5;
      for (const w of points) {
        const [cx, cy] = toCanvas(w.x, w.y);
        ctx.beginPath();
        if (square) {
          ctx.rect(cx - 2.5, cy - 2.5, 5, 5);
        } else {
          ctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
        }
        ctx.fill();
        ctx.stroke();
      }
    };
    if (showLeft45) drawWall(data.leftWallPoints, false);
    if (showRight45) drawWall(data.rightWallPoints, true);
  }, [data, size, showLeft45, showRight45]);

  const handleClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!data || data.allPoints.length === 0) {
      onPointClick(null);
      return;
    }
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    const { toWorld } = makeTransform(data.worldBounds, size.width, size.height);
    const [wx, wy] = toWorld(cx, cy);

    let nearest: TrajectoryPoint | null = null;
    let bestDist = Infinity;
    for (const p of data.allPoints) {
      const dx = p.x + TRAJECTORY_POS_OFFSET_X - wx;
      const dy = p.y - wy;
      const d = dx * dx + dy * dy;
      if (d < bestDist) {
        bestDist = d;
        nearest = p;
      }
    }
    onPointClick(nearest);
  };

  return (
    <div ref={containerRef} className="h-full w-full">
      <canvas ref={canvasRef} onClick={handleClick} className="cursor-crosshair" />
    </div>
  );
}
