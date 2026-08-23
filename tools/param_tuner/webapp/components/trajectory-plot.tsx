"use client";

import { useEffect, useRef, useState } from "react";
import type { AnalysisEvent } from "@/lib/log-analysis";
import { TRAJECTORY_POS_OFFSET_X, type TrajectoryData, type TrajectoryPoint } from "@/lib/trajectory";

interface Props {
  data: TrajectoryData | null;
  showLeft45: boolean;
  showRight45: boolean;
  markers?: AnalysisEvent[];
  onPointClick: (point: TrajectoryPoint | null) => void;
}

const MARKER_STYLE: Record<AnalysisEvent["kind"], { color: string; shape: "x" | "diamond" | "circle" }> = {
  drop: { color: "#ff5555", shape: "x" },
  rise: { color: "#3ddc84", shape: "x" },
  "state-start": { color: "#f5a623", shape: "diamond" },
  "state-end": { color: "#5aa9ff", shape: "diamond" },
  trough: { color: "#c678f5", shape: "circle" },
  "trough-rise": { color: "#4ad4d4", shape: "circle" },
};

const PADDING = 24;
const MIN_ZOOM = 0.5;
const MAX_ZOOM = 40;
const DRAG_CLICK_THRESHOLD = 4; // px; below this a pointer down->up is treated as a click, not a pan

interface View {
  zoom: number;
  panX: number;
  panY: number;
}

const DEFAULT_VIEW: View = { zoom: 1, panX: 0, panY: 0 };

function clampZoom(z: number): number {
  return Math.min(MAX_ZOOM, Math.max(MIN_ZOOM, z));
}

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

export function TrajectoryPlot({ data, showLeft45, showRight45, markers, onPointClick }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });
  const [view, setView] = useState<View>(DEFAULT_VIEW);
  const viewRef = useRef(view);
  useEffect(() => {
    viewRef.current = view;
  }, [view]);
  const dragRef = useRef<{ startX: number; startY: number; startPanX: number; startPanY: number; moved: boolean } | null>(
    null
  );

  // New log selected -> forget the previous pan/zoom. Reset during render
  // (React's documented pattern for "adjust state when a prop changes")
  // rather than in an effect, to avoid an extra cascading render.
  const [prevData, setPrevData] = useState(data);
  if (data !== prevData) {
    setPrevData(data);
    setView(DEFAULT_VIEW);
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

  // Wheel-to-zoom, centered on the cursor. Attached as a native listener
  // (not React's onWheel) so preventDefault actually stops page scroll -
  // React registers wheel handlers as passive by default.
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const rect = canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      setView((v) => {
        const nextZoom = clampZoom(v.zoom * Math.exp(-e.deltaY * 0.0015));
        if (nextZoom === v.zoom) return v;
        // Keep the world point under the cursor fixed while zoom changes.
        const worldPx = (mx - v.panX) / v.zoom;
        const worldPy = (my - v.panY) / v.zoom;
        return { zoom: nextZoom, panX: mx - worldPx * nextZoom, panY: my - worldPy * nextZoom };
      });
    };
    canvas.addEventListener("wheel", onWheel, { passive: false });
    return () => canvas.removeEventListener("wheel", onWheel);
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

    // Pan/zoom applies on top of the fit-to-canvas base transform above, in
    // CSS-pixel space (kept separate from the dpr scaling just set). Stroke
    // widths and marker radii below are divided by zoom so they stay a
    // constant on-screen size instead of growing with the zoom level.
    const { zoom, panX, panY } = view;
    ctx.setTransform(dpr * zoom, 0, 0, dpr * zoom, dpr * panX, dpr * panY);
    const iz = 1 / zoom;

    ctx.strokeStyle = "rgba(255,60,60,0.25)";
    ctx.lineWidth = 1.5 * iz;
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
        ctx.arc(cx, cy, 1.8 * iz, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    const drawWall = (points: { x: number; y: number }[], square: boolean) => {
      ctx.fillStyle = "rgba(230,230,230,0.65)";
      ctx.strokeStyle = "rgba(255,255,255,0.5)";
      ctx.lineWidth = 0.5 * iz;
      for (const w of points) {
        const [cx, cy] = toCanvas(w.x, w.y);
        ctx.beginPath();
        if (square) {
          ctx.rect(cx - 2.5 * iz, cy - 2.5 * iz, 5 * iz, 5 * iz);
        } else {
          ctx.arc(cx, cy, 2.5 * iz, 0, Math.PI * 2);
        }
        ctx.fill();
        ctx.stroke();
      }
    };
    if (showLeft45) drawWall(data.leftWallPoints, false);
    if (showRight45) drawWall(data.rightWallPoints, true);

    if (markers && markers.length > 0) {
      const r = 6 * iz;
      for (const m of markers) {
        const style = MARKER_STYLE[m.kind];
        // "sensor"-anchored markers are already projected wall-contact
        // points (POS_OFFSET_X baked in by projectSensorPoint), same as
        // leftWallPoints/rightWallPoints above - don't offset them again.
        const [cx, cy] = m.anchored === "sensor" ? toCanvas(m.x, m.y) : toCanvas(m.x + TRAJECTORY_POS_OFFSET_X, m.y);
        ctx.strokeStyle = style.color;
        ctx.fillStyle = style.color;
        ctx.lineWidth = 2 * iz;
        ctx.beginPath();
        if (style.shape === "x") {
          ctx.moveTo(cx - r, cy - r);
          ctx.lineTo(cx + r, cy + r);
          ctx.moveTo(cx + r, cy - r);
          ctx.lineTo(cx - r, cy + r);
          ctx.stroke();
        } else if (style.shape === "diamond") {
          ctx.moveTo(cx, cy - r);
          ctx.lineTo(cx + r, cy);
          ctx.lineTo(cx, cy + r);
          ctx.lineTo(cx - r, cy);
          ctx.closePath();
          ctx.stroke();
        } else {
          ctx.arc(cx, cy, r * 0.75, 0, Math.PI * 2);
          ctx.stroke();
        }
      }
    }
  }, [data, size, showLeft45, showRight45, markers, view]);

  const handleClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (dragRef.current?.moved) return; // drag-to-pan, not a point pick
    if (!data || data.allPoints.length === 0) {
      onPointClick(null);
      return;
    }
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    const { zoom, panX, panY } = viewRef.current;
    const { toWorld } = makeTransform(data.worldBounds, size.width, size.height);
    const [wx, wy] = toWorld((cx - panX) / zoom, (cy - panY) / zoom);

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

  const handlePointerDown = (e: React.PointerEvent<HTMLCanvasElement>) => {
    (e.target as HTMLCanvasElement).setPointerCapture(e.pointerId);
    dragRef.current = { startX: e.clientX, startY: e.clientY, startPanX: view.panX, startPanY: view.panY, moved: false };
  };

  const handlePointerMove = (e: React.PointerEvent<HTMLCanvasElement>) => {
    const drag = dragRef.current;
    if (!drag) return;
    const dx = e.clientX - drag.startX;
    const dy = e.clientY - drag.startY;
    if (!drag.moved && Math.hypot(dx, dy) < DRAG_CLICK_THRESHOLD) return;
    drag.moved = true;
    setView((v) => ({ ...v, panX: drag.startPanX + dx, panY: drag.startPanY + dy }));
  };

  const handlePointerUp = () => {
    dragRef.current = null;
  };

  const resetView = () => setView(DEFAULT_VIEW);

  return (
    <div ref={containerRef} className="relative h-full w-full">
      <canvas
        ref={canvasRef}
        onClick={handleClick}
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={handlePointerUp}
        className={view.zoom > 1 || view.panX !== 0 || view.panY !== 0 ? "cursor-grab active:cursor-grabbing" : "cursor-crosshair"}
      />
      <div className="pointer-events-none absolute bottom-1.5 right-1.5 flex items-center gap-1.5 rounded bg-black/60 px-1.5 py-0.5 text-[10px] text-white/80">
        <span>{Math.round(view.zoom * 100)}%</span>
        {(view.zoom !== 1 || view.panX !== 0 || view.panY !== 0) && (
          <button type="button" className="pointer-events-auto underline hover:text-white" onClick={resetView}>
            リセット
          </button>
        )}
      </div>
    </div>
  );
}
