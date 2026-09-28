"use client";

import { useCallback, useEffect, useMemo, useRef, useState, type PointerEvent } from "react";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { ResizableHandle, ResizablePanel, ResizablePanelGroup } from "@/components/ui/resizable";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import type { MazeContent, MazeFileInfo, MazeGroup } from "@/lib/maze";
import {
  blankMaze,
  edgeKey,
  edgeState,
  isOuterEdge,
  mazeIndex,
  normalizeEdge,
  setEdge,
  WALL_E,
  WALL_N,
  WALL_S,
  WALL_W,
  type Cell,
  type Edge,
  type WallDir,
} from "@/lib/maze-shared";

const GROUP_LABEL: Record<MazeGroup, string> = {
  log: "受信ログ (maze_logs)",
  edit: "編集用 (profile/hf)",
  contest: "大会迷路 (maze_data)",
};
const GROUP_ORDER: MazeGroup[] = ["edit", "log", "contest"];
const HISTORY_LIMIT = 200;

// 描画はセル = 1 の座標系。画面の y は下向きなので北を上にするため反転する。
const MARGIN_L = 0.9;
const MARGIN_B = 0.9;
const MARGIN_T = 0.3;
const MARGIN_R = 0.3;
const WALL_WIDTH = 0.12;
const POST_SIZE = 0.16;

interface MazeDoc {
  id: string | null; // null = 未保存の新規
  name: string;
  size: number;
  goals: Cell[] | null; // ファイル自身のゴール(大会迷路のみ)
  editable: boolean; // 上書き保存できる(編集用)
}

interface SystemMaze {
  goals: Cell[] | null;
  mazeSize: number | null;
}

interface Props {
  active: boolean; // タブ表示中(キーボード操作はこのときだけ拾う)
  autoOpen: { id: string; nonce: number } | null;
  onAutoOpenHandled: () => void;
  refreshNonce: number; // 迷路の受信などで一覧を取り直す
  onFilesChanged: () => void; // 別名保存でプロファイル一覧が変わった
}

function formatDate(mtimeMs: number): string {
  const d = new Date(mtimeMs);
  const p = (n: number) => String(n).padStart(2, "0");
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`;
}

function defaultSaveName(doc: MazeDoc): string {
  if (doc.id === null) return "maze_new";
  const base = doc.name.replace(/\.(maze|yaml)$/, "");
  return doc.id.startsWith("log/") ? `log_${base}` : base;
}

function wallLetters(w: number): string {
  const s = [w & WALL_N ? "N" : "", w & WALL_E ? "E" : "", w & WALL_W ? "W" : "", w & WALL_S ? "S" : ""].join("");
  return s || "-";
}

export function MazePanel({ active, autoOpen, onAutoOpenHandled, refreshNonce, onFilesChanged }: Props) {
  const [files, setFiles] = useState<MazeFileInfo[]>([]);
  const [system, setSystem] = useState<SystemMaze>({ goals: null, mazeSize: null });
  const [doc, setDoc] = useState<MazeDoc | null>(null);
  const [walls, setWalls] = useState<number[]>([]);
  // 最後に読み込んだ/保存した内容。新規(未保存)は null で、常に未保存扱い。
  const [savedWalls, setSavedWalls] = useState<number[] | null>(null);
  const [past, setPast] = useState<number[][]>([]);
  const [future, setFuture] = useState<number[][]>([]);
  const [hover, setHover] = useState<{ cell: Cell; edge: Edge | null } | null>(null);
  const [saveAsName, setSaveAsName] = useState<string | null>(null);
  const [busy, setBusy] = useState<"save" | "send" | null>(null);

  const svgRef = useRef<SVGSVGElement>(null);
  // ドラッグ中の 1 ストローク: 最初の壁で決めた「置く/消す」と向き(横=N/縦=E)を、
  // 通った壁すべてに当てる。向きを固定しないと、格子線に沿ってなぞっても柱の
  // 近くで直交する壁を拾ってしまう。
  const strokeRef = useRef<{ present: boolean; dir: "N" | "E"; visited: Set<string> } | null>(null);

  const size = doc?.size ?? 0;
  const dirty = doc !== null && (savedWalls === null || walls.some((w, i) => w !== savedWalls[i]));

  const refreshFiles = useCallback(async () => {
    try {
      const res = await fetch("/api/maze?action=list");
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "一覧の取得に失敗しました");
      setFiles(data.files as MazeFileInfo[]);
      setSystem(data.system as SystemMaze);
    } catch (err) {
      toast.error(`迷路: ${(err as Error).message}`);
    }
  }, []);

  useEffect(() => {
    // fetch-on-mount(このプロジェクトは React Compiler を使っていない)。
    // eslint-disable-next-line react-hooks/set-state-in-effect
    void refreshFiles();
  }, [refreshFiles, refreshNonce]);

  const confirmDiscard = useCallback(
    () => !dirty || window.confirm("未保存の変更があります。破棄して切り替えますか?"),
    [dirty],
  );

  const resetEditState = (next: number[]) => {
    setWalls(next);
    setPast([]);
    setFuture([]);
    setHover(null);
    setSaveAsName(null);
  };

  const openFile = useCallback(
    async (id: string) => {
      try {
        const res = await fetch(`/api/maze?action=read&id=${encodeURIComponent(id)}`);
        const data = await res.json();
        if (!res.ok) throw new Error(data.error ?? "読み込みに失敗しました");
        const content = data as MazeContent;
        setDoc({
          id,
          name: id.slice(id.indexOf("/") + 1),
          size: content.size,
          goals: content.goals,
          editable: content.editable,
        });
        setSavedWalls(content.walls);
        resetEditState(content.walls);
      } catch (err) {
        toast.error(`${id}: ${(err as Error).message}`);
      }
    },
    [],
  );

  const selectFile = (id: string) => {
    if (doc?.id === id || !confirmDiscard()) return;
    void openFile(id);
  };

  const newMaze = (n: number) => {
    if (!confirmDiscard()) return;
    setDoc({ id: null, name: `新規 ${n}x${n}`, size: n, goals: null, editable: false });
    setSavedWalls(null);
    resetEditState(blankMaze(n));
  };

  useEffect(() => {
    if (!autoOpen) return;
    onAutoOpenHandled();
    if (doc?.id === autoOpen.id || !confirmDiscard()) return;
    // 外(トースト/プロファイル一覧)からの「開いて」要求。fetch 後に setState する。
    // eslint-disable-next-line react-hooks/set-state-in-effect
    void openFile(autoOpen.id);
    // autoOpen が変わったときだけ動かす。
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [autoOpen]);

  // ===== 編集 =====

  const commit = (next: number[]) => {
    setPast((p) => [...p.slice(-(HISTORY_LIMIT - 1)), walls]);
    setFuture([]);
    setWalls(next);
  };

  const undo = useCallback(() => {
    if (past.length === 0) return;
    setFuture((f) => [walls, ...f]);
    setWalls(past[past.length - 1]);
    setPast((p) => p.slice(0, -1));
  }, [past, walls]);

  const redo = useCallback(() => {
    if (future.length === 0) return;
    setPast((p) => [...p, walls]);
    setWalls(future[0]);
    setFuture((f) => f.slice(1));
  }, [future, walls]);

  // ポインタ位置 → 区画と、いちばん近い壁(拡張と同じく区画を対角線で 4 分割)。
  // lockDir を渡すとその向きの壁だけから選ぶ(ドラッグ中)。
  const locate = (
    ev: PointerEvent<SVGSVGElement>,
    lockDir?: "N" | "E",
  ): { cell: Cell; edge: Edge | null } | null => {
    const svg = svgRef.current;
    const ctm = svg?.getScreenCTM();
    if (!svg || !ctm || size === 0) return null;
    const pt = new DOMPoint(ev.clientX, ev.clientY).matrixTransform(ctm.inverse());
    const u = pt.x;
    const v = size - pt.y;
    if (u < 0 || v < 0 || u >= size || v >= size) return null;
    const cx = Math.floor(u);
    const cy = Math.floor(v);
    if (lockDir === "N") {
      const line = Math.round(v); // 横の格子線 y = line
      const edge: Edge | null = line >= 1 && line <= size - 1 ? { x: cx, y: line - 1, dir: "N" } : null;
      return { cell: [cx, cy], edge };
    }
    if (lockDir === "E") {
      const line = Math.round(u); // 縦の格子線 x = line
      const edge: Edge | null = line >= 1 && line <= size - 1 ? { x: line - 1, y: cy, dir: "E" } : null;
      return { cell: [cx, cy], edge };
    }
    const fx = u - cx;
    const fy = v - cy;
    const cand: [WallDir, number][] = [
      ["W", fx],
      ["E", 1 - fx],
      ["S", fy],
      ["N", 1 - fy],
    ];
    const dir = cand.reduce((a, b) => (b[1] < a[1] ? b : a))[0];
    const raw: Edge = { x: cx, y: cy, dir };
    return { cell: [cx, cy], edge: isOuterEdge(size, raw) ? null : normalizeEdge(raw) };
  };

  const onPointerDown = (ev: PointerEvent<SVGSVGElement>) => {
    if (ev.button !== 0 || !doc) return;
    const hit = locate(ev);
    if (!hit?.edge) return;
    ev.currentTarget.setPointerCapture(ev.pointerId);
    const present = edgeState(walls, size, hit.edge) !== "wall";
    // locate() は外周を返さないので、正規化後の向きは N か E。
    const dir = hit.edge.dir === "N" ? "N" : "E";
    strokeRef.current = { present, dir, visited: new Set([edgeKey(hit.edge)]) };
    commit(setEdge(walls, size, hit.edge, present));
  };

  const onPointerMove = (ev: PointerEvent<SVGSVGElement>) => {
    const stroke = strokeRef.current;
    const hit = locate(ev, stroke?.dir);
    setHover(hit);
    if (!stroke || !hit?.edge) return;
    const key = edgeKey(hit.edge);
    if (stroke.visited.has(key)) return;
    stroke.visited.add(key);
    // ストローク中は履歴を積まない(ストローク全体で 1 回の元に戻す)。
    setWalls((w) => setEdge(w, size, hit.edge!, stroke.present));
  };

  const endStroke = () => {
    strokeRef.current = null;
  };

  // ===== 保存・送信 =====

  const save = async () => {
    if (!doc) return;
    if (!doc.editable || doc.id === null) {
      setSaveAsName(defaultSaveName(doc));
      return;
    }
    setBusy("save");
    try {
      const res = await fetch("/api/maze", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: "save", id: doc.id, walls }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "保存に失敗しました");
      setSavedWalls(walls);
      toast.success(`${doc.name}: 保存しました`);
      void refreshFiles();
    } catch (err) {
      toast.error(`${doc.name}: ${(err as Error).message}`);
    } finally {
      setBusy(null);
    }
  };

  const saveAs = async () => {
    if (!doc || saveAsName === null) return;
    setBusy("save");
    try {
      const res = await fetch("/api/maze", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: "saveAs", name: saveAsName, walls }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "保存に失敗しました");
      const id = data.id as string;
      const name = id.slice(id.indexOf("/") + 1);
      // 大会迷路のゴールは .maze に書けないので、保存後は system.yaml のゴールで見る。
      setDoc({ ...doc, id, name, goals: null, editable: true });
      setSavedWalls(walls);
      setSaveAsName(null);
      toast.success(`profile/hf/${name} に保存しました`);
      void refreshFiles();
      onFilesChanged();
    } catch (err) {
      toast.error((err as Error).message);
    } finally {
      setBusy(null);
    }
  };

  const send = async () => {
    if (!doc) return;
    setBusy("send");
    try {
      const res = await fetch("/api/maze", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: "send", walls, label: doc.name }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "送信に失敗しました");
      toast.success(`${doc.name}: maze.txt へ送信しました(次にメインモードへ入ると読み込まれます)`);
    } catch (err) {
      toast.error(`${doc.name}: ${(err as Error).message}`);
    } finally {
      setBusy(null);
    }
  };

  // Ctrl+Z / Ctrl+Shift+Z / Ctrl+Y / Ctrl+S。タブ表示中だけ。入力欄の中は触らない。
  const saveRef = useRef(save);
  useEffect(() => {
    saveRef.current = save;
  });
  useEffect(() => {
    if (!active) return;
    const onKey = (e: KeyboardEvent) => {
      if (!(e.ctrlKey || e.metaKey)) return;
      const k = e.key.toLowerCase();
      const tag = (e.target as HTMLElement | null)?.tagName;
      if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") {
        // 保存名の入力中はブラウザの「ページを保存」だけ止める(確定は Enter)。
        if (k === "s") e.preventDefault();
        return;
      }
      if (k === "z" && !e.shiftKey) undo();
      else if ((k === "z" && e.shiftKey) || k === "y") redo();
      else if (k === "s") void saveRef.current();
      else return;
      e.preventDefault();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [active, undo, redo]);

  // ===== 描画 =====

  const goals = doc?.goals ?? system.goals ?? [];
  const goalSource = doc?.goals ? "ファイル" : system.goals ? "system.yaml" : null;

  const paths = useMemo(() => {
    let wall = "";
    let mismatch = "";
    let grid = "";
    let mismatchCount = 0;
    if (size === 0 || walls.length !== size * size) return { wall, mismatch, grid, mismatchCount };
    // 世界座標 (x, y) → 画面 (x, size - y)
    const seg = (x0: number, y0: number, x1: number, y1: number) => `M${x0} ${size - y0}L${x1} ${size - y1}`;
    const add = (e: Edge, d: string) => {
      const st = edgeState(walls, size, e);
      if (st === "wall") wall += d;
      else if (st === "mismatch") {
        mismatch += d;
        mismatchCount++;
      } else grid += d;
    };
    for (let x = 0; x < size; x++) {
      for (let y = 0; y < size; y++) {
        add({ x, y, dir: "N" }, seg(x, y + 1, x + 1, y + 1));
        add({ x, y, dir: "E" }, seg(x + 1, y, x + 1, y + 1));
        if (y === 0) add({ x, y, dir: "S" }, seg(x, 0, x + 1, 0));
        if (x === 0) add({ x, y, dir: "W" }, seg(0, y, 0, y + 1));
      }
    }
    return { wall, mismatch, grid, mismatchCount };
  }, [walls, size]);

  const posts = useMemo(() => {
    let d = "";
    const h = POST_SIZE / 2;
    for (let x = 0; x <= size; x++) {
      for (let y = 0; y <= size; y++) d += `M${x - h} ${y - h}h${POST_SIZE}v${POST_SIZE}h${-POST_SIZE}z`;
    }
    return d;
  }, [size]);

  // locate() は外周を返さないので、正規化後の壁は N か E だけ。
  const hoverEdge = hover?.edge ?? null;
  const hoverEdgePath = !hoverEdge
    ? null
    : hoverEdge.dir === "N"
      ? `M${hoverEdge.x} ${size - hoverEdge.y - 1}L${hoverEdge.x + 1} ${size - hoverEdge.y - 1}`
      : `M${hoverEdge.x + 1} ${size - hoverEdge.y}L${hoverEdge.x + 1} ${size - hoverEdge.y - 1}`;
  const hoverWouldRemove = hoverEdge ? edgeState(walls, size, hoverEdge) === "wall" : false;

  const labelSize = size > 20 ? 0.42 : 0.5;
  const grouped = GROUP_ORDER.map((g) => ({ group: g, items: files.filter((f) => f.group === g) }));
  const hoverWall = hover ? walls[mazeIndex(size, hover.cell[0], hover.cell[1])] : 0;

  return (
    <Card className="flex flex-1 flex-row overflow-hidden">
      <ResizablePanelGroup direction="horizontal" autoSaveId="param-console-maze">
        <ResizablePanel defaultSize={20} minSize={12} maxSize={40} className="min-w-0">
          <div className="flex h-full flex-col overflow-hidden border-r border-border">
            <div className="flex items-center justify-between px-2 py-1">
              <span className="text-sm font-medium">迷路</span>
              <div className="flex gap-1">
                <Button size="sm" variant="ghost" onClick={() => newMaze(16)} title="外周だけの 16x16 を新規作成">
                  新規16
                </Button>
                <Button size="sm" variant="ghost" onClick={() => newMaze(32)} title="外周だけの 32x32 を新規作成">
                  新規32
                </Button>
                <Button size="sm" variant="ghost" onClick={() => void refreshFiles()}>
                  更新
                </Button>
              </div>
            </div>
            <Separator />
            <ScrollArea className="min-h-0 flex-1">
              <div className="flex flex-col p-0.5">
                {grouped.map(({ group, items }) => (
                  <div key={group} className="flex flex-col">
                    <span className="px-1.5 pt-1.5 pb-0.5 text-xs font-medium text-muted-foreground">
                      {GROUP_LABEL[group]}
                    </span>
                    {items.length === 0 && <span className="px-1.5 py-0.5 text-xs text-muted-foreground">なし</span>}
                    {items.map((f) => {
                      const selected = doc?.id === f.id;
                      return (
                        <div
                          key={f.id}
                          role="button"
                          tabIndex={0}
                          onClick={() => selectFile(f.id)}
                          onKeyDown={(e) => {
                            if (e.key === "Enter") selectFile(f.id);
                          }}
                          className={`flex min-w-0 flex-col rounded px-1.5 py-0.5 text-left text-xs transition-colors ${
                            selected ? "bg-primary text-primary-foreground" : "cursor-pointer hover:bg-muted"
                          }`}
                        >
                          <span className="truncate font-medium">{f.name}</span>
                          {group === "log" && (
                            <span className={selected ? "text-primary-foreground/70" : "text-muted-foreground"}>
                              {formatDate(f.mtimeMs)}
                            </span>
                          )}
                        </div>
                      );
                    })}
                  </div>
                ))}
              </div>
            </ScrollArea>
          </div>
        </ResizablePanel>
        <ResizableHandle withHandle />
        <ResizablePanel defaultSize={80} minSize={30} className="min-w-0">
          <div className="flex h-full flex-col overflow-hidden">
            {/* 1 行: 状態・案内・主操作・保存だけ。 */}
            <div className="flex flex-wrap items-center gap-x-2 gap-y-0.5 px-1.5 py-0.5 text-xs">
              {doc ? (
                <>
                  <span className="font-medium">{doc.name}</span>
                  <span className="text-muted-foreground">
                    {doc.size}x{doc.size}
                  </span>
                  {dirty && <span className="text-accent-gold">● 未保存</span>}
                  {paths.mismatchCount > 0 && (
                    <span
                      className="text-accent-gold"
                      title="片側の区画にだけ壁が記録されている壁(金の点線)。クリックすると両側がそろう"
                    >
                      食い違い {paths.mismatchCount}
                    </span>
                  )}
                  <span className="text-muted-foreground" title={goals.map(([x, y]) => `(${x},${y})`).join(" ")}>
                    {goalSource ? `G: ${goalSource}` : "G: なし"}
                  </span>
                  <span className="min-w-0 truncate font-mono text-muted-foreground">
                    {hover
                      ? `(${hover.cell[0]}, ${hover.cell[1]}) ${wallLetters(hoverWall)} [${hoverWall}]` +
                        (hover.edge ? ` クリックで壁を${hoverWouldRemove ? "消す" : "置く"}` : "")
                      : "クリック/ドラッグで壁を置く・消す  Ctrl+Z で戻す"}
                  </span>
                </>
              ) : (
                <span className="text-muted-foreground">左の一覧から迷路を選ぶか、新規16/新規32 で作る</span>
              )}
              <div className="flex-1" />
              {doc && (
                <>
                  <Button size="xs" variant="ghost" disabled={past.length === 0} onClick={undo} title="元に戻す (Ctrl+Z)">
                    戻す
                  </Button>
                  <Button
                    size="xs"
                    variant="ghost"
                    disabled={future.length === 0}
                    onClick={redo}
                    title="やり直す (Ctrl+Shift+Z / Ctrl+Y)"
                  >
                    進む
                  </Button>
                  {saveAsName !== null ? (
                    <form
                      className="flex items-center gap-1"
                      onSubmit={(e) => {
                        e.preventDefault();
                        void saveAs();
                      }}
                    >
                      <span className="text-muted-foreground">profile/hf/</span>
                      <Input
                        autoFocus
                        className="h-6 w-40 text-xs"
                        value={saveAsName}
                        onChange={(e) => setSaveAsName(e.target.value)}
                        onKeyDown={(e) => {
                          if (e.key === "Escape") setSaveAsName(null);
                        }}
                      />
                      <span className="text-muted-foreground">.maze</span>
                      <Button size="xs" type="submit" disabled={busy !== null}>
                        {busy === "save" ? "保存中..." : "保存"}
                      </Button>
                      <Button size="xs" variant="ghost" type="button" onClick={() => setSaveAsName(null)}>
                        取消
                      </Button>
                    </form>
                  ) : doc.editable && doc.id !== null ? (
                    <>
                      <Button size="xs" variant="ghost" onClick={() => setSaveAsName(defaultSaveName(doc))}>
                        別名で保存
                      </Button>
                      <Button size="xs" disabled={busy !== null || !dirty} onClick={() => void save()} title="上書き保存 (Ctrl+S)">
                        {busy === "save" ? "保存中..." : "保存"}
                      </Button>
                    </>
                  ) : (
                    <Button
                      size="xs"
                      variant={dirty ? "default" : "outline"}
                      disabled={busy !== null}
                      onClick={() => setSaveAsName(defaultSaveName(doc))}
                      title="受信ログ・大会迷路は上書きしない。profile/hf/ に .maze として保存する (Ctrl+S)"
                    >
                      別名で保存
                    </Button>
                  )}
                  <Button
                    size="xs"
                    variant="secondary"
                    disabled={busy !== null}
                    onClick={() => void send()}
                    title="表示中の迷路を maze.txt として機体へ送る(全マス踏破済み扱い)。機体がボタン待ちのときに送り、次にメインモードへ入ると読み込まれる"
                  >
                    {busy === "send" ? "送信中..." : "機体へ送信"}
                  </Button>
                </>
              )}
            </div>
            <div className="min-h-0 flex-1 p-1">
              {doc && size > 0 && (
                <svg
                  ref={svgRef}
                  className="h-full w-full touch-none select-none"
                  viewBox={`${-MARGIN_L} ${-MARGIN_T} ${size + MARGIN_L + MARGIN_R} ${size + MARGIN_T + MARGIN_B}`}
                  style={{ cursor: hover?.edge ? "pointer" : "default" }}
                  onPointerDown={onPointerDown}
                  onPointerMove={onPointerMove}
                  onPointerUp={endStroke}
                  onPointerCancel={endStroke}
                  onPointerLeave={() => setHover(null)}
                >
                  <rect x={0} y={0} width={size} height={size} fill="oklch(0.12 0.015 250)" />
                  {goals
                    .filter(([x, y]) => x >= 0 && y >= 0 && x < size && y < size)
                    .map(([x, y]) => (
                      <rect
                        key={`g${x},${y}`}
                        x={x}
                        y={size - y - 1}
                        width={1}
                        height={1}
                        fill="var(--primary)"
                        fillOpacity={0.28}
                      />
                    ))}
                  <rect x={0} y={size - 1} width={1} height={1} fill="var(--accent-gold)" fillOpacity={0.25} />
                  {hover && (
                    <rect
                      x={hover.cell[0]}
                      y={size - hover.cell[1] - 1}
                      width={1}
                      height={1}
                      fill="var(--foreground)"
                      fillOpacity={0.06}
                    />
                  )}
                  <path d={paths.grid} stroke="var(--border)" strokeWidth={0.03} fill="none" />
                  <path
                    d={paths.wall}
                    stroke="var(--chart-2)"
                    strokeWidth={WALL_WIDTH}
                    strokeLinecap="square"
                    fill="none"
                  />
                  <path
                    d={paths.mismatch}
                    stroke="var(--accent-gold)"
                    strokeWidth={WALL_WIDTH}
                    strokeDasharray="0.15 0.12"
                    fill="none"
                  />
                  <path d={posts} fill="var(--muted-foreground)" />
                  {hoverEdgePath && (
                    <path
                      d={hoverEdgePath}
                      stroke={hoverWouldRemove ? "var(--foreground)" : "var(--primary)"}
                      strokeOpacity={0.75}
                      strokeWidth={WALL_WIDTH * 1.6}
                      strokeLinecap="round"
                      fill="none"
                      pointerEvents="none"
                    />
                  )}
                  <text
                    x={0.5}
                    y={size - 0.5}
                    fontSize={0.45}
                    fill="var(--accent-gold)"
                    textAnchor="middle"
                    dominantBaseline="central"
                    pointerEvents="none"
                  >
                    S
                  </text>
                  {Array.from({ length: size }, (_, i) => (
                    <g key={i} fill="var(--muted-foreground)" fontSize={labelSize} pointerEvents="none">
                      <text x={i + 0.5} y={size + 0.55} textAnchor="middle" dominantBaseline="central">
                        {i}
                      </text>
                      <text x={-0.2} y={size - i - 0.5} textAnchor="end" dominantBaseline="central">
                        {i}
                      </text>
                    </g>
                  ))}
                </svg>
              )}
            </div>
          </div>
        </ResizablePanel>
      </ResizablePanelGroup>
    </Card>
  );
}
