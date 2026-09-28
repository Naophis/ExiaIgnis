"use client";

import { useCallback, useEffect, useMemo, useRef, useState, type PointerEvent } from "react";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { ResizableHandle, ResizablePanel, ResizablePanelGroup } from "@/components/ui/resizable";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { MazePathPanel } from "@/components/maze-path-panel";
import { MazeSearchPanel } from "@/components/maze-search-panel";
import type { MazeContent, MazeFileInfo, MazeGroup } from "@/lib/maze";
import { buildPathGeometry, pathD, type Pt } from "@/lib/maze-path";
import { usePathSim } from "@/lib/use-path-sim";
import { useSearchSim } from "@/lib/use-search-sim";
import {
  blankMaze,
  detectGoalCandidates,
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
  log: "受信ログ・保存した迷路 (maze_logs)",
  profile: "過去の迷路 (profile)",
  contest: "過去の迷路 (maze_data)",
};
// 受信ログは探索のたびに増えるので最後(過去の迷路が埋もれて見つからなかった)
const GROUP_ORDER: MazeGroup[] = ["profile", "contest", "log"];
const HISTORY_LIMIT = 200;

// 描画はセル = 1 の座標系。画面の y は下向きなので北を上にするため反転する。
const MARGIN_L = 0.9;
const MARGIN_B = 0.9;
const MARGIN_T = 0.3;
const MARGIN_R = 0.3;
const WALL_WIDTH = 0.12;
const POST_SIZE = 0.16;
const PATH_COLOR = "oklch(0.82 0.13 230)";
// Direction の値(N=1 / E=2 / W=4 / S=8)→ 区画単位の向き
const DIR_VEC: Record<number, Pt> = { 1: [0, 1], 2: [1, 0], 4: [-1, 0], 8: [0, -1] };

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
}

// ゴールの上書き(system.yaml / ファイル以外のゴール)を迷路ごとに覚える。ブラウザだけの便利機能で、
// 読めなくても既定のゴールで動く。
const GOALS_KEY = "exia-maze-goals-v1";
function readGoalOverrides(): Record<string, Cell[]> {
  try {
    return JSON.parse(localStorage.getItem(GOALS_KEY) ?? "{}") as Record<string, Cell[]>;
  } catch {
    return {};
  }
}
function writeGoalOverride(id: string, goals: Cell[] | null) {
  try {
    const all = readGoalOverrides();
    if (goals) all[id] = goals;
    else delete all[id];
    localStorage.setItem(GOALS_KEY, JSON.stringify(all));
  } catch {
    // 保存できなくても動く
  }
}
const sameCell = (a: Cell, b: Cell) => a[0] === b[0] && a[1] === b[1];
function setGoalCell(list: Cell[], c: Cell, add: boolean): Cell[] {
  const has = list.some((g) => sameCell(g, c));
  if (add && !has) return [...list, c];
  if (!add && has) return list.filter((g) => !sameCell(g, c));
  return list;
}

function formatDate(mtimeMs: number): string {
  const d = new Date(mtimeMs);
  const p = (n: number) => String(n).padStart(2, "0");
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`;
}

function defaultSaveName(doc: MazeDoc): string {
  if (doc.id === null) return "maze_new";
  const base = doc.name.replace(/\.(maze|yaml)$/, "");
  // 受信した迷路は log_<日時>。保存した迷路をさらに別名で保存するときは log_log_ と重ねない
  if (doc.id.startsWith("log/")) return /^\d{8}_/.test(base) ? `log_${base}` : `${base}_2`;
  return base;
}

function wallLetters(w: number): string {
  const s = [w & WALL_N ? "N" : "", w & WALL_E ? "E" : "", w & WALL_W ? "W" : "", w & WALL_S ? "S" : ""].join("");
  return s || "-";
}

export function MazePanel({ active, autoOpen, onAutoOpenHandled, refreshNonce }: Props) {
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
  // 経路(MainTask::path_run() の経路生成を tools/path_sim で再現)
  // 右に出すシミュレーション: 経路(MainTask::path_run)/ 探索(SearchController::exec)
  const [sim, setSim] = useState<"none" | "path" | "search">("none");
  const pathOn = sim === "path";
  const searchOn = sim === "search";
  // 探索のステップ(null = 最後)と再生
  const [stepRaw, setStepRaw] = useState<number | null>(null);
  const [playing, setPlaying] = useState(false);
  const [playSpeed, setPlaySpeed] = useState(20);
  const [shownCandidate, setShownCandidate] = useState<number | null>(null);
  const [hoverSeg, setHoverSeg] = useState<number | null>(null);
  // ゴールの上書き(null = ファイルのゴール、無ければ system.yaml)とゴール編集
  const [goalOverride, setGoalOverride] = useState<Cell[] | null>(null);
  const [goalMode, setGoalMode] = useState(false);
  const goalStrokeRef = useRef<{ add: boolean; visited: Set<string> } | null>(null);

  const svgRef = useRef<SVGSVGElement>(null);
  // ドラッグ中の 1 ストローク: 最初の壁で決めた「置く/消す」と向き(横=N/縦=E)を、
  // 通った壁すべてに当てる。向きを固定しないと、格子線に沿ってなぞっても柱の
  // 近くで直交する壁を拾ってしまう。
  const strokeRef = useRef<{ present: boolean; dir: "N" | "E"; visited: Set<string> } | null>(null);

  const size = doc?.size ?? 0;
  // ゴールの自動検出(2×2 / 3×3、中に壁なし、外周に入口)。既定のゴールは
  // ファイルのゴール → 自動検出 → system.yaml の順。
  const goalCands = useMemo(
    () => (size > 0 && walls.length === size * size ? detectGoalCandidates(walls, size) : []),
    [walls, size],
  );
  const autoGoal = goalCands[0]?.reachable ? goalCands[0] : null;
  const defaultGoals = doc?.goals ?? autoGoal?.cells ?? system.goals ?? [];
  const goals = goalOverride ?? defaultGoals;
  const goalSource = goalOverride
    ? "手動"
    : doc?.goals
      ? "ファイル"
      : autoGoal
        ? "自動"
        : system.goals
          ? "system.yaml"
          : null;
  const defaultSource = doc?.goals ? "ファイルのゴール" : autoGoal ? "自動検出" : "system.yaml";
  const pathSim = usePathSim(pathOn && doc !== null, walls, goals);
  const searchSim = useSearchSim(searchOn && doc !== null, walls, goals);
  const searchResult = searchOn && searchSim.result?.ok ? searchSim.result : null;
  const nSteps = searchResult?.steps?.length ?? 0;
  const step = nSteps === 0 ? 0 : stepRaw === null ? nSteps - 1 : Math.min(stepRaw, nSteps - 1);
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
        setGoalOverride(readGoalOverrides()[id] ?? null);
        setGoalMode(false);
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
    setGoalOverride(null);
    setGoalMode(false);
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

  // ゴールの上書きを迷路ごとに覚える(開いた直後の読み戻しも同じ値を書くだけ)。
  useEffect(() => {
    if (doc?.id) writeGoalOverride(doc.id, goalOverride);
  }, [doc?.id, goalOverride]);

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
    if (goalMode) {
      // ゴール編集: 最初の区画で「足す/外す」を決め、ドラッグで通った区画すべてに当てる
      const hit = locate(ev);
      if (!hit) return;
      ev.currentTarget.setPointerCapture(ev.pointerId);
      const add = !goals.some((g) => sameCell(g, hit.cell));
      goalStrokeRef.current = { add, visited: new Set([hit.cell.join(",")]) };
      setGoalOverride(setGoalCell(goals, hit.cell, add));
      return;
    }
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
    if (goalMode) {
      const hit = locate(ev);
      setHover(hit ? { cell: hit.cell, edge: null } : null);
      const gs = goalStrokeRef.current;
      if (!gs || !hit || gs.visited.has(hit.cell.join(","))) return;
      gs.visited.add(hit.cell.join(","));
      setGoalOverride((prev) => setGoalCell(prev ?? defaultGoals, hit.cell, gs.add));
      return;
    }
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
    goalStrokeRef.current = null;
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
      // .maze にゴールは書けないので、過去の迷路のゴール(と手動のゴール)は上書きとして引き継ぐ
      setDoc({ ...doc, id, name, goals: null, editable: true });
      setGoalOverride(goalOverride ?? doc.goals);
      setSavedWalls(walls);
      setSaveAsName(null);
      toast.success(`maze_logs/${name} に保存しました`);
      void refreshFiles();
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

  // ゴール編集は Esc で終える
  useEffect(() => {
    if (!active || !goalMode) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") setGoalMode(false);
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [active, goalMode]);

  // 探索の再生: playSpeed 判断 / 秒で進め、最後で止める。
  useEffect(() => {
    if (!playing || !searchOn || nSteps === 0) return;
    const timer = setInterval(() => {
      setStepRaw((s) => Math.min(nSteps - 1, (s === null ? nSteps - 1 : s) + 1));
    }, 1000 / playSpeed);
    return () => clearInterval(timer);
  }, [playing, playSpeed, searchOn, nSteps]);
  useEffect(() => {
    // 最後まで進んだら止める
    // eslint-disable-next-line react-hooks/set-state-in-effect
    if (playing && step >= nSteps - 1) setPlaying(false);
  }, [playing, step, nSteps]);

  // 探索のステップ送り: ← → (Shift で 10)、Home / End。入力欄の中は触らない。
  useEffect(() => {
    if (!active || !searchOn || nSteps === 0) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.ctrlKey || e.metaKey || e.altKey) return;
      const tag = (e.target as HTMLElement | null)?.tagName;
      if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
      const n = e.shiftKey ? 10 : 1;
      if (e.key === "ArrowLeft") setStepRaw(Math.max(0, step - n));
      else if (e.key === "ArrowRight") setStepRaw(Math.min(nSteps - 1, step + n));
      else if (e.key === "Home") setStepRaw(0);
      else if (e.key === "End") setStepRaw(nSteps - 1);
      else return;
      setPlaying(false);
      e.preventDefault();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [active, searchOn, nSteps, step]);

  // ===== 描画 =====

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

  const pathResult = pathOn ? pathSim.result : null;
  const pathGeo = useMemo(
    () => (pathResult?.ok && pathResult.path_s && pathResult.path_t ? buildPathGeometry(pathResult.path_s, pathResult.path_t) : null),
    [pathResult],
  );
  const candGeo = useMemo(() => {
    const c = pathResult?.candidates?.find((x) => x.type === shownCandidate && x.result);
    return c ? buildPathGeometry(c.path_s, c.path_t) : null;
  }, [pathResult, shownCandidate]);
  const toScreen = (p: Pt): Pt => [p[0], size - p[1]];
  const hoverTurn = hoverSeg !== null ? pathGeo?.turns.find((t) => t.index === hoverSeg) : undefined;

  // 探索: そのステップでロボットが知っている壁・分かっている区画・軌跡・ロボット
  const searchView = useMemo(() => {
    if (!searchResult || nSteps === 0 || size === 0) return null;
    const M = searchResult.maze_size ?? size;
    const map = searchSim.maps[step];
    if (!map) return null;
    let known = "";
    let cells = "";
    let knownCells = 0;
    const seg = (x0: number, y0: number, x1: number, y1: number) => `M${x0} ${size - y0}L${x1} ${size - y1}`;
    for (let x = 0; x < size; x++) {
      for (let y = 0; y < size; y++) {
        const v = map[x + y * M];
        // 踏破フラグ(上位 4bit)が立っている向きの壁だけ「知っている」
        if (v & 0x10 && v & 0x01) known += seg(x, y + 1, x + 1, y + 1);
        if (v & 0x20 && v & 0x02) known += seg(x + 1, y, x + 1, y + 1);
        if (y === 0 && v & 0x80 && v & 0x08) known += seg(x, 0, x + 1, 0);
        if (x === 0 && v & 0x40 && v & 0x04) known += seg(0, y, 0, y + 1);
        if ((v & 0xf0) === 0xf0) {
          knownCells++;
          cells += `M${x} ${size - y - 1}h1v1h-1z`;
        }
      }
    }
    // 判断した位置 = その区画の入口の境界
    const at = (st: { f: [number, number, number] }): Pt => {
      const d = DIR_VEC[st.f[2]] ?? [0, 1];
      return [st.f[0] + 0.5 - d[0] * 0.5, st.f[1] + 0.5 - d[1] * 0.5];
    };
    const steps = searchResult.steps!;
    let trail = "";
    for (let i = 0; i <= step; i++) {
      const [x, y] = at(steps[i]);
      trail += `${i === 0 ? "M" : "L"}${x} ${size - y}`;
    }
    const cur = steps[step];
    const [rx, ry] = at(cur);
    const d = DIR_VEC[cur.f[2]] ?? [0, 1];
    // ロボット: 向きの三角形(画面座標は y が下向き)
    const tip: Pt = [rx + d[0] * 0.32, size - (ry + d[1] * 0.32)];
    const l: Pt = [rx - d[0] * 0.12 - d[1] * 0.2, size - (ry - d[1] * 0.12 + d[0] * 0.2)];
    const r: Pt = [rx - d[0] * 0.12 + d[1] * 0.2, size - (ry - d[1] * 0.12 - d[0] * 0.2)];
    const robot = `M${tip[0]} ${tip[1]}L${l[0]} ${l[1]}L${r[0]} ${r[1]}Z`;
    const next = step + 1 < nSteps ? at(steps[step + 1]) : null;
    return { known, cells, knownCells, trail, robot, from: [rx, size - ry] as Pt, next: next ? ([next[0], size - next[1]] as Pt) : null };
  }, [searchResult, searchSim.maps, step, nSteps, size]);

  const labelSize = size > 20 ? 0.42 : 0.5;
  const grouped = GROUP_ORDER.map((g) => ({ group: g, items: files.filter((f) => f.group === g) }));
  const hoverWall = hover ? walls[mazeIndex(size, hover.cell[0], hover.cell[1])] : 0;

  const mazeView = (
    <div className="h-full min-h-0 p-1">
      {doc && size > 0 && (
        <svg
          ref={svgRef}
          className="h-full w-full touch-none select-none"
          viewBox={`${-MARGIN_L} ${-MARGIN_T} ${size + MARGIN_L + MARGIN_R} ${size + MARGIN_T + MARGIN_B}`}
          style={{ cursor: (goalMode ? hover : hover?.edge) ? "pointer" : "default" }}
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
          {searchView && <path d={searchView.cells} fill="var(--primary)" fillOpacity={0.07} pointerEvents="none" />}
          {/* 探索中は正解の壁を薄く出し、ロボットが知っている壁を上に重ねる */}
          <path
            d={paths.wall}
            stroke={searchView ? "var(--muted-foreground)" : "var(--chart-2)"}
            strokeOpacity={searchView ? 0.35 : 1}
            strokeWidth={WALL_WIDTH}
            strokeLinecap="square"
            fill="none"
          />
          {searchView && (
            <path d={searchView.known} stroke="var(--chart-2)" strokeWidth={WALL_WIDTH} strokeLinecap="square" fill="none" pointerEvents="none" />
          )}
          <path
            d={paths.mismatch}
            stroke="var(--accent-gold)"
            strokeWidth={WALL_WIDTH}
            strokeDasharray="0.15 0.12"
            fill="none"
          />
          <path d={posts} fill="var(--muted-foreground)" />
          {searchView && (
            <g pointerEvents="none" strokeLinecap="round" strokeLinejoin="round">
              <path d={searchView.trail} fill="none" stroke={PATH_COLOR} strokeOpacity={0.55} strokeWidth={0.07} />
              {searchView.next && (
                <path
                  d={`M${searchView.from[0]} ${searchView.from[1]}L${searchView.next[0]} ${searchView.next[1]}`}
                  fill="none"
                  stroke="var(--accent-gold)"
                  strokeWidth={0.07}
                  strokeDasharray="0.12 0.1"
                />
              )}
              <path d={searchView.robot} fill="var(--accent-gold)" stroke="oklch(0.12 0.015 250)" strokeWidth={0.04} />
            </g>
          )}
          {pathGeo && (
            <g pointerEvents="none" fill="none" strokeLinecap="round" strokeLinejoin="round">
              <path
                d={pathD(pathGeo, toScreen)}
                stroke={PATH_COLOR}
                strokeWidth={0.09}
                strokeOpacity={candGeo ? 0.35 : 0.9}
              />
              {candGeo && (
                <path
                  d={pathD(candGeo, toScreen)}
                  stroke="var(--accent-gold)"
                  strokeWidth={0.09}
                  strokeDasharray="0.25 0.12"
                />
              )}
              {!candGeo &&
                pathGeo.turns.map((t) => {
                  const [x, y] = toScreen(t.at);
                  return <circle key={t.index} cx={x} cy={y} r={0.1} fill={PATH_COLOR} />;
                })}
              {hoverSeg !== null && (
                <path d={pathD(pathGeo, toScreen, hoverSeg)} stroke="var(--foreground)" strokeWidth={0.16} />
              )}
              {hoverTurn && (
                <text
                  x={toScreen(hoverTurn.at)[0] + 0.25}
                  y={toScreen(hoverTurn.at)[1] - 0.25}
                  fontSize={0.55}
                  fill="var(--foreground)"
                  stroke="oklch(0.12 0.015 250)"
                  strokeWidth={0.12}
                  paintOrder="stroke"
                >
                  {hoverTurn.index}: {hoverTurn.name} {hoverTurn.right ? "R" : "L"}
                </text>
              )}
            </g>
          )}
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
  );

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
                  <button
                    type="button"
                    onClick={() => setGoalMode((m) => !m)}
                    className={`rounded px-1 ${goalMode ? "bg-primary text-primary-foreground" : "text-muted-foreground ring-1 ring-border hover:bg-muted"}`}
                    title={`ゴール: ${goals.map(([x, y]) => `(${x},${y})`).join(" ") || "なし"}\nクリックでゴールを編集(経路・探索の計算に使う。機体の system.yaml は変えない)`}
                  >
                    G: {goalSource ?? "なし"}
                    {goals.length > 0 ? ` ${goals.length}区画` : ""}
                  </button>
                  {goalMode ? (
                    <>
                      <span className="text-primary">区画をクリック / ドラッグでゴールを追加・削除</span>
                      <Button size="xs" variant="ghost" onClick={() => setGoalOverride(null)} disabled={goalOverride === null}>
                        {defaultSource}に戻す
                      </Button>
                      {goalCands.slice(0, 3).map((c) => (
                        <Button
                          key={`${c.x},${c.y},${c.k}`}
                          size="xs"
                          variant="ghost"
                          onClick={() => setGoalOverride(c.cells)}
                          title={`自動検出の候補: 中に壁の無い ${c.k}×${c.k}、外周の入口 ${c.openings} か所${c.reachable ? "" : "、スタートから行けない"}`}
                        >
                          候補 ({c.x},{c.y}) {c.k}×{c.k}
                        </Button>
                      ))}
                      {system.goals && defaultSource !== "system.yaml" && (
                        <Button size="xs" variant="ghost" onClick={() => setGoalOverride(system.goals)}>
                          system.yaml
                        </Button>
                      )}
                      <Button size="xs" variant="ghost" onClick={() => setGoalOverride([])}>
                        クリア
                      </Button>
                      <Button size="xs" onClick={() => setGoalMode(false)} title="ゴール編集を終える (Esc)">
                        完了
                      </Button>
                    </>
                  ) : (
                  <span className="min-w-0 truncate font-mono text-muted-foreground">
                    {hover
                      ? `(${hover.cell[0]}, ${hover.cell[1]}) ${wallLetters(hoverWall)} [${hoverWall}]` +
                        (hover.edge ? ` クリックで壁を${hoverWouldRemove ? "消す" : "置く"}` : "")
                      : "クリック/ドラッグで壁を置く・消す  Ctrl+Z で戻す"}
                  </span>
                  )}
                </>
              ) : (
                <span className="text-muted-foreground">左の一覧から迷路を選ぶか、新規16/新規32 で作る</span>
              )}
              <div className="flex-1" />
              {doc && (
                <>
                  <Button
                    size="xs"
                    variant={pathOn ? "default" : "outline"}
                    onClick={() => {
                      setSim((v) => (v === "path" ? "none" : "path"));
                      setShownCandidate(null);
                    }}
                    title="機体の最短走行(MainTask::path_run)と同じ経路生成で経路とタイムを出す。全マス既知として扱う"
                  >
                    経路
                  </Button>
                  <Button
                    size="xs"
                    variant={searchOn ? "default" : "outline"}
                    onClick={() => {
                      setSim((v) => (v === "search" ? "none" : "search"));
                      setStepRaw(null);
                      setPlaying(false);
                    }}
                    title="機体の探索(メインモード 0、SearchController::exec)を足立法そのもので再現し、判断ごとの動きと探索時間を出す"
                  >
                    探索
                  </Button>
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
                      <span className="text-muted-foreground">maze_logs/</span>
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
                      title="受信した迷路・過去の迷路 (maze_data) は上書きしない。maze_logs/ に .maze として保存する (Ctrl+S)"
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
            {searchOn && doc ? (
              <ResizablePanelGroup direction="horizontal" autoSaveId="param-console-maze-search" className="min-h-0 flex-1">
                <ResizablePanel defaultSize={60} minSize={30} className="min-w-0">
                  {mazeView}
                </ResizablePanel>
                <ResizableHandle withHandle />
                <ResizablePanel defaultSize={40} minSize={20} className="min-w-0">
                  <MazeSearchPanel
                    result={searchSim.result}
                    busy={searchSim.busy}
                    step={step}
                    onStep={(i) => {
                      setStepRaw(i);
                      setPlaying(false);
                    }}
                    playing={playing}
                    onTogglePlay={() => {
                      if (!playing && step >= nSteps - 1) setStepRaw(0);
                      setPlaying((p) => !p);
                    }}
                    speed={playSpeed}
                    onSpeed={setPlaySpeed}
                    knownCells={searchView?.knownCells ?? 0}
                    totalCells={size * size}
                  />
                </ResizablePanel>
              </ResizablePanelGroup>
            ) : pathOn && doc ? (
              // 迷路は正方形で横が余るので、経路の表は右に置く(プロットタブの旋回表と同じ配置)。
              <ResizablePanelGroup direction="horizontal" autoSaveId="param-console-maze-path" className="min-h-0 flex-1">
                <ResizablePanel defaultSize={60} minSize={30} className="min-w-0">
                  {mazeView}
                </ResizablePanel>
                <ResizableHandle withHandle />
                <ResizablePanel defaultSize={40} minSize={20} className="min-w-0">
                  <MazePathPanel
                    options={pathSim.options}
                    exec={pathSim.exec}
                    onExecChange={pathSim.setExec}
                    direction={pathSim.direction}
                    onDirectionChange={(d) => {
                      pathSim.setDirection(d);
                      setShownCandidate(null);
                    }}
                    busy={pathSim.busy}
                    result={pathResult}
                    geometry={pathGeo}
                    shownCandidate={shownCandidate}
                    onShowCandidate={setShownCandidate}
                    hoverSeg={hoverSeg}
                    onHoverSeg={setHoverSeg}
                  />
                </ResizablePanel>
              </ResizablePanelGroup>
            ) : (
              <div className="min-h-0 flex-1">{mazeView}</div>
            )}
          </div>
        </ResizablePanel>
      </ResizablePanelGroup>
    </Card>
  );
}
