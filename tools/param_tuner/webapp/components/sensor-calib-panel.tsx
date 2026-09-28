"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { ResizableHandle, ResizablePanel, ResizablePanelGroup } from "@/components/ui/resizable";
import {
  CHANNELS,
  GROUP_LABEL,
  GROUP_ORDER,
  PRESET_POSITIONS,
  SWEEP_DEFAULT_D0,
  SWEEP_TARGET_END,
  TARGETS,
  channelIndex,
  distToRaw,
  estimateSweepD0,
  fitGain,
  gainToDist,
  groupChannels,
  mean,
  pairedKey,
  parseDump2Line,
  parseSweepLog,
  parseSweepStateLine,
  rowServes,
  sampleDist,
  std,
  targetPoints,
  type CalibGroup,
  type CalibRow,
  type Channel,
  type FitPoint,
  type FitResult,
  type Gain,
  type SweepD0,
  type SweepFwState,
  type TargetDef,
  type TargetKey,
} from "@/lib/sensor-calib-shared";

// 記録途中の位置表は per-viewer の作業状態なので localStorage に置く
// (リロードで 15 分の測定が消えるのを防ぐ)。読めなくても空で動く。
const STORAGE_KEY = "exia-sensor-calib-v2";
const RX_ALIVE_MS = 1000;
const UNSTABLE_REL = 0.03; // σ/平均 がこれを超えたら「動いた」とみなして警告

// 範囲の既定値を変えたら上げる(古い既定値のまま保存された範囲を捨てるため)。
const RANGES_VERSION = 2;
// 位置表の既定を変えたら上げる(保存済みの表は migrateRows で直す)。
const ROWS_VERSION = 2;

interface Stored {
  rows: CalibRow[];
  ranges: Partial<Record<TargetKey, [number, number]>>;
  rangesVersion?: number;
  rowsVersion?: number;
  linkLR?: boolean;
  autoD0?: boolean;
  fitStatic?: boolean;
  nSamples: number;
  sweepD0?: number;
}

function loadStored(): Stored | null {
  try {
    const s = localStorage.getItem(STORAGE_KEY);
    return s ? (JSON.parse(s) as Stored) : null;
  } catch {
    return null;
  }
}

function newId(): string {
  return Math.random().toString(36).slice(2, 10);
}

const isSweep = (r: CalibRow) => !!r.offsets;
const isEmptySweep = (r: CalibRow) => !!r.offsets && r.samples.length === 0;

// 静止点を壁・距離の順、スイープはその後ろ(取り込んだ順)
function sortRows(rows: CalibRow[]): CalibRow[] {
  return rows
    .map((r, i) => [r, i] as const)
    .sort(
      ([a, ia], [b, ib]) =>
        Number(isSweep(a)) - Number(isSweep(b)) ||
        (isSweep(a) ? ia - ib : GROUP_ORDER.indexOf(a.group) - GROUP_ORDER.indexOf(b.group) || a.dist - b.dist),
    )
    .map(([r]) => r);
}

function emptySweepRow(d0: number): CalibRow {
  return { id: newId(), group: "f", dist: d0, use: true, samples: [], offsets: [] };
}

// 機体(テストモード28)の進行状況を、どれだけの間「いまの状態」とみなすか [ms]。
// ready/place/done は 1 秒ごとに出る。hand は手をかざすまで何も出ない。
const FW_STATE_TTL: Record<SweepFwState, number> = {
  ready: 3000,
  wired: 3000,
  place: 3000,
  near: 3000,
  hand: 120000,
  running: 10000,
  done: 3000,
  dumped: 3000,
};

// 既定の位置表: 横壁は置いて記録、前壁は機体が自分で走って測る(スイープ)。
// 前壁を 9 位置に置き直す従来の手順は「前壁の取り方」で選ぶ。
function presetRows(): CalibRow[] {
  const sides = PRESET_POSITIONS.filter((p) => p.group !== "f").map(
    (p): CalibRow => ({ id: newId(), group: p.group, dist: p.dist, use: true, samples: [] }),
  );
  return [...sides, emptySweepRow(SWEEP_DEFAULT_D0)];
}

// 既定を「前壁 9 位置の置き直し」から「スイープ」へ変える前に保存された表を直す。
// 記録済みの行は残し、まだ取っていない前壁の置き直しの行をスイープの行に替える。
function migrateRows(rows: CalibRow[]): CalibRow[] {
  const out = rows.filter((r) => r.samples.length > 0 || isSweep(r) || r.group !== "f");
  if (!out.some(isSweep)) out.push(emptySweepRow(SWEEP_DEFAULT_D0));
  return sortRows(out);
}

// 同じログを二重に入れないための指紋(ケーブルを挿し直すと機体は同じログを送り直す)
function sweepSignature(samples: number[][]): string {
  const n = samples.length;
  if (!n) return "";
  let sum = 0;
  for (const s of samples) sum += s[0] + s[8];
  const pick = [0, n >> 3, n >> 2, n >> 1, (3 * n) >> 2, n - 1];
  return `${n}|${sum}|${pick.map((i) => `${samples[i][0]}:${samples[i][8]}`).join("|")}`;
}

function defaultRanges(): Record<TargetKey, [number, number]> {
  return Object.fromEntries(TARGETS.map((t) => [t.key, t.range])) as Record<TargetKey, [number, number]>;
}

// 機体の置き方を言葉にする(案内と記録中の表示用)
function placeLabel(r: CalibRow): string {
  const wall = r.group === "f" ? "前壁" : r.group === "l" ? "左壁" : r.group === "r" ? "右壁" : "横壁";
  return `${wall}から ${r.dist}mm `;
}

// 行の安定度: その置き方で見るチャンネルの σ/平均 の最大。
// スイープ行は距離が動いているので対象外。
function rowInstability(r: CalibRow): number {
  if (r.offsets || r.samples.length < 2) return NaN;
  let worst = 0;
  for (const ch of groupChannels(r.group)) {
    const ci = channelIndex(ch);
    const xs = r.samples.map((s) => s[ci]);
    const m = mean(xs);
    if (m > 0) worst = Math.max(worst, std(xs) / m);
  }
  return worst;
}

function offsetSpan(offsets: number[]): [number, number] {
  let lo = Infinity;
  let hi = -Infinity;
  for (const o of offsets) {
    if (o < lo) lo = o;
    if (o > hi) hi = o;
  }
  return [lo, hi];
}

// スイープ行の状態表示。label は表の 1 セルに収まる短さにし、内訳は title へ。
function sweepInfo(
  r: CalibRow,
  est: Partial<Record<Channel, SweepD0>> | undefined,
  autoD0: boolean,
): { label: string; title: string; warn: boolean } {
  if (!r.samples.length) {
    return {
      label: "未走行",
      title:
        "機体が自分で前壁へ走りながら測る行です(人が置き直す位置はありません)。走行はケーブルなしで行います。テストモード28 でケーブルを抜く → 機体のボタン → スタート位置に置いて前に手をかざす → 止まったらケーブルをつなぐ(ログは自動で届きます)",
      warn: false,
    };
  }
  const lines: string[] = [];
  let warn = false;
  const l = est?.L90;
  const rr = est?.R90;
  const [lo, hi] = offsetSpan(r.offsets ?? []);
  let label = "寸法";
  if (autoD0 && (l || rr)) {
    label = "静止点";
    lines.push("距離の基準: 静止点に合わせた開始位置");
  } else {
    lines.push(`距離の基準: 迷路の寸法(開始位置 ${r.dist}mm)`);
  }
  lines.push(`測った範囲: 前壁から ${(r.dist + lo).toFixed(0)}〜${(r.dist + hi).toFixed(0)}mm(走行 ${(hi - lo).toFixed(0)}mm)`);
  // 移動距離の精度の確認: 静止点が読む位置と、寸法で決めた距離との差
  for (const [name, e] of [
    ["L90", l],
    ["R90", rr],
  ] as const) {
    if (!e) continue;
    lines.push(
      `${name} 静止点との差(スイープの距離 − 静止点の距離): ${e.anchors
        .map((a) => `${a.dist}mm で ${fmtSigned(r.dist - a.d0)}`)
        .join(" / ")}`,
    );
    const worst = Math.max(...e.anchors.map((a) => Math.abs(r.dist - a.d0)));
    if (worst > ANCHOR_DIFF_WARN_MM) warn = true;
  }
  if (l || rr) {
    lines.push("  差が前壁に近いほど大きいなら走行距離(タイヤ径)、どこでも同じなら開始位置のずれ");
    if (l && rr) {
      const d = l.d0 - rr.d0;
      lines.push(`L90 と R90 の差 ${fmtSigned(d)}mm(傾きの目安)`);
      if (Math.abs(d) > D0_LR_WARN_MM) warn = true;
    }
  } else {
    lines.push("前壁の静止点を記録すると、寸法で決めた距離との差をここに出します(移動距離の精度の確認)");
  }
  if (r.pose && r.pose.headingDriftDeg !== undefined) {
    const p = r.pose;
    lines.push(
      `姿勢(測った範囲): 向きの変化 ${p.headingDriftDeg.toFixed(2)}° / 横ずれ ${
        p.latOffsetMm === null || p.latOffsetMm === undefined
          ? "不明(両側の壁が見えない)"
          : `${fmtSigned(p.latOffsetMm)}mm(左寄りが正)`
      } / 壁制御 ${p.wallCtrl ? "あり" : "なし"} / 最高 ${p.vMax.toFixed(0)}mm/s`,
    );
    if (p.headingDriftDeg > HEADING_WARN_DEG) {
      warn = true;
      lines.push(`  測っている間に向きが ${HEADING_WARN_DEG}° 以上変わっています。取り直しを推奨`);
    }
  }
  if (r.source) lines.push(`ログ: ${r.source}`);
  return { label, title: lines.join("\n"), warn };
}

const f1 = (v: number) => (Number.isFinite(v) ? v.toFixed(1) : "-");
const f2 = (v: number) => (Number.isFinite(v) ? v.toFixed(2) : "-");
const fg = (g?: Gain) => (g ? `${g[0].toFixed(1)}, ${g[1].toFixed(2)}` : "-");

interface TargetFit {
  def: TargetDef;
  range: [number, number];
  pts: FitPoint[];
  fit: FitResult | null;
  delta: number; // 基準距離での新旧差 [mm]
  sweepOnly: boolean; // スイープの点だけでフィットした(静止点はアンカー)
}

const SWEEP_CHANNELS: Channel[] = ["L90", "R90"];
const SWEEP_FIT_MIN_POINTS = 30;
const HEADING_WARN_DEG = 1.5;
const D0_LR_WARN_MM = 1.5;
const ANCHOR_DIFF_WARN_MM = 3.0;
const fmtSigned = (v: number) => `${v >= 0 ? "+" : ""}${v.toFixed(1)}`;

// connected: param console が機体とつながっているか。スイープはケーブルなしで
// 走らせるので、つながっていない間の案内を出すのに使う。
export function SensorCalibPanel({ connected }: { connected: boolean }) {
  const [rows, setRows] = useState<CalibRow[]>(() => presetRows());
  const [ranges, setRanges] = useState<Record<TargetKey, [number, number]>>(() => defaultRanges());
  const [nSamples, setNSamples] = useState(50);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [recordingId, setRecordingId] = useState<string | null>(null);
  const [latest, setLatest] = useState<number[] | null>(null);
  const [lastRxAt, setLastRxAt] = useState(0);
  const [now, setNow] = useState(0);
  const [curGains, setCurGains] = useState<Partial<Record<TargetKey, Gain>>>({});
  const [fitMode, setFitMode] = useState<"ab" | "b">("ab");
  const [activeKey, setActiveKey] = useState<TargetKey>("L45");
  // 反映対象はフィットできたキー全部から、手で外したものを除く。
  const [excluded, setExcluded] = useState<Set<TargetKey>>(new Set());
  const [dirs, setDirs] = useState<string[]>([]);
  const [loadDir, setLoadDir] = useState("");
  // 下位フォルダも読むか。csv/ 直下を読むと 1st/2nd まで混ざるので既定はオフ
  // (今の far は「直下の f_84〜96 + 2nd の f_126〜138」で作られていた)。
  const [loadRecursive, setLoadRecursive] = useState(false);
  const [busy, setBusy] = useState<string | null>(null);
  const [showDetail, setShowDetail] = useState(false);
  const [addGroup, setAddGroup] = useState<CalibGroup>("f");
  const [addDist, setAddDist] = useState("");
  // 前壁スイープ(テストモード28)の開始位置 D0 [mm] と、手動取り込み用のログ一覧
  const [sweepD0, setSweepD0] = useState(SWEEP_DEFAULT_D0);
  // 機体(テストモード28)が最後に出した進行状況
  const [fw, setFw] = useState<{ state: SweepFwState; at: number } | null>(null);
  // ファームが知らせた開始位置の前壁距離(迷路の寸法から)。取り込むスイープに使う
  const fwD0Ref = useRef<number | null>(null);
  // このタブを開いてからテストモード28 の機体を一度でも見たか
  const [seenMode28, setSeenMode28] = useState(false);
  const seenMode28Ref = useRef(false);
  // 範囲の左右連動(L90_near を変えたら R90_near も同じにする)
  const [linkLR, setLinkLR] = useState(true);
  // スイープの開始位置は迷路の寸法で決める(既定)。オンにすると、既知距離に置いた
  // 静止点へ合わせて L90/R90 別々に求め直す(lib/sensor-calib-shared.ts
  // estimateSweepD0)。オフでも、静止点があれば寸法との差を行のホバーに出す。
  const [autoD0, setAutoD0] = useState(false);
  // スイープがある範囲でも静止点をフィットに混ぜるか。既定は混ぜない
  // (形はスイープ、絶対位置は静止点、と役割を分ける)。
  const [fitStatic, setFitStatic] = useState(false);
  const [logFiles, setLogFiles] = useState<string[]>([]);
  // 保存済みの表を読み終えたか。ref ではなく state にして、読み込みと同じ描画の
  // 中では保存しない(開発モードは effect を 2 回走らせるので、ref だと 1 回目の
  // 保存が既定の表で上書きし、2 回目の読み込みがそれを拾って記録が消えていた)。
  const [hydrated, setHydrated] = useState(false);
  const sweepD0Ref = useRef(sweepD0);
  useEffect(() => {
    sweepD0Ref.current = sweepD0;
  }, [sweepD0]);

  // 記録中の対象と残り数は SSE ハンドラから同期的に見たいので ref に持つ。
  const recRef = useRef<{ id: string; left: number } | null>(null);
  const rowsRef = useRef(rows);
  useEffect(() => {
    rowsRef.current = rows;
  }, [rows]);

  useEffect(() => {
    const s = loadStored();
    // eslint-disable-next-line react-hooks/set-state-in-effect
    if (s?.rows?.length) setRows(s.rowsVersion === ROWS_VERSION ? s.rows : migrateRows(s.rows));
    if (s?.ranges && s.rangesVersion === RANGES_VERSION) setRanges((prev) => ({ ...prev, ...s.ranges }));
    if (s?.linkLR !== undefined) setLinkLR(s.linkLR);
    if (s?.autoD0 !== undefined) setAutoD0(s.autoD0);
    if (s?.fitStatic !== undefined) setFitStatic(s.fitStatic);
    if (s?.nSamples) setNSamples(s.nSamples);
    if (s?.sweepD0) setSweepD0(s.sweepD0);
    setHydrated(true);
  }, []);

  useEffect(() => {
    if (!hydrated) return;
    try {
      localStorage.setItem(
        STORAGE_KEY,
        JSON.stringify({
          rows,
          ranges,
          rangesVersion: RANGES_VERSION,
          rowsVersion: ROWS_VERSION,
          linkLR,
          autoD0,
          fitStatic,
          nSamples,
          sweepD0,
        } satisfies Stored),
      );
    } catch {
      // 容量超過などは無視(作業は続けられる)
    }
  }, [hydrated, rows, ranges, nSamples, sweepD0, linkLR, autoD0, fitStatic]);

  const refreshGains = useCallback(async () => {
    const res = await fetch("/api/sensor-calib?action=gains");
    const data = await res.json();
    if (res.ok) setCurGains(data.gains);
  }, []);

  const refreshDirs = useCallback(async () => {
    const res = await fetch("/api/sensor-calib?action=dirs");
    const data = await res.json();
    if (res.ok) setDirs(data.dirs);
  }, []);

  const refreshLogFiles = useCallback(async () => {
    const res = await fetch("/api/logs");
    const data = await res.json();
    if (res.ok) setLogFiles((data.files as { name: string }[]).map((f) => f.name).filter((n) => n !== "latest.csv"));
  }, []);

  // ログを読んでスイープ行として足す。quiet(=保存イベントからの自動取り込み)では
  // テストモード28のログ(後退以外のモーションが無い)だけを黙って拾う。
  const importSweep = useCallback(async (name: string, quiet: boolean): Promise<boolean> => {
    if (rowsRef.current.some((r) => r.source === name)) {
      if (!quiet) toast.error(`${name}: 取り込み済みです`);
      return false;
    }
    const res = await fetch(`/api/logs/content?name=${encodeURIComponent(name)}`);
    if (!res.ok) {
      if (!quiet) toast.error(`${name}: 読めませんでした`);
      return false;
    }
    const sweep = parseSweepLog(await res.text());
    if (!sweep || (quiet && !sweep.looksLikeSweep)) {
      if (!quiet) {
        toast.error(`${name}: 直進/後退の区間がありません。テストモード28のログですか?`);
      } else if (sweep && seenMode28Ref.current) {
        // 校正モードの機体から届いたログなのに使えない。理由を出す
        toast.error(`${name}: スイープのログとして使えません — ${sweep.reject}`, { duration: 30000 });
      }
      return false;
    }
    // 取り込み待ちの間に同じログが別経路で入っていたら捨てる
    if (rowsRef.current.some((r) => r.source === name)) return false;
    // 機体が同じログを送り直したもの(ケーブルの挿し直し)は入れない
    const sig = sweepSignature(sweep.samples);
    if (rowsRef.current.some((r) => r.offsets && sweepSignature(r.samples) === sig)) {
      toast.info(`${name}: 取り込み済みのスイープと同じログです`);
      return false;
    }
    // 開始位置: 保存イベントからの取り込みは機体が知らせた値、手で選んだ過去ログは入力欄の値
    const startD0 = quiet && fwD0Ref.current !== null ? fwD0Ref.current : sweepD0Ref.current;
    const row: CalibRow = {
      id: newId(),
      group: "f",
      dist: startD0,
      use: true,
      samples: sweep.samples,
      offsets: sweep.offsets,
      source: name,
      pose: sweep.pose,
    };
    // 位置表に「まだ走っていないスイープ」の行があればそこへ入れる
    const slot = rowsRef.current.findIndex(isEmptySweep);
    const put = (list: CalibRow[]) => {
      const i = list.findIndex(isEmptySweep);
      if (i < 0) return [...list, row];
      const next = [...list];
      next[i] = { ...row, id: list[i].id };
      return next;
    };
    const placed = slot >= 0 ? { ...row, id: rowsRef.current[slot].id } : row;
    rowsRef.current = put(rowsRef.current);
    setRows(put);
    setSelectedId(null);
    if (!sweep.looksLikeSweep) toast.warning(`${name}: ${sweep.reject}。そのまま取り込みました`);

    const [lo, hi] = offsetSpan(sweep.offsets);
    const near = placed.dist + lo;
    toast.success(
      `前壁スイープを取り込みました: 前壁から ${near.toFixed(0)}〜${(placed.dist + hi).toFixed(0)}mm(${sweep.samples.length}点、開始位置 ${placed.dist}mm)`,
      { duration: 8000 },
    );
    if (near > SWEEP_TARGET_END + 5) {
      toast.warning(
        `スイープの終点が前壁から ${near.toFixed(0)}mm です。${SWEEP_TARGET_END}mm 付近まで届いていないので、42〜${near.toFixed(0)}mm は測れていません`,
        { duration: 12000 },
      );
    }
    return true;
  }, []);

  useEffect(() => {
    // eslint-disable-next-line react-hooks/set-state-in-effect
    void refreshGains();
    void refreshDirs();
    void refreshLogFiles();
    const t = setInterval(() => setNow(Date.now()), 500);
    return () => clearInterval(t);
  }, [refreshGains, refreshDirs, refreshLogFiles]);

  const finishRecording = useCallback((id: string, aborted: boolean) => {
    recRef.current = null;
    setRecordingId(null);
    if (aborted) return;
    const list = rowsRef.current;
    const row = list.find((r) => r.id === id);
    toast.success(`${row ? `${GROUP_LABEL[row.group]} ${row.dist}mm` : ""} 記録完了`);
    // 次の未記録の行へ進める
    const idx = list.findIndex((r) => r.id === id);
    const next = [...list.slice(idx + 1), ...list.slice(0, idx)].find((r) => r.samples.length === 0);
    // 全部取り終えたら選択を外す(Space で最後の行を上書きしないように)
    setSelectedId(next ? next.id : null);
  }, []);

  useEffect(() => {
    const es = new EventSource("/api/stream");
    es.addEventListener("log", (e) => {
      const { line } = JSON.parse((e as MessageEvent).data) as { line: string };
      const st = parseSweepStateLine(line);
      if (st) {
        if (st.d0 !== undefined && Number.isFinite(st.d0)) fwD0Ref.current = st.d0;
        setFw({ state: st.state, at: Date.now() });
        setSeenMode28(true);
        seenMode28Ref.current = true;
        return;
      }
      const vals = parseDump2Line(line);
      if (!vals) return;
      setLatest(vals);
      setLastRxAt(Date.now());
      const rec = recRef.current;
      if (!rec) return;
      rec.left -= 1;
      const id = rec.id;
      setRows((prev) => prev.map((r) => (r.id === id ? { ...r, samples: [...r.samples, vals] } : r)));
      if (rec.left <= 0) finishRecording(id, false);
    });
    // テストモード28のダンプが保存されたら自動で取り込む(スイープ以外のログは黙って無視)
    es.addEventListener("saved", (e) => {
      const { type, file } = JSON.parse((e as MessageEvent).data) as { type: string; file: string };
      if (type !== "csv") return;
      void importSweep(file, true);
      void refreshLogFiles();
    });
    return () => es.close();
  }, [finishRecording, importSweep, refreshLogFiles]);

  const rxAlive = now - lastRxAt < RX_ALIVE_MS;
  const rxAliveRef = useRef(false);
  useEffect(() => {
    rxAliveRef.current = rxAlive;
  }, [rxAlive]);

  // 記録する位置。行を選んでいなければ、未記録の最初の位置。
  // (以前は行をクリックで選ぶまで記録もSpaceも無反応で、理由も出ていなかった)
  // 次にやる行。選んでいなければ、まだ取っていない最初の行(スイープの行も含む)。
  const todoRow = useMemo(
    () => rows.find((r) => r.id === selectedId) ?? rows.find((r) => r.samples.length === 0) ?? null,
    [rows, selectedId],
  );
  const targetRow = todoRow && !isSweep(todoRow) ? todoRow : null; // 置いて記録する行
  const sweepTodo = todoRow && isSweep(todoRow) ? todoRow : null; // 機体が走って測る行

  const startRecording = useCallback(
    (id: string) => {
      if (recRef.current) return;
      setRows((prev) => prev.map((r) => (r.id === id ? { ...r, samples: [] } : r)));
      recRef.current = { id, left: nSamples };
      setRecordingId(id);
      setSelectedId(id);
    },
    [nSamples],
  );

  const toggleRecording = useCallback(() => {
    if (recRef.current) {
      finishRecording(recRef.current.id, true);
      return;
    }
    if (!targetRow) {
      if (sweepTodo) toast.info("スイープはケーブルを抜いて、機体のボタンで始めます(案内を見てください)");
      return;
    }
    if (!rxAliveRef.current) {
      toast.error("機体の生値が届いていません。テストモード 15(または 14)で起動してください");
      return;
    }
    startRecording(targetRow.id);
  }, [targetRow, sweepTodo, startRecording, finishRecording]);

  // 両手が機体にあるので Space で記録開始/停止。文字を打つ欄(数値・選択)に
  // いるときだけ奪わない。ボタンやチェックボックスにフォーカスが残っていても
  // 記録にする(タブのボタンを押した直後はフォーカスがそこに残り、Space が
  // そのボタンに取られて無反応に見えていた)。
  useEffect(() => {
    const isTyping = (el: HTMLElement | null) => {
      if (!el) return false;
      if (el.tagName === "SELECT" || el.tagName === "TEXTAREA" || el.isContentEditable) return true;
      return el.tagName === "INPUT" && (el as HTMLInputElement).type !== "checkbox";
    };
    const onKey = (e: KeyboardEvent) => {
      if (e.code !== "Space" || e.repeat) return;
      const el = e.target as HTMLElement | null;
      if (isTyping(el)) return;
      e.preventDefault();
      e.stopPropagation();
      // ボタンは keyup で押されるので、フォーカスを外して押させない
      if (el && el !== document.body) el.blur();
      toggleRecording();
    };
    window.addEventListener("keydown", onKey, true);
    return () => window.removeEventListener("keydown", onKey, true);
  }, [toggleRecording]);

  // スイープ行ごと・センサーごとの開始位置(アンカーから)
  const sweepD0s = useMemo(() => {
    const m = new Map<string, Partial<Record<Channel, SweepD0>>>();
    for (const r of rows) {
      if (!r.offsets || !r.samples.length) continue;
      const e: Partial<Record<Channel, SweepD0>> = {};
      for (const ch of SWEEP_CHANNELS) {
        const est = estimateSweepD0(r, rows, ch);
        if (est) e[ch] = est;
      }
      m.set(r.id, e);
    }
    return m;
  }, [rows]);

  const d0Of = useCallback(
    (r: CalibRow, ch: Channel) => (autoD0 ? sweepD0s.get(r.id)?.[ch]?.d0 : undefined),
    [autoD0, sweepD0s],
  );

  const fits: TargetFit[] = useMemo(
    () =>
      TARGETS.map((def) => {
        const range = ranges[def.key];
        // この範囲をスイープが通っていれば、形はスイープだけで決める
        const sweepPts = targetPoints(def, rows, range, d0Of, true);
        const sweepOnly = !fitStatic && sweepPts.length >= SWEEP_FIT_MIN_POINTS;
        const pts = sweepOnly ? sweepPts : targetPoints(def, rows, range, d0Of);
        const cur = curGains[def.key];
        const fit = fitMode === "b" ? (cur ? fitGain(pts, cur[0]) : null) : fitGain(pts);
        let delta = NaN;
        if (fit && cur) delta = gainToDist(distToRaw(def.refDist, cur), fit.gain) - def.refDist;
        return { def, range, pts, fit, delta, sweepOnly };
      }),
    [rows, ranges, curGains, fitMode, d0Of, fitStatic],
  );

  const apply = async (send: boolean) => {
    const gains = Object.fromEntries(
      fits.filter((f) => f.fit && !excluded.has(f.def.key)).map((f) => [f.def.key, f.fit!.gain]),
    );
    if (!Object.keys(gains).length) {
      toast.error("反映するゲインがありません");
      return;
    }
    setBusy(send ? "send" : "apply");
    try {
      const res = await fetch("/api/sensor-calib", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: "apply", gains, send }),
      });
      const data = await res.json();
      if (!res.ok) {
        if (data.patched) {
          // yaml は書けたが機体へ届かなかった
          throw new Error(
            "sensor.yaml へは保存しました。機体へは届いていません。テストモード15 の実行中は受信しないので、モード28 にするか、機体を再起動してボタン待ちの状態で「保存+送信」を押してください",
          );
        }
        throw new Error(data.error ?? "反映に失敗しました");
      }
      toast.success(
        `sensor.yaml ${send ? "へ保存して機体へ送信しました" : "へ保存しました(機体へは未送信)"}: ${(data.patched as string[]).join(", ")}`,
      );
    } catch (err) {
      toast.error((err as Error).message);
    } finally {
      setBusy(null);
      void refreshGains();
    }
  };

  const saveCsv = async () => {
    setBusy("save");
    try {
      const res = await fetch("/api/sensor-calib", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: "save", rows }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "保存に失敗しました");
      toast.success(`csv/${data.dir}/ に保存しました`);
      void refreshDirs();
    } catch (err) {
      toast.error((err as Error).message);
    } finally {
      setBusy(null);
    }
  };

  // append=false は置き換え、true は今の表に足す(同じファイルは二重に足さない)。
  const loadFromDir = async (append: boolean) => {
    if (!loadDir) return;
    setBusy("load");
    try {
      const res = await fetch(
        `/api/sensor-calib?action=load&rec=${loadRecursive ? 1 : 0}&dir=${encodeURIComponent(loadDir)}`,
      );
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "読込に失敗しました");
      const loaded = (
        data.rows as { group: CalibGroup; dist: number; samples: number[][]; offsets?: number[]; file: string }[]
      ).map(
        (r): CalibRow => ({
          id: newId(),
          group: r.group,
          dist: r.dist,
          use: true,
          samples: r.samples,
          source: r.file,
          ...(r.offsets ? { offsets: r.offsets } : {}),
        }),
      );
      if (append) {
        const have = new Set(rowsRef.current.map((r) => r.source).filter(Boolean));
        const fresh = loaded.filter((r) => !have.has(r.source));
        // 記録していない空の行(標準位置の枠)は追加読込のときに片付ける
        setRows((prev) => sortRows([...prev.filter((r) => r.samples.length), ...fresh]));
        toast.success(`csv/${loadDir} から ${fresh.length} 位置を追加しました`);
      } else {
        setRows(loaded);
        setSelectedId(null);
        toast.success(`csv/${loadDir} から ${loaded.length} 位置を読み込みました`);
      }
    } catch (err) {
      toast.error((err as Error).message);
    } finally {
      setBusy(null);
    }
  };

  const updateRow = (id: string, patch: Partial<CalibRow>) =>
    setRows((prev) => prev.map((r) => (r.id === id ? { ...r, ...patch } : r)));

  const addRow = () => {
    const d = Number(addDist);
    if (!Number.isFinite(d) || addDist === "") return;
    const row: CalibRow = { id: newId(), group: addGroup, dist: d, use: true, samples: [] };
    setRows((prev) => sortRows([...prev, row]));
    setSelectedId(row.id);
    setAddDist("");
  };

  const resetPreset = () => {
    if (rows.some((r) => r.samples.length) && !confirm("記録済みの値をすべて捨てて、最初からやり直しますか?")) return;
    setRows(presetRows());
    setSelectedId(null);
  };

  // 前壁の取り方を切り替える。記録済みの行と横壁の行はそのまま残し、
  // まだ取っていない前壁の行だけを入れ替える。
  //  static: 9 位置に置き直す(従来の手順)
  //  sweep : 機体が走って測る(距離は迷路の寸法で決まるので、置いて測る位置は無い)
  const applyFrontPreset = (kind: "static" | "sweep") => {
    setRows((prev) => {
      const keep = prev.filter((r) => r.samples.length > 0 || (r.group !== "f" && !isSweep(r)));
      const want = kind === "sweep" ? [] : PRESET_POSITIONS.filter((p) => p.group === "f").map((p) => p.dist);
      const out = [...keep];
      for (const d of want) {
        if (out.some((r) => !isSweep(r) && r.group === "f" && r.dist === d)) continue;
        out.push({ id: newId(), group: "f", dist: d, use: true, samples: [] });
      }
      if (kind === "sweep") out.push(emptySweepRow(sweepD0));
      return sortRows(out);
    });
    setSelectedId(null);
  };

  const active = fits.find((f) => f.def.key === activeKey)!;
  const recRow = rows.find((r) => r.id === recordingId);
  const recorded = rows.filter((r) => r.samples.length).length;
  const doneCount = rows.filter((r) => r.samples.length).length;
  const usingSweep = rows.some(isSweep);

  // 機体(テストモード28)のいまの状態。古い表示は使わない
  const fwState = fw && now - fw.at < FW_STATE_TTL[fw.state] ? fw.state : null;
  const mode28 = fwState !== null;

  // いま何をするか(1 行)
  const guide = ((): { text: string; warn: boolean } => {
    if (recRow) {
      return {
        text: `記録中 ${recRow.samples.length}/${nSamples} … 機体を動かさないでください(${placeLabel(recRow)})`,
        warn: false,
      };
    }
    // スイープの走行は必ずケーブルなしで行う
    const SWEEP_STEPS =
      "機体のボタン → スタート位置に置いて前に手をかざす → 機体が走って止まる → ケーブルをつなぐ(ログは自動で届く)";
    if (connected && fwState === "done") {
      return { text: "スイープのログを受信中…", warn: false };
    }
    if (connected && fwState === "wired") {
      return {
        text: "スイープはケーブルなしで走らせます → ケーブルを抜いてください(抜くと続きます。やめるなら機体のボタン)",
        warn: true,
      };
    }
    if (connected && (fwState === "place" || fwState === "near" || fwState === "hand" || fwState === "running")) {
      return { text: "ケーブルがつながっています。走らせる前に必ず抜いてください", warn: true };
    }
    if (!connected && seenMode28 && usingSweep) {
      return {
        text: `前壁スイープ(機体が自分で走ります): 直線 3 区画の奥に前壁 → ${SWEEP_STEPS}。走らないとき: 低い音 4 回=前壁が近すぎる / 短い音 1 回=ケーブル接続中と判定`,
        warn: false,
      };
    }
    if (sweepTodo) {
      if (connected && mode28) {
        return {
          text: `${sweepTodo.samples.length ? "前壁スイープをもう 1 本" : `次 (${doneCount + 1}/${rows.length}) 前壁スイープ(機体が自分で走ります)`}: 直線 3 区画の奥に前壁 → ケーブルを抜く → ${SWEEP_STEPS}`,
          warn: false,
        };
      }
      return {
        text: "前壁スイープはテストモード 28 で行います → 機体をつないで system.yaml を mode: 28 にして送信 → 機体を起動してボタン",
        warn: true,
      };
    }
    if (!rxAlive) {
      return {
        text: connected
          ? "機体の生値が届いていません → system.yaml を mode: 28(または 15)にして送信 → 機体を起動してボタン"
          : "機体がつながっていません → ケーブルをつなぐ(置いて記録する位置は、つないだまま測ります)",
        warn: true,
      };
    }
    if (targetRow) {
      const redo = targetRow.samples.length > 0;
      return {
        text: `${redo ? "取り直し" : `次 (${doneCount + 1}/${rows.length})`}: ${placeLabel(targetRow)}に置いて、手を離して Space`,
        warn: false,
      };
    }
    return {
      text: `${doneCount} 件を記録済み → 右下の表で新しい a, b を確認して「保存」。取り直す位置は行をクリックして Space`,
      warn: false,
    };
  })();

  return (
    <Card className="flex h-full min-h-0 flex-col gap-1.5 overflow-hidden p-2 text-sm">
      {/* 1 行目: いま何をするか + 記録 + 保存。普段使わない操作は「詳細」に畳む */}
      <div className="flex shrink-0 items-center gap-1.5">
        <span
          className={`shrink-0 rounded px-1.5 py-0.5 text-xs ${rxAlive ? "bg-primary/20 text-primary" : "bg-destructive/20 text-destructive"}`}
          title="機体の生値(テストモード15 の 9 列、またはモード14 の sensor: 行)を受信しているか"
        >
          {!connected ? "未接続" : rxAlive ? "受信中" : "未受信"}
        </span>
        <span
          data-calib-guide
          className={`line-clamp-2 min-w-0 flex-1 leading-tight font-medium ${guide.warn ? "text-destructive" : ""}`}
          title={guide.text}
        >
          {guide.text}
        </span>
        <Button
          size="sm"
          variant={recordingId ? "destructive" : "default"}
          disabled={!recordingId && !targetRow}
          onClick={toggleRecording}
          title={
            sweepTodo && !recordingId
              ? "次はスイープです。機体のボタンで始めます(置いて記録する位置は行をクリック)"
              : "Space キーでも開始/停止"
          }
        >
          {recordingId && recRow ? `停止 ${recRow.samples.length}/${nSamples}` : "● 記録 (Space)"}
        </Button>
        <Button
          size="sm"
          variant="outline"
          disabled={busy !== null || !fits.some((f) => f.fit)}
          onClick={() => void apply(false)}
          title="新しい a, b を sensor.yaml に書き込む(機体へは送らない)"
        >
          保存
        </Button>
        <Button
          size="sm"
          variant="secondary"
          disabled={busy !== null || !fits.some((f) => f.fit)}
          onClick={() => void apply(true)}
          title="sensor.yaml に書き込んで機体へ送る。機体が受信できるのは、テストモード28(校正)か 14 の実行中、または起動直後のボタン待ち(モード15 の実行中は受信しない)"
        >
          {busy === "send" ? "送信中..." : "保存+送信"}
        </Button>
        <Button
          size="sm"
          variant={showDetail ? "default" : "ghost"}
          onClick={() => setShowDetail((v) => !v)}
          title="過去の csv の読込、CSV保存、前壁スイープ、フィット方法"
        >
          詳細
        </Button>
      </div>
      {showDetail && (
        <div className="flex shrink-0 flex-wrap items-center gap-1 text-xs">
          <span className="text-muted-foreground">N</span>
          <input
            type="number"
            className="h-7 w-14 rounded border border-input bg-transparent px-1 text-xs"
            value={nSamples}
            min={5}
            onChange={(e) => setNSamples(Math.max(5, Number(e.target.value) || 50))}
            title="1位置あたりのサンプル数(10Hzなので50で約5秒)"
          />
          <span className="h-4 w-px bg-border" />
        <select
          className="h-7 max-w-24 rounded border border-input bg-transparent px-1 text-xs"
          value={loadDir}
          onChange={(e) => setLoadDir(e.target.value)}
          onFocus={() => void refreshDirs()}
          title="csv/ 配下の l_/r_/f_*.csv を読む"
        >
          <option value="">csv/ 読込…</option>
          {dirs.map((d) => (
            <option key={d} value={d}>
              {d}
            </option>
          ))}
        </select>
        <label className="cursor-pointer text-xs text-muted-foreground" title="下位フォルダの csv も読む">
          <input
            type="checkbox"
            className="mr-0.5 align-middle"
            checked={loadRecursive}
            onChange={(e) => setLoadRecursive(e.target.checked)}
          />
          下位
        </label>
        <Button
          size="sm"
          variant="outline"
          disabled={!loadDir || busy !== null}
          onClick={() => void loadFromDir(false)}
          title="表を置き換えて読む"
        >
          読込
        </Button>
        <Button
          size="sm"
          variant="outline"
          disabled={!loadDir || busy !== null}
          onClick={() => void loadFromDir(true)}
          title="今の表に足す(例: csv/ 直下 + 2nd)。同じファイルは二重に足さない"
        >
          +追加
        </Button>
        <Button
          size="sm"
          variant="outline"
          disabled={!recorded || busy !== null}
          onClick={() => void saveCsv()}
          title="csv/calib_<日時>/ に旧形式(l_45.csv 等)で保存。pyplot.py でも読める"
        >
          CSV保存
        </Button>
        <span className="h-4 w-px bg-border" />
        <input
          type="number"
          className="h-7 w-12 rounded border border-input bg-transparent px-1 text-xs"
          value={sweepD0}
          onChange={(e) => setSweepD0(Number(e.target.value) || 0)}
          title="スイープ開始位置の前壁距離 [mm]。過去のログを「ログ取込…」で取り込むときに使う(機体から届いたスイープは機体が知らせた値を使う)。取り込み済みの行は表の距離欄で直す"
        />
        <Button
          size="sm"
          variant={autoD0 ? "default" : "outline"}
          onClick={() => setAutoD0((v) => !v)}
          title="オフ(既定): スイープの距離は迷路の寸法で決める。オン: 既知距離に置いた静止点(前壁の行)へ合わせて、開始位置を L90/R90 別々に求め直す"
        >
          静止点に合わせる
        </Button>
        <label
          className="cursor-pointer text-xs text-muted-foreground"
          title="オフ(既定): スイープが通っている範囲は形をスイープだけで決め、静止点は位置合わせ(アンカー)にだけ使う。オン: 静止点もフィットに混ぜる"
        >
          <input
            type="checkbox"
            className="mr-0.5 align-middle"
            checked={fitStatic}
            onChange={(e) => setFitStatic(e.target.checked)}
          />
          静止点も
        </label>
        <select
          className="h-7 max-w-24 rounded border border-input bg-transparent px-1 text-xs"
          value=""
          onChange={(e) => {
            if (e.target.value) void importSweep(e.target.value, false);
          }}
          onFocus={() => void refreshLogFiles()}
          title="logs/ のログを選ぶと前壁スイープとして取り込む(テストモード28のダンプは保存時に自動で入る)"
        >
          <option value="">ログ取込…</option>
          {logFiles.map((f) => (
            <option key={f} value={f}>
              {f}
            </option>
          ))}
        </select>
        <span className="h-4 w-px bg-border" />
        <Button
          size="sm"
          variant={fitMode === "ab" ? "default" : "outline"}
          onClick={() => setFitMode("ab")}
          title="a, b 両方をフィット(距離2種以上が必要)"
        >
          a,b
        </Button>
        <Button
          size="sm"
          variant={fitMode === "b" ? "default" : "outline"}
          onClick={() => setFitMode("b")}
          title="現在の a を固定して b だけ合わせる(センサー付け直し後の簡易補正。1位置でよい)"
        >
          bのみ
        </Button>
          <span className="h-4 w-px bg-border" />
          <select
            className="h-7 rounded border border-input bg-transparent px-1 text-xs"
            value={addGroup}
            onChange={(e) => setAddGroup(e.target.value as CalibGroup)}
            title="手で置いて記録する位置を足す(比較用)"
          >
            {GROUP_ORDER.map((g) => (
              <option key={g} value={g}>
                {GROUP_LABEL[g]}
              </option>
            ))}
          </select>
          <input
            type="number"
            className="h-7 w-14 rounded border border-input bg-transparent px-1 text-xs"
            placeholder="mm"
            value={addDist}
            onChange={(e) => setAddDist(e.target.value)}
            onKeyDown={(e) => e.key === "Enter" && addRow()}
          />
          <Button size="sm" variant="outline" onClick={addRow} title="手で置いて記録する位置を足す(比較用)">
            + 位置
          </Button>
          <Button
            size="sm"
            variant="outline"
            onClick={() => applyFrontPreset(usingSweep ? "static" : "sweep")}
            title="前壁の取り方を切り替える。既定はスイープ(機体が自分で走って測る)。手置きは前壁から 42〜138mm の 9 位置に置き直す従来の手順"
          >
            {usingSweep ? "前壁を手置き 9 点にする" : "前壁をスイープに戻す"}
          </Button>
        </div>
      )}

      <ResizablePanelGroup direction="horizontal" autoSaveId="sensor-calib" className="min-h-0 flex-1">
        <ResizablePanel defaultSize={30} minSize={20} className="flex min-w-0 flex-col gap-1">
          <div className="shrink-0 font-mono text-[11px] leading-tight text-muted-foreground">
            {CHANNELS.map((c, i) => (
              <span key={c} className="mr-2 inline-block">
                {c}:<span className="text-foreground">{latest ? latest[i] : "-"}</span>
              </span>
            ))}
          </div>
          <div className="min-h-0 flex-1 overflow-auto">
            <table className="w-full text-xs">
              <thead className="sticky top-0 bg-card text-muted-foreground">
                <tr>
                  <th
                    className="px-1 text-left"
                    title="この位置のデータを計算に使うか。外すと計算から除く(置き損じた位置を外す用)。壁の有無ではない"
                  >
                    採用
                  </th>
                  <th className="px-1 text-left" title="どの壁から測った距離か(機体を置く場所)">
                    基準の壁
                  </th>
                  <th className="px-1 text-right" title="基準の壁から機体までの距離 [mm]">
                    距離
                  </th>
                  <th className="px-1 text-right" title="記録したサンプル数">
                    n
                  </th>
                  <th
                    className="px-1 text-right"
                    title="静止点: 見るチャンネルの σ/平均 の最大。3%超は置き直し推奨。スイープ: 開始位置の求め方と姿勢(ホバーで内訳)"
                  >
                    σ%
                  </th>
                  <th className="px-1 text-right" title="この位置での残差(フィット値−実距離)の平均。見るキーは右のプロット">
                    残差
                  </th>
                  <th />
                </tr>
              </thead>
              <tbody>
                {rows.map((r) => {
                  const inst = rowInstability(r);
                  const isRec = r.id === recordingId;
                  const sel = r.id === selectedId || r.id === todoRow?.id;
                  // 選択中キーの範囲外の行は、外挿の誤差として薄く出す
                  const outOfRange =
                    !r.offsets && (r.dist < active.range[0] || r.dist > active.range[1]);
                  const rowD0 = r.offsets ? d0Of(r, active.def.channel) : undefined;
                  // 静止点は全サンプルの平均。スイープは距離が動くので、選択中キーの
                  // 範囲に入る点だけの平均(範囲外は外挿なので混ぜない)。
                  const res =
                    rowServes(r.group, active.def.group) && active.fit
                      ? mean(
                          r.samples.flatMap((s, i) => {
                            const d = sampleDist(r, i, rowD0);
                            if (r.offsets && (d < active.range[0] || d > active.range[1])) return [];
                            const e = gainToDist(s[channelIndex(active.def.channel)], active.fit!.gain) - d;
                            return Number.isFinite(e) ? [e] : [];
                          }),
                        )
                      : NaN;
                  const info = r.offsets ? sweepInfo(r, sweepD0s.get(r.id), autoD0) : null;
                  return (
                    <tr
                      key={r.id}
                      onClick={() => setSelectedId(r.id)}
                      className={`cursor-pointer border-t border-border/40 ${sel ? "bg-primary/15" : "hover:bg-muted/40"} ${r.use ? "" : "opacity-50"}`}
                    >
                      <td className="px-1">
                        <input
                          type="checkbox"
                          checked={r.use}
                          title="この位置のデータを計算に使う"
                          onClick={(e) => e.stopPropagation()}
                          onChange={(e) => updateRow(r.id, { use: e.target.checked })}
                        />
                      </td>
                      <td className="px-1" title={r.source}>
                        {r.offsets ? "前壁スイープ" : GROUP_LABEL[r.group]}
                      </td>
                      <td className="px-1 text-right">
                        {isEmptySweep(r) ? (
                          <span className="text-muted-foreground" title="機体が走って測る行。走らせるとここに入る">
                            走行
                          </span>
                        ) : rowD0 !== undefined ? (
                          // アンカーから求めた開始位置(選択中のキーのセンサー)。手入力値は使わない
                          <span className="font-mono" title={info?.title}>
                            {rowD0.toFixed(1)}
                          </span>
                        ) : (
                          <input
                            type="number"
                            className="w-12 bg-transparent text-right"
                            value={r.dist}
                            onClick={(e) => e.stopPropagation()}
                            onChange={(e) => updateRow(r.id, { dist: Number(e.target.value) })}
                            title={
                              r.offsets
                                ? (() => {
                                    const [lo, hi] = offsetSpan(r.offsets);
                                    return `スイープ開始位置の前壁距離(仮の値を手入力)。前壁から ${(r.dist + lo).toFixed(0)}〜${(r.dist + hi).toFixed(0)}mm を測った扱い。前壁の静止点を記録すると自動で合わせる`;
                                  })()
                                : undefined
                            }
                          />
                        )}
                      </td>
                      <td className="px-1 text-right">
                        {isRec ? <span className="text-destructive">●{r.samples.length}</span> : r.samples.length || "-"}
                      </td>
                      {info ? (
                        <td
                          className={`px-1 text-right whitespace-nowrap ${info.warn ? "font-semibold text-destructive" : ""}`}
                          title={info.title}
                        >
                          {info.label}
                        </td>
                      ) : (
                        <td
                          className={`px-1 text-right ${inst > UNSTABLE_REL ? "font-semibold text-destructive" : ""}`}
                        >
                          {Number.isFinite(inst) ? (inst * 100).toFixed(1) : "-"}
                        </td>
                      )}
                      <td
                        className={`px-1 text-right ${outOfRange ? "opacity-40" : ""}`}
                        title={outOfRange ? `${active.def.key} のフィット範囲外(外挿)` : undefined}
                      >
                        {f2(res)}
                      </td>
                      <td className="px-1 text-right whitespace-nowrap">
                        <button
                          type="button"
                          className={`px-1 text-muted-foreground hover:text-foreground ${r.offsets ? "invisible" : ""}`}
                          disabled={!!recordingId || !rxAlive || !!r.offsets}
                          onClick={(e) => {
                            e.stopPropagation();
                            startRecording(r.id);
                          }}
                          title="この位置を記録(記録済みなら取り直し)"
                        >
                          ●
                        </button>
                        <button
                          type="button"
                          className="px-1 text-muted-foreground hover:text-destructive"
                          disabled={isRec}
                          onClick={(e) => {
                            e.stopPropagation();
                            setRows((prev) => prev.filter((x) => x.id !== r.id));
                          }}
                          title="行を削除"
                        >
                          ×
                        </button>
                      </td>
                    </tr>
                  );
                })}
              </tbody>
            </table>
          </div>
          <div className="flex shrink-0 items-center gap-1">
            <Button
              size="sm"
              variant="ghost"
              className="ml-auto"
              onClick={resetPreset}
              title="記録をすべて捨てて、最初の状態(横壁 4 位置 + 前壁スイープ)に戻す"
            >
              全部やり直す
            </Button>
          </div>
        </ResizablePanel>
        <ResizableHandle withHandle />
        <ResizablePanel defaultSize={70} minSize={40} className="flex min-w-0 flex-col gap-1">
          <div className="min-h-0 flex-1">
            <CalibPlot
              fit={active}
              rows={rows}
              d0Of={d0Of}
              cur={curGains[activeKey]}
              siblings={fits.filter((f) => f.def.channel === active.def.channel && f.def.key !== active.def.key)}
            />
          </div>
          <div className="max-h-[45%] shrink-0 overflow-auto">
            <table className="w-full text-xs">
              <thead className="sticky top-0 bg-card text-muted-foreground">
                <tr>
                  <th className="px-1 text-left" title="yaml保存の対象">
                    反映
                  </th>
                  <th className="px-1 text-left">キー</th>
                  <th className="px-1 text-left whitespace-nowrap" title="フィットに使う距離範囲 [mm]">
                    範囲
                    <label
                      className="ml-1 cursor-pointer font-normal"
                      title="左右連動: L90_near を変えると R90_near も同じ範囲にする"
                    >
                      <input
                        type="checkbox"
                        className="mr-0.5 align-middle"
                        checked={linkLR}
                        onChange={(e) => setLinkLR(e.target.checked)}
                      />
                      L=R
                    </label>
                    <button
                      type="button"
                      className="ml-1 font-normal text-muted-foreground hover:text-foreground"
                      onClick={() => setRanges(defaultRanges())}
                      title="範囲を既定(45系 全域 / near,mid 42〜96 / far 84〜138)に戻す"
                    >
                      既定
                    </button>
                  </th>
                  <th className="px-1 text-right" title="サンプル数 / 距離の種類">
                    n
                  </th>
                  <th className="px-1 text-right" title="全サンプルの残差 RMS [mm]">
                    rms
                  </th>
                  <th className="px-1 text-right">現在 a, b</th>
                  <th className="px-1 text-right">新 a, b</th>
                  <th className="px-1 text-right" title="基準距離で現在値が読む生値を、新しい係数で換算したときのずれ [mm]">
                    Δ@基準
                  </th>
                  <th className="px-1 text-right" title="最新の生値を 現在→新 の係数で換算した距離 [mm]">
                    いま
                  </th>
                </tr>
              </thead>
              <tbody>
                {fits.map((f) => {
                  const k = f.def.key;
                  const cur = curGains[k];
                  const raw = latest ? latest[channelIndex(f.def.channel)] : NaN;
                  const sel = k === activeKey;
                  return (
                    <tr
                      key={k}
                      onClick={() => setActiveKey(k)}
                      className={`cursor-pointer border-t border-border/40 ${sel ? "bg-primary/15" : "hover:bg-muted/40"}`}
                    >
                      <td className="px-1">
                        <input
                          type="checkbox"
                          disabled={!f.fit}
                          checked={!!f.fit && !excluded.has(k)}
                          onClick={(e) => e.stopPropagation()}
                          onChange={(e) =>
                            setExcluded((prev) => {
                              const next = new Set(prev);
                              if (e.target.checked) next.delete(k);
                              else next.add(k);
                              return next;
                            })
                          }
                        />
                      </td>
                      <td className="px-1 font-mono">{k}</td>
                      <td className="px-1 whitespace-nowrap">
                        {(["0", "1"] as const).map((i) => (
                          <input
                            key={i}
                            type="number"
                            className="w-11 bg-transparent text-right"
                            value={f.range[Number(i)]}
                            onClick={(e) => e.stopPropagation()}
                            onChange={(e) =>
                              setRanges((prev) => {
                                const r: [number, number] = [...prev[k]];
                                r[Number(i)] = Number(e.target.value);
                                const pk = linkLR ? pairedKey(k) : null;
                                return pk ? { ...prev, [k]: r, [pk]: [...r] } : { ...prev, [k]: r };
                              })
                            }
                          />
                        ))}
                      </td>
                      <td
                        className="px-1 text-right"
                        title={f.sweepOnly ? "スイープの点だけでフィット(静止点は位置合わせにだけ使用)" : undefined}
                      >
                        {f.fit
                          ? f.sweepOnly
                            ? `${f.fit.n} スイープ`
                            : `${f.fit.n}/${f.fit.nDist}`
                          : f.pts.length
                            ? `${f.pts.length}/-`
                            : "-"}
                      </td>
                      <td className="px-1 text-right">{f.fit ? f2(f.fit.rms) : "-"}</td>
                      <td className="px-1 text-right font-mono text-muted-foreground">{fg(cur)}</td>
                      <td className="px-1 text-right font-mono">{f.fit ? fg(f.fit.gain) : "-"}</td>
                      <td
                        className={`px-1 text-right ${Math.abs(f.delta) > 2 ? "font-semibold text-destructive" : ""}`}
                        title={`基準 ${f.def.refDist}mm`}
                      >
                        {Number.isFinite(f.delta) ? `${f.delta > 0 ? "+" : ""}${f1(f.delta)}` : "-"}
                      </td>
                      <td className="px-1 text-right whitespace-nowrap font-mono">
                        {cur ? f1(gainToDist(raw, cur)) : "-"}
                        {f.fit ? ` → ${f1(gainToDist(raw, f.fit.gain))}` : ""}
                      </td>
                    </tr>
                  );
                })}
              </tbody>
            </table>
          </div>
        </ResizablePanel>
      </ResizablePanelGroup>
    </Card>
  );
}

// raw(対数軸) → 距離 の散布図と換算曲線。点は全サンプル、範囲外の位置は薄く。
// 90度センサーの near/mid/far の色(同じチャンネルの曲線と範囲帯を重ねて描く)
const SEGMENT_COLOR: Record<string, string> = {
  near: "var(--chart-3)",
  mid: "var(--chart-4)",
  far: "var(--chart-2)",
};
const segmentOf = (k: TargetKey) => /_(near|mid|far)$/.exec(k)?.[1] ?? "";
const colorOf = (k: TargetKey) => SEGMENT_COLOR[segmentOf(k)] ?? "var(--chart-3)";

function CalibPlot({
  fit,
  rows,
  d0Of,
  cur,
  siblings,
}: {
  fit: TargetFit;
  rows: CalibRow[];
  d0Of: (r: CalibRow, ch: Channel) => number | undefined;
  cur?: Gain;
  siblings: TargetFit[]; // 同じチャンネルの他のキー(L90_near に対する L90_mid/far)
}) {
  const ref = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ w: 600, h: 300 });
  const [hover, setHover] = useState<string | null>(null);

  useEffect(() => {
    const el = ref.current;
    if (!el) return;
    const ro = new ResizeObserver(([e]) => setSize({ w: e.contentRect.width, h: e.contentRect.height }));
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  const { def, range } = fit;
  const ci = channelIndex(def.channel);
  const groupRows = rows.filter((r) => rowServes(r.group, def.group) && r.samples.length);
  const all = groupRows.flatMap((r) => {
    const d0 = r.offsets ? d0Of(r, def.channel) : undefined;
    // 静止点がアンカー専用のとき(形はスイープで決めている)は、フィットに使って
    // いないことが分かるよう薄く描く。平均マーカーは下で別に出す。
    const used = r.use && (r.offsets || !fit.sweepOnly);
    return r.samples.map((s, i) => {
      const d = sampleDist(r, i, d0);
      return { raw: s[ci], dist: d, inRange: !!used && d >= range[0] && d <= range[1] };
    });
  });
  const valid = all.filter((p) => p.raw > 1);
  // 描画だけ間引く(スイープ 1 本で 2000 点超になり SVG が重くなる)。フィットは全点。
  const MAX_DOTS = 3000;
  const stride = Math.max(1, Math.ceil(valid.length / MAX_DOTS));
  const dots = stride > 1 ? valid.filter((_, i) => i % stride === 0) : valid;

  const PAD = { l: 44, r: 12, t: 18, b: 28 };
  const w = Math.max(100, size.w);
  const h = Math.max(100, size.h);
  let rawMin = valid.length ? Infinity : 10;
  let rawMax = valid.length ? -Infinity : 3000;
  for (const p of valid) {
    if (p.raw < rawMin) rawMin = p.raw;
    if (p.raw > rawMax) rawMax = p.raw;
  }
  const lx0 = Math.log10(Math.max(2, rawMin * 0.8));
  const lx1 = Math.log10(Math.max(rawMin * 1.25 + 1, rawMax * 1.25));
  const dists = valid.map((p) => p.dist);
  let dMin = Infinity;
  let dMax = -Infinity;
  for (const d of dists) {
    if (d < dMin) dMin = d;
    if (d > dMax) dMax = d;
  }
  const y0 = dists.length ? dMin - 10 : 0;
  const y1 = dists.length ? dMax + 10 : 150;
  const X = (raw: number) => PAD.l + ((Math.log10(raw) - lx0) / (lx1 - lx0)) * (w - PAD.l - PAD.r);
  const Y = (d: number) => h - PAD.b - ((d - y0) / (y1 - y0)) * (h - PAD.t - PAD.b);

  const curve = (g: Gain) => {
    const pts: string[] = [];
    for (let i = 0; i <= 80; i++) {
      const raw = 10 ** (lx0 + ((lx1 - lx0) * i) / 80);
      const d = gainToDist(raw, g);
      if (Number.isFinite(d) && d > y0 - 40 && d < y1 + 40) pts.push(`${X(raw).toFixed(1)},${Y(d).toFixed(1)}`);
    }
    return pts.join(" ");
  };

  const xTicks: number[] = [];
  for (const m of [1, 2, 5]) for (let e = 0; e <= 4; e++) xTicks.push(m * 10 ** e);
  const yStep = y1 - y0 > 100 ? 20 : 10;
  const yTicks: number[] = [];
  for (let d = Math.ceil(y0 / yStep) * yStep; d <= y1; d += yStep) yTicks.push(d);

  // 位置ごとの平均マーカー(スイープ行は点が連続しているので出さない)
  const means = groupRows.filter((r) => !r.offsets).map((r) => {
    const xs = r.samples.map((s) => s[ci]).filter((v) => v > 1);
    return { id: r.id, dist: r.dist, raw: mean(xs), inRange: r.use && r.dist >= range[0] && r.dist <= range[1] };
  });

  return (
    <div ref={ref} className="relative h-full w-full">
      <svg width={w} height={h} className="absolute inset-0">
        <text x={PAD.l} y={12} className="fill-foreground text-[11px]">
          {def.key}({def.channel}, {GROUP_LABEL[def.group]}) — 太線: 新 / 破線: 現在
          {siblings.length > 0 && " / 細線: 同じセンサーの他の組(帯=フィット範囲)"}
        </text>
        {/* フィット範囲の帯。選択中のキーは全幅に薄く、同じセンサーの他の組は右端に細く */}
        {[fit, ...siblings].map((f, i) => {
          const top = Y(Math.min(f.range[1], y1));
          const bottom = Y(Math.max(f.range[0], y0));
          if (!(bottom > top)) return null;
          const main = i === 0;
          const x = main ? PAD.l : w - PAD.r - 6 * (siblings.length - i + 1);
          return (
            <rect
              key={`band-${f.def.key}`}
              x={x}
              y={top}
              width={main ? w - PAD.l - PAD.r : 5}
              height={bottom - top}
              fill={colorOf(f.def.key)}
              opacity={main ? 0.07 : 0.6}
            >
              <title>
                {f.def.key}: {f.range[0]}〜{f.range[1]}mm
              </title>
            </rect>
          );
        })}
        {xTicks
          .filter((t) => Math.log10(t) >= lx0 && Math.log10(t) <= lx1)
          .map((t) => (
            <g key={`x${t}`}>
              <line x1={X(t)} x2={X(t)} y1={PAD.t} y2={h - PAD.b} className="stroke-border" strokeWidth={0.5} />
              <text x={X(t)} y={h - PAD.b + 12} textAnchor="middle" className="fill-muted-foreground text-[10px]">
                {t}
              </text>
            </g>
          ))}
        {yTicks.map((t) => (
          <g key={`y${t}`}>
            <line x1={PAD.l} x2={w - PAD.r} y1={Y(t)} y2={Y(t)} className="stroke-border" strokeWidth={0.5} />
            <text x={PAD.l - 4} y={Y(t) + 3} textAnchor="end" className="fill-muted-foreground text-[10px]">
              {t}
            </text>
          </g>
        ))}
        <text x={w - PAD.r} y={h - 4} textAnchor="end" className="fill-muted-foreground text-[10px]">
          raw (log)
        </text>
        <text x={4} y={PAD.t - 4} className="fill-muted-foreground text-[10px]">
          mm
        </text>
        {dots.map((p, i) => (
          <circle
            key={i}
            cx={X(p.raw)}
            cy={Y(p.dist)}
            r={1.6}
            fill="var(--chart-1)"
            opacity={p.inRange ? 0.35 : 0.08}
          />
        ))}
        {cur && (
          <polyline points={curve(cur)} fill="none" stroke="var(--muted-foreground)" strokeWidth={1.5} strokeDasharray="4 3" />
        )}
        {siblings
          .filter((f) => f.fit)
          .map((f) => (
            <polyline
              key={`sib-${f.def.key}`}
              points={curve(f.fit!.gain)}
              fill="none"
              stroke={colorOf(f.def.key)}
              strokeWidth={1}
              opacity={0.8}
            />
          ))}
        {fit.fit && (
          <polyline points={curve(fit.fit.gain)} fill="none" stroke={colorOf(def.key)} strokeWidth={2.5} />
        )}
        {means
          .filter((m) => Number.isFinite(m.raw))
          .map((m) => (
            <circle
              key={m.id}
              cx={X(m.raw)}
              cy={Y(m.dist)}
              r={hover === m.id ? 6 : 4.5}
              fill="var(--card)"
              stroke={m.inRange ? "var(--chart-1)" : "var(--muted-foreground)"}
              strokeWidth={2}
              onMouseEnter={() => setHover(m.id)}
              onMouseLeave={() => setHover(null)}
            />
          ))}
      </svg>
      {siblings.length > 0 && (
        <div className="pointer-events-none absolute top-5 right-8 flex flex-col gap-0.5 rounded bg-card/80 px-1.5 py-1 text-[10px]">
          {[fit, ...siblings].map((f) => (
            <div key={`lg-${f.def.key}`} className="flex items-center gap-1">
              <span
                className="inline-block h-0.5 w-4"
                style={{ background: colorOf(f.def.key), height: f === fit ? 3 : 1 }}
              />
              <span className={f === fit ? "text-foreground" : "text-muted-foreground"}>
                {f.def.key} {f.range[0]}〜{f.range[1]}
                {f.fit ? ` rms ${f2(f.fit.rms)}` : ""}
              </span>
            </div>
          ))}
        </div>
      )}
      {hover &&
        (() => {
          const m = means.find((x) => x.id === hover);
          if (!m) return null;
          const dNew = fit.fit ? gainToDist(m.raw, fit.fit.gain) : NaN;
          const dCur = cur ? gainToDist(m.raw, cur) : NaN;
          return (
            <div
              className="pointer-events-none absolute rounded border border-border bg-popover px-2 py-1 text-xs shadow"
              style={{ left: Math.min(X(m.raw) + 10, w - 170), top: Math.max(Y(m.dist) - 40, 0) }}
            >
              <div>
                {m.dist}mm / raw 平均 {m.raw.toFixed(1)}
              </div>
              <div className="text-muted-foreground">
                現在 {f1(dCur)}mm → 新 {f1(dNew)}mm
              </div>
            </div>
          );
        })()}
      {!valid.length && (
        <div className="absolute inset-0 flex items-center justify-center text-muted-foreground">
          {def.group === "f" ? "前壁" : `横(左右)か${GROUP_LABEL[def.group]}`}の位置を記録するとここに出ます
        </div>
      )}
    </div>
  );
}
