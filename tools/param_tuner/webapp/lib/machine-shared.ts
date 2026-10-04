// クライアント / サーバー共用の機体まわりの型と小さな関数(Node 依存なし)。

export interface Machine {
  id: string; // フォルダ名(machines/<id>/profile)。英数字と _ -
  label: string; // 画面に出す名前
  color: string; // 画面での色(#rrggbb)
  serials: string[]; // この機体の基板の USB シリアル番号
  note?: string;
}

// 機体ごとに違ってよい値。鍵はファイル(profile からの相対パス。* を使える)、
// 値は true(ファイル全体)かキーの一覧(そのキーと、その下すべて)。
export type SpecificMap = Record<string, true | string[]>;

export interface MachineRegistry {
  default: string | null; // 機体を指定しないスクリプトが使う機体
  machines: Machine[];
  specific: SpecificMap;
}

export const MACHINE_ID_RE = /^[A-Za-z0-9][A-Za-z0-9_-]{0,31}$/;

// 色を指定しないときの割り当て(並び順)。暗い背景で見分けやすい 6 色。
// 先頭は既定のテーマカラー(GN グリーン)。機体の色はそのまま画面全体のテーマカラーになる(lib/theme.ts)。
export const MACHINE_COLORS = ["#00d7a8", "#f59e0b", "#a78bfa", "#38bdf8", "#fb7185", "#a3e635"];

// クライアントが今の機体を API へ伝えるヘッダー(lib/machine-client.ts の apiFetch が付ける)
export const MACHINE_HEADER = "x-exia-machine";

export type PathSeg = string | number;

// キーの場所を 1 行の文字列にする(a.b[3].c)。表示と「固有」の登録に使う。
export function pathToString(segs: readonly PathSeg[]): string {
  let out = "";
  for (const s of segs) {
    if (typeof s === "number") out += `[${s}]`;
    else out += out ? `.${s}` : s;
  }
  return out;
}

function globToRegExp(glob: string): RegExp {
  const esc = glob.replace(/[.+^${}()|[\]\\]/g, "\\$&").replace(/\*/g, "[^/]*");
  return new RegExp(`^${esc}$`);
}

// file(例 "hf/offset.yaml")の path(例 "sla_wall_ref_l")が「固有」に入っているか。
// キーの登録は、そのキー自身と、その下(a.b / a[0])すべてに効く。
export function isSpecific(specific: SpecificMap, file: string, pathStr: string): boolean {
  for (const [pattern, rule] of Object.entries(specific)) {
    if (pattern !== file && !(pattern.includes("*") && globToRegExp(pattern).test(file))) continue;
    if (rule === true) return true;
    for (const p of rule) {
      if (pathStr === p) return true;
      if (pathStr.startsWith(p) && (pathStr[p.length] === "." || pathStr[p.length] === "[")) return true;
    }
  }
  return false;
}

// ファイル全体が固有か(キー単位の登録は見ない)
export function isFileSpecific(specific: SpecificMap, file: string): boolean {
  for (const [pattern, rule] of Object.entries(specific)) {
    if (rule !== true) continue;
    if (pattern === file || (pattern.includes("*") && globToRegExp(pattern).test(file))) return true;
  }
  return false;
}

// 接続状態に添える基板の情報
export interface BoardInfo {
  serial: string | null; // 接続中の基板の USB シリアル番号
  machine: string | null; // その基板が登録されている機体(未登録なら null)
}

// シリアル番号を短く出す(末尾 6 桁)
export function shortSerial(serial: string): string {
  return serial.length > 8 ? `…${serial.slice(-6)}` : serial;
}

// 背景色の上に載せる文字色(明るい色なら黒、暗い色なら白)
export function contrastText(hex: string): string {
  const m = /^#?([0-9a-f]{6})$/i.exec(hex);
  if (!m) return "#000";
  const n = parseInt(m[1], 16);
  const r = (n >> 16) & 255;
  const g = (n >> 8) & 255;
  const b = n & 255;
  return (r * 299 + g * 587 + b * 114) / 1000 > 140 ? "#0b1220" : "#f8fafc";
}
