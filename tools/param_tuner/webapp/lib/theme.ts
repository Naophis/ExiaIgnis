// テーマカラー(画面全体の主色)。クライアント / サーバー共用、Node 依存なし。
//
// 主色から作る色(ボタン・バッジ・選択中のタブ・フォーカスの輪・枠線の色味など)は
// app/globals.css が --theme-* の変数から計算している。ここでは、選んだ色からその変数の
// 値を決めて <html> の style に書くだけ。背景(濃紺のパネル)・警告色・グラフの系列の色は
// 変えない。

export interface ThemePrefs {
  // machine = 機体ごとの色(machines.yaml の color。機体を切り替えると全体の色も変わる)
  // fixed   = 全機体で 1 色(このブラウザに覚える)
  mode: "machine" | "fixed";
  color: string | null; // fixed のときの色(#rrggbb)。null = 既定
}

// 既定は「機体ごとの色」: どの機体を見ているかが画面全体の色で分かる。
export const DEFAULT_PREFS: ThemePrefs = { mode: "machine", color: null };

// 既定の主色 oklch(0.78 0.16 170)(GN 粒子のグリーン)にいちばん近い sRGB の色。
export const DEFAULT_THEME_HEX = "#00d7a8";

// 設定(このブラウザに覚える)
export const THEME_PREFS_KEY = "exia-theme-v1";
// いま当てている CSS 変数({"--theme-l": "0.45", ...})。機体ごとの色のときも、機体を知らない
// ページ(/logs)や読み込み直後の描画前スクリプトがそのまま当てられるように、決まった値を残す。
export const THEME_APPLIED_KEY = "exia-theme-applied-v2";
// 同じページの中で色を変えたことを知らせるイベント(別のタブへは storage イベントで伝わる)
export const THEME_EVENT = "exia-theme-change";

// 色見本。ソレスタルビーイングの機体と装備の色から(名前は色見本のツールチップに出る)。
export const THEME_PRESETS: { name: string; color: string | null }[] = [
  { name: "GN 粒子(既定)", color: null },
  { name: "エクシア(青)", color: "#2b31d4" },
  { name: "ダブルオー(空色)", color: "#38bdf8" },
  { name: "デュナメス(緑)", color: "#22a55b" },
  { name: "キュリオス(橙)", color: "#fb923c" },
  { name: "ヴァーチェ(紫)", color: "#a78bfa" },
  { name: "トランザム(赤)", color: "#ff4d6d" },
  { name: "ヴェーダ(水色)", color: "#22d3ee" },
  { name: "ゴールド(金)", color: "#fbbf24" },
  { name: "GN フィールド(若草)", color: "#a3e635" },
  { name: "プトレマイオス(白)", color: "#e5e7eb" },
];

interface Lch {
  l: number;
  c: number;
  h: number;
}

// sRGB の #rrggbb → OKLCH(Björn Ottosson の OKLab)
export function hexToLch(hex: string): Lch | null {
  const m = /^#?([0-9a-f]{6})$/i.exec(hex.trim());
  if (!m) return null;
  const n = parseInt(m[1], 16);
  const lin = (v: number) => {
    const s = v / 255;
    return s <= 0.04045 ? s / 12.92 : Math.pow((s + 0.055) / 1.055, 2.4);
  };
  const r = lin((n >> 16) & 255);
  const g = lin((n >> 8) & 255);
  const b = lin(n & 255);
  const l_ = Math.cbrt(0.4122214708 * r + 0.5363325363 * g + 0.0514459929 * b);
  const m_ = Math.cbrt(0.2119034982 * r + 0.6806995451 * g + 0.1073969566 * b);
  const s_ = Math.cbrt(0.0883024619 * r + 0.2817188376 * g + 0.6299787005 * b);
  const L = 0.2104542553 * l_ + 0.793617785 * m_ - 0.0040720468 * s_;
  const A = 1.9779984951 * l_ - 2.428592205 * m_ + 0.4505937099 * s_;
  const B = 0.0259040371 * l_ + 0.7827717662 * m_ - 0.808675766 * s_;
  let h = (Math.atan2(B, A) * 180) / Math.PI;
  if (h < 0) h += 360;
  return { l: L, c: Math.hypot(A, B), h };
}

export type ThemeVars = Record<string, string>;

// globals.css の .dark と同じ既定値。color が null のときは何も上書きしない(= この値)。
//   --theme-l/c/h   塗り(ボタン・バッジの地)。選んだ色そのもの
//   --theme-bl/bc   暗い背景の上の文字・線・フォーカスの輪に使う、明るい方の色
//   --theme-tc      背景や枠線に薄く混ぜる色味の強さ
const L_MIN = 0.42; // これより暗い色は、同じ色相のまま明るくする(暗い背景に溶けて見えなくなる)
const L_MAX = 0.9;
const BRIGHT_L_MIN = 0.74; // 文字・線として読める明るさ
const BRIGHT_C_MAX = 0.17;
const TINT_C_MAX = 0.16;
const FG_FLIP_L = 0.66; // 塗りがこれより暗ければ、上に載せる文字を白にする

const r3 = (v: number) => String(Math.round(v * 1000) / 1000);

// 選んだ色 → <html> に書く CSS 変数。null = 既定(上書きなし)。
export function themeVars(color: string | null): ThemeVars | null {
  if (!color) return null;
  const lch = hexToLch(color);
  if (!lch) return null;
  const l = Math.min(L_MAX, Math.max(L_MIN, lch.l));
  const c = lch.c;
  const h = c < 0.005 ? 0 : lch.h;
  return {
    "--theme-l": r3(l),
    "--theme-c": r3(c),
    "--theme-h": r3(h),
    "--theme-bl": r3(Math.max(l, BRIGHT_L_MIN)),
    "--theme-bc": r3(Math.min(c, BRIGHT_C_MAX)),
    "--theme-tc": r3(Math.min(c, TINT_C_MAX)),
    // 塗りの上の文字: 暗い塗りには白、明るい塗りには今までどおりの濃紺
    "--primary-foreground": l < FG_FLIP_L ? `oklch(0.98 0.01 ${r3(h)})` : "oklch(0.13 0.02 240)",
  };
}

const VAR_NAMES = ["--theme-l", "--theme-c", "--theme-h", "--theme-bl", "--theme-bc", "--theme-tc", "--primary-foreground"];

// <html> に当てる(null = 既定へ戻す)。ブラウザの中だけで呼ぶ。
export function applyThemeVars(vars: ThemeVars | null): void {
  const s = document.documentElement.style;
  for (const name of VAR_NAMES) {
    const v = vars?.[name];
    if (v) s.setProperty(name, v);
    else s.removeProperty(name);
  }
}

function readApplied(): ThemeVars | null {
  try {
    const v = JSON.parse(localStorage.getItem(THEME_APPLIED_KEY) ?? "null") as ThemeVars | null;
    return v && typeof v === "object" ? v : null;
  } catch {
    return null; // 読めなくても既定の色で動く
  }
}

// 残してある色を当て直す(読み込み直後・別のタブで変えたとき)
export function applyStoredTheme(): void {
  applyThemeVars(readApplied());
}

// 色を決めて当て、残す。同じページのほかの部品と、別のタブへも伝わる。
export function commitTheme(color: string | null): void {
  const vars = themeVars(color);
  applyThemeVars(vars);
  try {
    if (vars) localStorage.setItem(THEME_APPLIED_KEY, JSON.stringify(vars));
    else localStorage.removeItem(THEME_APPLIED_KEY);
  } catch {
    // 覚えられなくても、この画面には当たっている
  }
  window.dispatchEvent(new Event(THEME_EVENT));
}

export function loadThemePrefs(): ThemePrefs {
  try {
    const v = JSON.parse(localStorage.getItem(THEME_PREFS_KEY) ?? "null") as Partial<ThemePrefs> | null;
    if (!v) return DEFAULT_PREFS;
    return {
      mode: v.mode === "fixed" ? "fixed" : "machine",
      color: typeof v.color === "string" && /^#[0-9a-f]{6}$/i.test(v.color) ? v.color : null,
    };
  } catch {
    return DEFAULT_PREFS;
  }
}

export function saveThemePrefs(prefs: ThemePrefs): void {
  try {
    localStorage.setItem(THEME_PREFS_KEY, JSON.stringify(prefs));
  } catch {
    // 覚えられなくても動く
  }
}

// 描画の前に走らせる 1 行スクリプト(app/layout.tsx の <head>)。読み込んだ瞬間に既定の色が
// 見えてから切り替わるのを防ぐ。applyStoredTheme() と同じことを素の JS で書いたもの
// (-- で始まる名前だけを当てる)。
export const THEME_INIT_SCRIPT = `(function(){try{var v=JSON.parse(localStorage.getItem(${JSON.stringify(
  THEME_APPLIED_KEY,
)})||"null");if(!v||typeof v!=="object")return;var s=document.documentElement.style;for(var k in v){if(k.slice(0,2)==="--"&&typeof v[k]==="string")s.setProperty(k,v[k])}}catch(e){}})()`;
