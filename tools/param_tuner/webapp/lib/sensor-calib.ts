import fs from "node:fs";
import path from "node:path";
import { load as loadYaml } from "js-yaml";
import {
  GROUP_ORDER,
  TARGET_KEYS,
  parseCsv,
  rowToCsv,
  type CalibGroup,
  type Gain,
  type TargetKey,
} from "./sensor-calib-shared";

// webapp/ is the Next.js server cwd; tools/param_tuner/ is one level up.
const PARAM_TUNER_ROOT = path.join(process.cwd(), "..");
const SENSOR_YAML_PATH = path.join(PARAM_TUNER_ROOT, "profile", "hf", "sensor.yaml");
// 旧手順(sensor.sh / pyplot.py)の作業場所。セッション保存もここに置いて
// pyplot.py からも読めるようにする。
export const CSV_DIR = path.join(PARAM_TUNER_ROOT, "..", "..", "csv");

const CSV_NAME_RE = /^([lrf])_(-?\d+(?:\.\d+)?)\.csv$/;
// 前壁スイープ(テストモード28)の保存形式。行ごとに dist が違う。
const SWEEP_NAME_RE = /^f_sweep\d*\.csv$/;
const isCalibCsv = (name: string) => CSV_NAME_RE.test(name) || SWEEP_NAME_RE.test(name);

function resolveCsvDir(rel: string): string {
  const abs = path.resolve(CSV_DIR, rel);
  if (abs !== CSV_DIR && !abs.startsWith(CSV_DIR + path.sep)) {
    throw new Error("csv/ の外は読めません");
  }
  return abs;
}

// l_/r_/f_*.csv を含むディレクトリ(csv/ からの相対パス)。
export function listCalibDirs(): string[] {
  const out: string[] = [];
  const walk = (abs: string, depth: number) => {
    let entries: fs.Dirent[];
    try {
      entries = fs.readdirSync(abs, { withFileTypes: true });
    } catch {
      return;
    }
    if (entries.some((e) => e.isFile() && isCalibCsv(e.name))) {
      out.push(path.relative(CSV_DIR, abs) || ".");
    }
    if (depth >= 3) return;
    for (const e of entries) if (e.isDirectory()) walk(path.join(abs, e.name), depth + 1);
  };
  walk(CSV_DIR, 0);
  return out.sort();
}

export interface LoadedRow {
  group: CalibGroup;
  dist: number;
  samples: number[][];
  offsets?: number[]; // スイープ行のみ(CalibRow と同じ意味)
  file: string;
}

// recursive=true ならサブディレクトリも含める(csv/1st/near, csv/1st/far のような
// 手作業の分割を1回で読むため)。
export function loadCalibDir(rel: string, recursive: boolean): LoadedRow[] {
  const root = resolveCsvDir(rel);
  const rows: LoadedRow[] = [];
  // 手作業で near/far を入れ替えていた名残で、同じファイルのコピーが親子の
  // ディレクトリに並んでいることがある(csv/1st と csv/1st/far)。二重に
  // 重み付けしないよう、同じ置き方で中身が同一のものは1回だけ読む。
  // l_ と r_ で中身が同一の組は左右同時の測定なので s の1行にまとめる。
  const seen = new Map<string, LoadedRow>();
  const walk = (abs: string, depth: number) => {
    for (const e of fs.readdirSync(abs, { withFileTypes: true })) {
      const p = path.join(abs, e.name);
      if (e.isDirectory()) {
        if (recursive && depth < 3) walk(p, depth + 1);
        continue;
      }
      if (SWEEP_NAME_RE.test(e.name)) {
        const text = fs.readFileSync(p, "utf-8");
        if (seen.has(text)) continue;
        const parsed = parseCsv(text);
        if (!parsed) continue;
        const d0 = Math.min(...parsed.dists);
        const row: LoadedRow = {
          group: "f",
          dist: d0,
          samples: parsed.samples,
          offsets: parsed.dists.map((d) => d - d0),
          file: path.relative(CSV_DIR, p),
        };
        seen.set(text, row);
        rows.push(row);
        continue;
      }
      const m = CSV_NAME_RE.exec(e.name);
      if (!m) continue;
      const group = m[1] as CalibGroup;
      const text = fs.readFileSync(p, "utf-8");
      const prev = seen.get(text);
      if (prev) {
        if ((prev.group === "l" && group === "r") || (prev.group === "r" && group === "l")) prev.group = "s";
        continue;
      }
      const parsed = parseCsv(text);
      if (!parsed) continue;
      const row: LoadedRow = {
        group,
        dist: Number.isFinite(parsed.dist) ? parsed.dist : Number(m[2]),
        samples: parsed.samples,
        file: path.relative(CSV_DIR, p),
      };
      seen.set(text, row);
      rows.push(row);
    }
  };
  walk(root, 0);
  rows.sort((a, b) => GROUP_ORDER.indexOf(a.group) - GROUP_ORDER.indexOf(b.group) || a.dist - b.dist);
  return rows;
}

function timestamp(): string {
  const d = new Date();
  const p = (n: number) => String(n).padStart(2, "0");
  return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
}

// 同じ (group, dist) の行は1ファイルにまとめる(旧 csv と同じ1位置1ファイル)。
// s(左右同時)は旧手順どおり l_ と r_ の両方に同じ内容を書く。
// スイープ行は1本ずつ f_sweep<n>.csv に、行ごとの絶対距離つきで書く。
export function saveCalibSession(
  rows: { group: CalibGroup; dist: number; samples: number[][]; offsets?: number[] }[],
): string {
  const dirName = `calib_${timestamp()}`;
  const dir = path.join(CSV_DIR, dirName);
  fs.mkdirSync(dir, { recursive: true });
  const merged = new Map<string, { group: CalibGroup; dist: number; samples: number[][] }>();
  let sweepNo = 0;
  for (const r of rows) {
    if (!r.samples.length) continue;
    if (r.offsets) {
      sweepNo += 1;
      const dists = r.offsets.map((o) => r.dist + o);
      fs.writeFileSync(path.join(dir, `f_sweep${sweepNo}.csv`), rowToCsv(r.dist, r.samples, dists), "utf-8");
      continue;
    }
    for (const group of r.group === "s" ? (["l", "r"] as const) : [r.group]) {
      const key = `${group}_${r.dist}`;
      const cur = merged.get(key);
      if (cur) cur.samples.push(...r.samples);
      else merged.set(key, { group, dist: r.dist, samples: [...r.samples] });
    }
  }
  for (const [key, r] of merged) {
    fs.writeFileSync(path.join(dir, `${key}.csv`), rowToCsv(r.dist, r.samples), "utf-8");
  }
  return dirName;
}

export function readSensorGains(): Partial<Record<TargetKey, Gain>> {
  const doc = loadYaml(fs.readFileSync(SENSOR_YAML_PATH, "utf-8")) as Record<string, unknown>;
  const gain = (doc?.gain ?? {}) as Record<string, unknown>;
  const out: Partial<Record<TargetKey, Gain>> = {};
  for (const k of TARGET_KEYS) {
    const v = gain[k];
    if (Array.isArray(v) && v.length >= 2) out[k] = [Number(v[0]), Number(v[1])];
  }
  return out;
}

const fmt = (v: number) => v.toFixed(6);

// sensor.yaml はコメントだらけなので YAML の parse→dump はしない
// (lib/test-templates.ts と同じ方針)。gain: ブロック内の、コメントアウト
// されていない "KEY: [a, b]" 行の値部分だけを置換し、行末コメントは残す。
export function patchSensorYaml(gains: Partial<Record<TargetKey, Gain>>): TargetKey[] {
  const text = fs.readFileSync(SENSOR_YAML_PATH, "utf-8");
  const lines = text.split("\n");
  const gainStart = lines.findIndex((l) => /^gain:\s*(#.*)?$/.test(l));
  if (gainStart < 0) throw new Error("sensor.yaml に gain: がありません");
  let gainEnd = lines.length;
  for (let i = gainStart + 1; i < lines.length; i++) {
    if (/^\S/.test(lines[i]) && !/^#/.test(lines[i])) {
      gainEnd = i;
      break;
    }
  }
  const patched: TargetKey[] = [];
  for (const [key, g] of Object.entries(gains) as [TargetKey, Gain][]) {
    if (!TARGET_KEYS.includes(key) || !g.every(Number.isFinite)) continue;
    const re = new RegExp(`^(\\s+${key}:\\s*)\\[[^\\]]*\\](.*)$`);
    let found = false;
    for (let i = gainStart + 1; i < gainEnd; i++) {
      const m = re.exec(lines[i]);
      if (!m) continue;
      lines[i] = `${m[1]}[${fmt(g[0])}, ${fmt(g[1])}]${m[2]}`;
      found = true;
      break;
    }
    if (!found) throw new Error(`sensor.yaml の gain に ${key} の行がありません`);
    patched.push(key);
  }
  const next = lines.join("\n");
  loadYaml(next); // 壊していないことだけ確認
  fs.writeFileSync(SENSOR_YAML_PATH, next, "utf-8");
  return patched;
}
