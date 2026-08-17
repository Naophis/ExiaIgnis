import fs from "node:fs";
import path from "node:path";
import { dump as dumpYaml, load as loadYaml } from "js-yaml";
import {
  emptyStatusRow,
  emptyVelBlock,
  REF_FIELD_KEYS,
  type ParamMatrixRow,
  type StatusRow,
  type VelBlock,
} from "./param-matrix-shared";

export {
  emptyStatusRow,
  emptyVelBlock,
  REF_FIELD_KEYS,
  STATUS_LABELS,
  STATUS_ROW_KEYS,
  type ParamMatrixRow,
  type StatusCell,
  type StatusRow,
  type VelBlock,
} from "./param-matrix-shared";

// webapp/ is the Next.js server cwd; tools/param_tuner/ is one level up.
const PARAM_TUNER_ROOT = path.join(process.cwd(), "..");
const PROFILE_DIR = path.join(PARAM_TUNER_ROOT, "profile");
const STATUS_FILE = "param_matrix_status.json";

interface ProfilesYaml {
  list: string[];
  profile_idx: Array<{
    run_param: number;
    suction: number;
    normal: number;
    large: number;
    orval: number;
    dia45: number;
    dia45_2: number;
    dia135: number;
    dia135_2: number;
    dia90: number;
  }>;
}

interface VelProfYaml {
  v_prof: Array<{ search: VelBlock; fast: VelBlock; dia: VelBlock }>;
}

interface RunPrfYaml {
  exec_prof: Array<{ fast: number; normal: number; slow: number }>;
}

interface ColumnNotes {
  status: StatusRow;
  profileIdxNote: string;
  led: string;
}

function readStatusMap(mode: string): Record<number, ColumnNotes> {
  const p = path.join(PROFILE_DIR, mode, STATUS_FILE);
  if (!fs.existsSync(p)) return {};
  const raw = JSON.parse(fs.readFileSync(p, "utf-8")) as Record<string, Partial<ColumnNotes>>;
  const out: Record<number, ColumnNotes> = {};
  for (const [k, v] of Object.entries(raw)) {
    out[Number(k)] = {
      status: v.status ?? emptyStatusRow(),
      profileIdxNote: v.profileIdxNote ?? "",
      led: v.led ?? "",
    };
  }
  return out;
}

function writeStatusMap(mode: string, rows: ParamMatrixRow[]): void {
  const p = path.join(PROFILE_DIR, mode, STATUS_FILE);
  const out: Record<string, ColumnNotes> = {};
  for (const r of rows) {
    out[String(r.vMax)] = { status: r.status, profileIdxNote: r.profileIdxNote, led: r.led };
  }
  fs.mkdirSync(path.dirname(p), { recursive: true });
  fs.writeFileSync(p, JSON.stringify(out, null, 2), "utf-8");
}

// Reads profiles.yaml/vel_prof.yaml/run_prf.yaml (array position == the
// shared "run_param" index they all reference each other by) and joins them
// with the tuning-status JSON (keyed by vMax, so it survives column
// deletes/reorders even though the YAMLs themselves are positional).
export function readParamMatrix(mode = "hf"): ParamMatrixRow[] {
  const modeDir = path.join(PROFILE_DIR, mode);
  const profiles = loadYaml(fs.readFileSync(path.join(modeDir, "profiles.yaml"), "utf-8")) as ProfilesYaml;
  const velProf = loadYaml(fs.readFileSync(path.join(modeDir, "vel_prof.yaml"), "utf-8")) as VelProfYaml;
  const runPrf = loadYaml(fs.readFileSync(path.join(modeDir, "run_prf.yaml"), "utf-8")) as RunPrfYaml;
  const notes = readStatusMap(mode);

  return profiles.list.map((entry, i) => {
    const vMaxMatch = entry.match(/^t_(\d+)\.hf$/);
    const vMax = vMaxMatch ? Number(vMaxMatch[1]) : NaN;
    const p = profiles.profile_idx[i];
    const v = velProf.v_prof[i];
    const e = runPrf.exec_prof[i];
    const n = notes[vMax];
    return {
      vMax,
      suction: p?.suction ?? 0,
      large: p?.large ?? 0,
      orval: p?.orval ?? 0,
      dia45: p?.dia45 ?? 0,
      dia45_2: p?.dia45_2 ?? 0,
      dia135: p?.dia135 ?? 0,
      dia135_2: p?.dia135_2 ?? 0,
      dia90: p?.dia90 ?? 0,
      search: v?.search ?? emptyVelBlock(),
      fast: v?.fast ?? emptyVelBlock(),
      dia: v?.dia ?? emptyVelBlock(),
      execFast: e?.fast ?? 0,
      execNormal: e?.normal ?? 0,
      execSlow: e?.slow ?? 0,
      status: n?.status ?? emptyStatusRow(),
      profileIdxNote: n?.profileIdxNote ?? "",
      led: n?.led ?? "",
    };
  });
}

// Full regenerate (not a surgical text edit): unlike system.yaml, these 3
// files are pure machine-generated data with no hand-written comment
// history to preserve (they're the direct output of param.gs's own
// yamlStringify), so a plain js-yaml dump is safe and matches how they were
// produced in the first place.
export function writeParamMatrix(rows: ParamMatrixRow[], newVMaxFiles: number[] = [], mode = "hf"): void {
  const modeDir = path.join(PROFILE_DIR, mode);
  const n = rows.length;

  const seen = new Set<number>();
  for (const r of rows) {
    if (!Number.isInteger(r.vMax) || r.vMax <= 0) {
      throw new Error(`不正な速度値です: ${r.vMax}`);
    }
    if (seen.has(r.vMax)) {
      throw new Error(`速度が重複しています: ${r.vMax}`);
    }
    seen.add(r.vMax);
    for (const key of REF_FIELD_KEYS) {
      const v = r[key];
      if (!Number.isInteger(v) || v < 0 || v >= n) {
        throw new Error(`${key} の参照インデックスが範囲外です (${v})`);
      }
    }
  }

  const list = rows.map((r) => `t_${r.vMax}.hf`);
  const profile_idx = rows.map((r, i) => ({
    run_param: i,
    suction: r.suction,
    normal: 1,
    large: r.large,
    orval: r.orval,
    dia45: r.dia45,
    dia45_2: r.dia45_2,
    dia135: r.dia135,
    dia135_2: r.dia135_2,
    dia90: r.dia90,
  }));
  fs.writeFileSync(
    path.join(modeDir, "profiles.yaml"),
    dumpYaml({ list, profile_idx_size: list.length, profile_idx }),
    "utf-8"
  );

  const v_prof = rows.map((r) => ({ search: r.search, fast: r.fast, dia: r.dia }));
  fs.writeFileSync(path.join(modeDir, "vel_prof.yaml"), dumpYaml({ v_prof }), "utf-8");

  const exec_prof = rows.map((r) => ({ fast: r.execFast, normal: r.execNormal, slow: r.execSlow }));
  fs.writeFileSync(path.join(modeDir, "run_prf.yaml"), dumpYaml({ exec_prof }), "utf-8");

  writeStatusMap(mode, rows);

  // New columns get a turn-detail file (t_<vMax>.yaml) seeded from the
  // previous last column, so they're immediately editable/sendable instead
  // of missing entirely. Never overwrite an existing hand-tuned file.
  for (const vMax of newVMaxFiles) {
    const dest = path.join(modeDir, `t_${vMax}.yaml`);
    if (fs.existsSync(dest)) continue;
    const idx = rows.findIndex((r) => r.vMax === vMax);
    const templateVMax = idx > 0 ? rows[idx - 1].vMax : undefined;
    if (templateVMax === undefined) continue;
    const templatePath = path.join(modeDir, `t_${templateVMax}.yaml`);
    if (fs.existsSync(templatePath)) {
      fs.copyFileSync(templatePath, dest);
    }
  }
}
