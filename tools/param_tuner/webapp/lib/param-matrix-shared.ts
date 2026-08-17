// Pure types shared between the server-only lib/param-matrix.ts (touches
// node:fs) and client components. Keep this file free of Node built-ins so
// client bundles don't pull fs/path/js-yaml into the browser bundle.

export type StatusCell = "o" | "x" | null;

export interface VelBlock {
  v: number;
  a: number;
  d: number;
  w0: number;
  w1: number;
  a2: number;
}

export interface StatusRow {
  large90: StatusCell;
  orval180: StatusCell;
  dia45: StatusCell;
  dia45_2: StatusCell;
  dia135: StatusCell;
  dia135_2: StatusCell;
  dia90: StatusCell;
}

// One column of the spreadsheet == one speed profile ("t_<vMax>.hf").
// suction/large/orval/dia45/dia45_2/dia135/dia135_2/dia90 are *indices into
// this same row array* (profile_idx in profiles.yaml), not raw values.
export interface ParamMatrixRow {
  vMax: number;
  suction: number;
  large: number;
  orval: number;
  dia45: number;
  dia45_2: number;
  dia135: number;
  dia135_2: number;
  dia90: number;
  search: VelBlock;
  fast: VelBlock;
  dia: VelBlock;
  execFast: number;
  execNormal: number;
  execSlow: number;
  status: StatusRow;
  // Reference-only fields from the spreadsheet's mode_idx/profile_idx/LED
  // table (rows 13-19). Never read by param.gs and not reflected in any of
  // the 3 YAMLs - free-form notes the tuner jots down for themselves, kept
  // alongside `status` in param_matrix_status.json.
  profileIdxNote: string;
  led: string;
}

export const STATUS_ROW_KEYS: (keyof StatusRow)[] = [
  "large90",
  "orval180",
  "dia45",
  "dia45_2",
  "dia135",
  "dia135_2",
  "dia90",
];

export const STATUS_LABELS: Record<keyof StatusRow, string> = {
  large90: "Large90",
  orval180: "Orval180",
  dia45: "Dia45",
  dia45_2: "Dia45_2",
  dia135: "Dia135",
  dia135_2: "Dia135_2",
  dia90: "Dia90",
};

// profile_idx reference fields, in the order they're stored/rendered.
export const REF_FIELD_KEYS: (keyof Pick<
  ParamMatrixRow,
  "suction" | "large" | "orval" | "dia45" | "dia45_2" | "dia135" | "dia135_2" | "dia90"
>)[] = ["suction", "large", "orval", "dia45", "dia45_2", "dia135", "dia135_2", "dia90"];

export function emptyStatusRow(): StatusRow {
  return {
    large90: null,
    orval180: null,
    dia45: null,
    dia45_2: null,
    dia135: null,
    dia135_2: null,
    dia90: null,
  };
}

export function emptyVelBlock(): VelBlock {
  return { v: 0, a: 0, d: 0, w0: 0, w1: 0, a2: 0 };
}
