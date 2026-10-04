import type { PathSeg } from "./machine-shared";
import type { DiffEntry } from "./yaml-compare";

// 機体比較の結果の型(クライアント / サーバー共用)。

export interface CompareEntry extends DiffEntry {
  specific: boolean; // 「固有」(機体ごとに違ってよい)に登録されているか
}

export interface CompareFile {
  file: string; // profile からの相対パス("hardware.yaml" / "hf/offset.yaml")
  present: boolean[]; // 機体の並び順。その機体にファイルがあるか
  errors: (string | null)[]; // 読めなかった理由(構文エラーなど)
  total: number; // 比べた値の数
  entries: CompareEntry[]; // 差のあるキーだけ
  fileSpecific: boolean; // ファイル全体が「固有」
}

export interface CompareResult {
  machines: string[]; // 機体 id(登録簿の並び順)
  files: CompareFile[];
}

// 編集画面用: 1 つのファイルの、機体ごとの読んだ値
export interface FileValues {
  machines: string[];
  values: Record<string, unknown>; // 機体 id → 値(ファイルが無い・読めない機体は入らない)
}

// 保存したとき「ほかの機体も同じ値だった」キー(同じ変更を入れる候補)
export interface PropagateSuggestion {
  machine: string; // 入れる先の機体
  paths: { segs: PathSeg[]; path: string; from: string | null; to: string | null }[];
}

export interface SyncResult {
  applied: string[];
  errors: { path: string; error: string }[];
  undo: string | null; // 元に戻すための番号(何も変えなかったら null)
}
