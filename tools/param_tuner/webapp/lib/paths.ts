import path from "node:path";

// サーバー専用。場所の決まりをここへ集める(各 lib が別々に組み立てていた)。
//
// TOOL_ROOT = tools/param_tuner(Next サーバーの cwd は webapp/ なので 1 つ上)。
// DATA_ROOT = 機体のパラメータ・ログ・迷路の置き場所。普段は TOOL_ROOT と同じ。
//   EXIA_PARAM_TUNER_ROOT を指定すると別の場所を使う(同期・上書きの操作を、
//   本物のパラメータを触らずに試すため)。
export const TOOL_ROOT = path.join(process.cwd(), "..");
export const REPO_ROOT = path.join(TOOL_ROOT, "..", "..");
export const DATA_ROOT = process.env.EXIA_PARAM_TUNER_ROOT
  ? path.resolve(process.env.EXIA_PARAM_TUNER_ROOT)
  : TOOL_ROOT;

export const LOGS_DIR = path.join(DATA_ROOT, "logs");
export const MAZE_LOGS_DIR = path.join(DATA_ROOT, "maze_logs");
export const MAZE_DATA_DIR = path.join(DATA_ROOT, "maze_data");

// 機体ごとのパラメータ: machines/<id>/profile/(以前の profile/ と同じ並び)。
// 登録簿は machines.yaml。読み書きは lib/machines.ts を通す。
export const MACHINES_DIR = path.join(DATA_ROOT, "machines");
export const MACHINES_YAML = path.join(DATA_ROOT, "machines.yaml");
// git に入れない状態(基板ごとの「最後に送った中身」など)
export const CONSOLE_STATE_FILE = path.join(DATA_ROOT, ".console_state.json");

// 機体を分ける前の置き場所。残っていれば「機体を追加」の取り込み元に出す。
export const LEGACY_PROFILE_DIR = path.join(DATA_ROOT, "profile");
