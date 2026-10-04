import { execFileSync, spawnSync } from "node:child_process";
import fs from "node:fs";
import path from "node:path";
import { dump as dumpYaml, load as loadYaml } from "js-yaml";
import {
  MACHINE_COLORS,
  MACHINE_HEADER,
  MACHINE_ID_RE,
  type Machine,
  type MachineRegistry,
  type SpecificMap,
} from "./machine-shared";
import { CONSOLE_STATE_FILE, LEGACY_PROFILE_DIR, MACHINES_DIR, MACHINES_YAML, REPO_ROOT } from "./paths";

// 機体(個体)の登録簿と、機体ごとのパラメータの置き場所。サーバー専用。
//
// 回路とファームは全機体で同じ。違うのはパラメータだけなので、機体ごとに
// machines/<id>/profile/ に一式(以前の profile/ と同じ並び)を持たせる。
// サーバーは「今の機体」を覚えない: どの機体のファイルを読む・書く・送るかは
// 毎回リクエストで受け取る(画面に出ている機体と、実際に触る機体がずれないように)。

const REGISTRY_HEADER = `# 機体の登録簿。Param Console(ヘッダーの「機体設定」と「機体比較」)が書き換える。手で直してもよい。
#   machines/<id>/profile/ … その機体のパラメータ一式(以前の profile/ と同じ並び)
#   serials  … その機体の基板の USB シリアル番号。つなぐと Param Console が機体を自動で見分ける
#              (手で書くときは "..." で囲む)
#   default  … 機体を指定しないスクリプト(update_param.sh・check_*.py など)が使う機体
#   specific … 機体ごとに違ってよい値(機体比較で「固有」に分ける)。
#              ファイル名(hf/t_*.yaml のように * も可): true = ファイル全体 / キーの一覧
`;

let cache: { mtimeMs: number; registry: MachineRegistry } | null = null;

function normalize(raw: unknown): MachineRegistry {
  const doc = (raw ?? {}) as { default?: unknown; machines?: unknown; specific?: unknown };
  const machines: Machine[] = [];
  if (Array.isArray(doc.machines)) {
    for (const m of doc.machines as Record<string, unknown>[]) {
      const id = String(m?.id ?? "");
      if (!MACHINE_ID_RE.test(id) || machines.some((x) => x.id === id)) continue;
      machines.push({
        id,
        label: typeof m.label === "string" && m.label.trim() ? m.label.trim() : id,
        color:
          typeof m.color === "string" && /^#[0-9a-f]{6}$/i.test(m.color)
            ? m.color
            : MACHINE_COLORS[machines.length % MACHINE_COLORS.length],
        serials: Array.isArray(m.serials) ? m.serials.map((s) => String(s)).filter((s) => s.length > 0) : [],
        ...(typeof m.note === "string" && m.note ? { note: m.note } : {}),
      });
    }
  }
  const specific: SpecificMap = {};
  if (doc.specific && typeof doc.specific === "object") {
    for (const [file, rule] of Object.entries(doc.specific as Record<string, unknown>)) {
      if (rule === true) specific[file] = true;
      else if (Array.isArray(rule)) specific[file] = rule.map((p) => String(p));
    }
  }
  const def = typeof doc.default === "string" && machines.some((m) => m.id === doc.default) ? doc.default : null;
  return { default: def ?? machines[0]?.id ?? null, machines, specific };
}

export function readRegistry(): MachineRegistry {
  let mtimeMs: number;
  try {
    mtimeMs = fs.statSync(MACHINES_YAML).mtimeMs;
  } catch {
    return { default: null, machines: [], specific: {} };
  }
  if (cache && cache.mtimeMs === mtimeMs) return cache.registry;
  const registry = normalize(loadYaml(fs.readFileSync(MACHINES_YAML, "utf-8")));
  cache = { mtimeMs, registry };
  return registry;
}

function writeRegistry(registry: MachineRegistry): void {
  const specific: SpecificMap = {};
  for (const file of Object.keys(registry.specific).sort()) {
    const rule = registry.specific[file];
    if (rule === true) specific[file] = true;
    else if (rule.length > 0) specific[file] = [...new Set(rule)].sort();
  }
  const body = dumpYaml(
    {
      default: registry.default,
      machines: registry.machines.map((m) => ({
        id: m.id,
        label: m.label,
        color: m.color,
        serials: m.serials,
        ...(m.note ? { note: m.note } : {}),
      })),
      specific,
    },
    { lineWidth: 200 },
  );
  fs.mkdirSync(path.dirname(MACHINES_YAML), { recursive: true });
  fs.writeFileSync(MACHINES_YAML, REGISTRY_HEADER + body, "utf-8");
  cache = null;
}

export function getMachine(id: string): Machine {
  const m = readRegistry().machines.find((x) => x.id === id);
  if (!m) throw new Error(`機体 "${id}" は登録されていません`);
  return m;
}

// 機体のパラメータの場所。登録簿にある id だけを通すので、外へは出られない。
export function profileDir(machineId: string): string {
  return path.join(MACHINES_DIR, getMachine(machineId).id, "profile");
}

export function machineForSerial(serial: string | null | undefined): Machine | null {
  if (!serial) return null;
  return readRegistry().machines.find((m) => m.serials.includes(serial)) ?? null;
}

// API ルート用: リクエストが指している機体(ヘッダーか ?machine=)。無ければエラーにする
// (既定の機体へ黙って落とすと、別の機体のファイルを書き換えかねない)。
export function machineOfRequest(request: Request): string {
  // ?machine= があればそれ(編集画面は「今の機体」以外のファイルも開く)、無ければヘッダー
  const url = new URL(request.url);
  const id = url.searchParams.get("machine") ?? request.headers.get(MACHINE_HEADER);
  if (!id) throw new Error("機体が指定されていません(ページを読み込み直してください)");
  return getMachine(id).id;
}

// ===== 追加・変更 =====

export interface GitSource {
  ref: string; // ブランチ名(origin/xxx を含む)
  date: string; // 最後のコミットの日付
  hash: string; // 短いコミット番号(同じ名前のローカル / origin を見分ける)
  subject: string;
}

const LEGACY_PROFILE_IN_REPO = "tools/param_tuner/profile";

function git(args: string[]): string {
  return execFileSync("git", ["-C", REPO_ROOT, ...args], { encoding: "utf-8", maxBuffer: 64 * 1024 * 1024 });
}

// 機体を分ける前の置き場所(tools/param_tuner/profile)を持つブランチ。
// ブランチで機体を分けていたときのパラメータを、そのまま 1 機体として取り込むため。
export function listGitSources(): GitSource[] {
  let out: string;
  try {
    out = git([
      "for-each-ref",
      "--format=%(refname:short)\t%(committerdate:short)\t%(objectname:short)\t%(subject)",
      "refs/heads",
      "refs/remotes",
    ]);
  } catch {
    return [];
  }
  const sources: GitSource[] = [];
  for (const line of out.split("\n")) {
    const [ref, date, hash, subject] = line.split("\t");
    if (!ref || ref.endsWith("/HEAD")) continue;
    try {
      git(["cat-file", "-e", `${ref}:${LEGACY_PROFILE_IN_REPO}/hardware.yaml`]);
    } catch {
      continue;
    }
    sources.push({ ref, date: date ?? "", hash: hash ?? "", subject: subject ?? "" });
  }
  return sources.sort((a, b) => b.date.localeCompare(a.date));
}

export function legacyProfileExists(): boolean {
  return fs.existsSync(path.join(LEGACY_PROFILE_DIR, "hardware.yaml"));
}

export type MachineSource =
  | { type: "machine"; id: string } // 登録済みの機体をコピー
  | { type: "git"; ref: string } // ブランチの tools/param_tuner/profile を取り込む
  | { type: "legacy" }; // 作業ツリーに残っている tools/param_tuner/profile を取り込む

export function createMachine(input: {
  id: string;
  label?: string;
  color?: string;
  source: MachineSource;
  serial?: string | null; // 接続中の基板をこの機体に登録する
}): Machine {
  const id = input.id.trim();
  if (!MACHINE_ID_RE.test(id)) {
    throw new Error("機体の名前(フォルダ名)は英数字・_・- で、先頭は英数字にしてください");
  }
  const registry = readRegistry();
  if (registry.machines.some((m) => m.id === id)) throw new Error(`機体 "${id}" は既にあります`);
  const dest = path.join(MACHINES_DIR, id, "profile");
  if (fs.existsSync(dest)) throw new Error(`machines/${id}/profile が既にあります(登録簿に無いフォルダ)`);

  const src = input.source;
  if (src.type === "machine") {
    const from = profileDir(src.id);
    fs.mkdirSync(path.dirname(dest), { recursive: true });
    fs.cpSync(from, dest, { recursive: true });
  } else if (src.type === "legacy") {
    if (!legacyProfileExists()) throw new Error("tools/param_tuner/profile がありません");
    fs.mkdirSync(path.dirname(dest), { recursive: true });
    fs.cpSync(LEGACY_PROFILE_DIR, dest, { recursive: true });
  } else {
    if (!listGitSources().some((s) => s.ref === src.ref)) {
      throw new Error(`ブランチ "${src.ref}" に ${LEGACY_PROFILE_IN_REPO} がありません`);
    }
    const tar = spawnSync("git", ["-C", REPO_ROOT, "archive", "--format=tar", src.ref, LEGACY_PROFILE_IN_REPO], {
      maxBuffer: 256 * 1024 * 1024,
    });
    if (tar.status !== 0) throw new Error(`git archive に失敗しました: ${tar.stderr?.toString().trim()}`);
    fs.mkdirSync(dest, { recursive: true });
    const depth = LEGACY_PROFILE_IN_REPO.split("/").length;
    const untar = spawnSync("tar", ["-x", `--strip-components=${depth}`, "-C", dest], { input: tar.stdout });
    if (untar.status !== 0) {
      fs.rmSync(path.join(MACHINES_DIR, id), { recursive: true, force: true });
      throw new Error(`取り込みに失敗しました: ${untar.stderr?.toString().trim()}`);
    }
  }

  const machine: Machine = {
    id,
    label: input.label?.trim() || id,
    color:
      input.color && /^#[0-9a-f]{6}$/i.test(input.color)
        ? input.color
        : (MACHINE_COLORS.find((c) => !registry.machines.some((m) => m.color === c)) ??
          MACHINE_COLORS[registry.machines.length % MACHINE_COLORS.length]),
    serials: [],
  };
  const next: MachineRegistry = { ...registry, machines: [...registry.machines, machine] };
  if (input.serial) assignSerialIn(next, id, input.serial);
  if (!next.default) next.default = id;
  writeRegistry(next);
  return getMachine(id);
}

export function updateMachine(id: string, patch: { label?: string; color?: string; note?: string }): void {
  getMachine(id);
  const registry = readRegistry();
  const machines = registry.machines.map((m) => {
    if (m.id !== id) return m;
    const next = { ...m };
    if (typeof patch.label === "string" && patch.label.trim()) next.label = patch.label.trim();
    if (typeof patch.color === "string" && /^#[0-9a-f]{6}$/i.test(patch.color)) next.color = patch.color;
    if (typeof patch.note === "string") next.note = patch.note;
    return next;
  });
  writeRegistry({ ...registry, machines });
}

function assignSerialIn(registry: MachineRegistry, id: string, serial: string): void {
  // 1 枚の基板は 1 機体だけ
  registry.machines = registry.machines.map((m) => ({
    ...m,
    serials: m.id === id ? [...new Set([...m.serials, serial])] : m.serials.filter((s) => s !== serial),
  }));
}

export function assignSerial(id: string, serial: string): void {
  getMachine(id);
  if (!serial) throw new Error("シリアル番号がありません");
  const registry = { ...readRegistry() };
  assignSerialIn(registry, id, serial);
  writeRegistry(registry);
}

export function removeSerial(id: string, serial: string): void {
  const registry = readRegistry();
  writeRegistry({
    ...registry,
    machines: registry.machines.map((m) =>
      m.id === id ? { ...m, serials: m.serials.filter((s) => s !== serial) } : m,
    ),
  });
}

export function setDefaultMachine(id: string): void {
  getMachine(id);
  writeRegistry({ ...readRegistry(), default: id });
}

// 「固有」の登録。paths = true でファイル全体、配列でキーを足す / 外す。
export function setSpecific(file: string, paths: true | string[], on: boolean): void {
  if (!/^[\w.*/-]+\.yaml$/.test(file) || file.includes("..")) throw new Error("不正なファイル名です");
  const registry = readRegistry();
  const specific: SpecificMap = { ...registry.specific };
  const cur = specific[file];
  if (paths === true) {
    if (on) specific[file] = true;
    else delete specific[file];
  } else {
    if (cur === true) {
      // ファイル全体が固有のときにキーを外すことはできない(全体の登録を先に外す)
      if (!on) throw new Error("ファイル全体が固有になっています。先にファイル全体の指定を外してください");
      return;
    }
    const set = new Set(cur ?? []);
    for (const p of paths) {
      if (on) set.add(p);
      else set.delete(p);
    }
    if (set.size > 0) specific[file] = [...set];
    else delete specific[file];
  }
  writeRegistry({ ...registry, specific });
}

// ===== git に入れない状態(.console_state.json) =====

interface ConsoleState {
  // 基板(USB シリアル番号)ごとに、最後に送れたファイルの中身のハッシュ
  sent?: Record<string, Record<string, string>>;
  // ログ(csv)を出した機体
  logs?: Record<string, string>;
}

let stateCache: ConsoleState | null = null;

function readState(): ConsoleState {
  if (stateCache) return stateCache;
  try {
    stateCache = JSON.parse(fs.readFileSync(CONSOLE_STATE_FILE, "utf-8")) as ConsoleState;
  } catch {
    stateCache = {};
  }
  return stateCache;
}

function writeState(state: ConsoleState): void {
  stateCache = state;
  try {
    fs.writeFileSync(CONSOLE_STATE_FILE, JSON.stringify(state), "utf-8");
  } catch {
    // 覚えられなくても送信・受信そのものは続ける
  }
}

export function getSentHashes(serial: string): Record<string, string> {
  return readState().sent?.[serial] ?? {};
}

export function recordSent(serial: string, remoteName: string, hash: string): void {
  const state = readState();
  const sent = { ...(state.sent ?? {}) };
  sent[serial] = { ...(sent[serial] ?? {}), [remoteName]: hash };
  writeState({ ...state, sent });
}

export function recordLogMachine(fileName: string, machineId: string): void {
  const state = readState();
  writeState({ ...state, logs: { ...(state.logs ?? {}), [fileName]: machineId } });
}

export function getLogMachines(): Record<string, string> {
  return readState().logs ?? {};
}
