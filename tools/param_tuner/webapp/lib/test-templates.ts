import { randomUUID } from "node:crypto";
import fs from "node:fs";
import path from "node:path";
import { load as loadYaml } from "js-yaml";
import {
  TEST_TEMPLATE_KEYS,
  type NamedOption,
  type TestTemplate,
  type TestTemplateKey,
  type TestTemplateValues,
} from "./test-template-shared";

export { TEST_TEMPLATE_KEYS, type TestTemplate, type TestTemplateKey, type TestTemplateValues };

// webapp/ is the Next.js server cwd; tools/param_tuner/ is one level up.
const PARAM_TUNER_ROOT = path.join(process.cwd(), "..");
const PROFILE_DIR = path.join(PARAM_TUNER_ROOT, "profile");
const SYSTEM_YAML_PATH = path.join(PROFILE_DIR, "system.yaml");
const TEMPLATES_PATH = path.join(PROFILE_DIR, "test_templates.json");

// Seeded from the 3 hand-toggled blocks already present in system.yaml's
// test: section (1 active + 2 commented-out alternatives) so existing
// presets aren't lost when this feature is first used.
const SEED_TEMPLATES: TestTemplate[] = [
  {
    id: "seed-normal",
    name: "通常",
    values: { v_max: 400, accl: 2500, decel: -2500, dia_accl: 2500, dia_decel: -2500, dist: 720, suction_active: 0 },
  },
  {
    id: "seed-fast",
    name: "高速",
    values: { v_max: 1500, accl: 42500, decel: -47500, dia_accl: 45000, dia_decel: -47500, dist: 120, suction_active: 2 },
  },
  {
    id: "seed-suction",
    name: "吸引テスト",
    values: { v_max: 11550, accl: 28000, decel: -68000, dist: 720, suction_active: 2 },
  },
];

export function listTestTemplates(): TestTemplate[] {
  if (!fs.existsSync(TEMPLATES_PATH)) {
    writeTemplates(SEED_TEMPLATES);
    return SEED_TEMPLATES;
  }
  return JSON.parse(fs.readFileSync(TEMPLATES_PATH, "utf-8"));
}

function writeTemplates(templates: TestTemplate[]): void {
  fs.mkdirSync(PROFILE_DIR, { recursive: true });
  fs.writeFileSync(TEMPLATES_PATH, JSON.stringify(templates, null, 2), "utf-8");
}

export function saveTestTemplate(name: string, values: TestTemplateValues, id?: string): TestTemplate {
  if (!name.trim()) throw new Error("テンプレート名を入力してください");
  const templates = listTestTemplates();
  const template: TestTemplate = { id: id ?? randomUUID(), name: name.trim(), values };
  const idx = templates.findIndex((t) => t.id === template.id);
  if (idx >= 0) templates[idx] = template;
  else templates.push(template);
  writeTemplates(templates);
  return template;
}

export function deleteTestTemplate(id: string): void {
  writeTemplates(listTestTemplates().filter((t) => t.id !== id));
}

export function getTestTemplate(id: string): TestTemplate {
  const template = listTestTemplates().find((t) => t.id === id);
  if (!template) throw new Error("テンプレートが見つかりません");
  return template;
}

// file_idx indexes into profile/hf/profiles.yaml's `list` array (each entry
// a speed-profile filename like "t_2200.hf"), so its selectable options are
// read from that file rather than hardcoded.
export function readFileIdxOptions(mode = "hf"): NamedOption[] {
  const profilesPath = path.join(PROFILE_DIR, mode, "profiles.yaml");
  const doc = loadYaml(fs.readFileSync(profilesPath, "utf-8")) as { list?: string[] };
  const list = doc.list ?? [];
  return list.map((entry, index) => ({ label: entry.replace(/\.\w+$/, ""), value: index }));
}

function keyLineRegex(key: string): RegExp {
  return new RegExp(`^(\\s*)(${key})(\\s*:\\s*)(-?\\d+(?:\\.\\d+)?)(\\s*)(#.*)?$`);
}

// Most target keys (v_max, dist, sla_type, ...) live inside the test: block,
// but "mode" is a top-level key before it starts. Rather than tracking which
// keys belong to which section, both helpers below scan the whole file per
// key and take the first non-commented match - safe here because each of
// these key names only appears once as a live (uncommented) assignment in
// this file.

// Reads the current *active* value for each key, without modifying anything.
// Used to prefill "quick apply" inputs with what's live right now.
export function readActiveValues(keys: readonly TestTemplateKey[]): TestTemplateValues {
  const lines = fs.readFileSync(SYSTEM_YAML_PATH, "utf-8").split("\n");
  const remaining = new Set(keys);
  const values: TestTemplateValues = {};

  for (const line of lines) {
    if (remaining.size === 0) break;
    if (line.trimStart().startsWith("#")) continue;

    for (const key of remaining) {
      const m = line.match(keyLineRegex(key));
      if (!m) continue;
      values[key] = Number(m[4]);
      remaining.delete(key);
      break;
    }
  }

  return values;
}

const MODE_LINE_RE = /^(#\s?)?mode\s*:\s*(-?\d+)/;
const MODE_OPTION_RE = /^(#\s?)?mode\s*:\s*(-?\d+)\s*#\s*(.*)$/;

// Reads the full "mode" menu straight from system.yaml's own comments (both
// the active line and every "# mode: N # <description>" alternative), so
// the option list can't drift out of sync with what's actually documented
// there. Some descriptions use ":" to mark a short name followed by longer
// detail (e.g. "wall off: search_mode(1)->..."); keep only the part before
// it so long descriptions don't blow up the button width in the UI.
export function readModeOptions(): NamedOption[] {
  const lines = fs.readFileSync(SYSTEM_YAML_PATH, "utf-8").split("\n");
  const options: NamedOption[] = [];
  for (const line of lines) {
    const m = line.match(MODE_OPTION_RE);
    if (!m) continue;
    const value = Number(m[2]);
    const shortLabel = m[3].split(":")[0].trim();
    options.push({ label: shortLabel || `mode ${value}`, value });
  }
  return options;
}

// "mode" isn't edited like the other keys: it's a menu of ~20 "# mode: N #
// <description>" lines, each carrying its own description comment, with
// exactly one left uncommented as the active choice. Overwriting the active
// line's number in place (like every other key) would leave it stamped with
// the *previous* mode's description - e.g. turning "mode: 0 # メイン" into
// "mode: 16 # メイン" is wrong, since メイン describes mode 0, not 16, and
// the correct, accurately-described "# mode: 16 # system identification
// (para)" line stays behind, commented out and unused.
//
// So instead: comment out whichever line is currently active, and uncomment
// the line matching the requested mode - preserving each line's own
// description text, exactly like flipping the # by hand.
function applyModeToggle(lines: string[], newMode: number): string[] {
  let activeIdx = -1;
  let targetIdx = -1;

  for (let i = 0; i < lines.length; i++) {
    const m = lines[i].match(MODE_LINE_RE);
    if (!m) continue;
    const isCommented = Boolean(m[1]);
    const value = Number(m[2]);
    if (!isCommented && activeIdx === -1) activeIdx = i;
    if (value === newMode && targetIdx === -1) targetIdx = i;
  }

  if (targetIdx === -1) {
    throw new Error(`system.yaml に mode: ${newMode} の行が見つかりませんでした`);
  }
  if (targetIdx === activeIdx) {
    return lines; // already the active mode, nothing to toggle
  }

  const next = [...lines];
  if (activeIdx !== -1) {
    next[activeIdx] = `# ${next[activeIdx]}`;
  }
  next[targetIdx] = next[targetIdx].replace(/^#\s?/, "");
  return next;
}

// Surgically overwrite only the given keys' *active* (non-commented) lines
// in system.yaml. Deliberately text-based rather than a YAML parse+dump
// round-trip: system.yaml carries ~150 lines of comments (goal-preset
// history, alternative test: blocks, AM32 migration notes) that a parser
// would silently discard. Every other line is left untouched.
export function applyTestTemplateToSystemYaml(values: TestTemplateValues): void {
  const content = fs.readFileSync(SYSTEM_YAML_PATH, "utf-8");
  let lines = content.split("\n");

  const remaining = new Set(Object.keys(values) as TestTemplateKey[]);

  if (remaining.has("mode")) {
    lines = applyModeToggle(lines, values.mode!);
    remaining.delete("mode");
  }

  for (let i = 0; i < lines.length && remaining.size > 0; i++) {
    const line = lines[i];
    if (line.trimStart().startsWith("#")) continue; // never touch commented-out lines

    for (const key of remaining) {
      const m = line.match(keyLineRegex(key));
      if (!m) continue;
      const [, indent, k, colonSpacing, , trailingSpace, comment] = m;
      lines[i] = `${indent}${k}${colonSpacing}${values[key]}${trailingSpace}${comment ?? ""}`;
      remaining.delete(key);
      break;
    }
  }

  if (remaining.size > 0) {
    throw new Error(
      `system.yaml に有効な行が見つかりませんでした: ${Array.from(remaining).join(", ")}`
    );
  }

  fs.writeFileSync(SYSTEM_YAML_PATH, lines.join("\n"), "utf-8");
}
