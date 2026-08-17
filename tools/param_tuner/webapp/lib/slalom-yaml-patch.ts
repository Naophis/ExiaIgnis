// Surgical text-level patch for profile/hf/t_<v>.yaml turn blocks - same
// philosophy as lib/test-templates.ts's system.yaml editor: these files
// carry hand-written comments ("# check" dated markers, commented-out
// alternate tunings - see t_300.yaml) that a full YAML parse+dump would
// silently discard, so only the specific field lines inside the target
// block are rewritten. Pure string in/out - no fs access - so it can patch
// the in-browser editor draft directly, before the user reviews and saves.

import type { SlalomSimResult, TurnType } from "./slalom-sim";

export interface SlalomYamlFields {
  v: string;
  rad: string;
  n: string;
  ang: string;
}

export interface SlalomApplySelection {
  v: boolean;
  ang: boolean;
  n: boolean;
  rad: boolean;
  front: boolean;
  back: boolean;
}

export const APPLY_ALL_SELECTION: SlalomApplySelection = {
  v: true,
  ang: true,
  n: true,
  rad: true,
  front: true,
  back: true,
};

function scalarLineRegex(key: string): RegExp {
  return new RegExp(`^(\\s*)(${key})(\\s*:\\s*)(-?\\d+(?:\\.\\d+)?)(\\s*)(#.*)?$`);
}

function objectLineRegex(key: "front" | "back"): RegExp {
  return new RegExp(`^(\\s*)(${key})(\\s*:\\s*)\\{[^}]*\\}(\\s*)(#.*)?$`);
}

// Finds the [start, end) line-index range of the *first* uncommented
// top-level `type:` block: its header line, plus every following line
// that's blank or indented, stopping at the next column-0 non-blank line (a
// new top-level key, or a comment like "# check" used as a section divider).
function findBlock(lines: string[], type: TurnType): { start: number; end: number } | null {
  const headerRe = new RegExp(`^${type}:\\s*(#.*)?$`);
  const header = lines.findIndex((l) => headerRe.test(l));
  if (header === -1) return null;

  let end = lines.length;
  for (let i = header + 1; i < lines.length; i++) {
    if (lines[i].length > 0 && !/^\s/.test(lines[i])) {
      end = i;
      break;
    }
  }
  return { start: header + 1, end };
}

// Overwrites the first uncommented `key: <number>` line within [start, end);
// appends `  key: value` right before `end` if none exists. Returns the
// (possibly shifted) end index for the next call to chain against.
function setScalarField(lines: string[], start: number, end: number, key: string, value: string): number {
  for (let i = start; i < end; i++) {
    if (lines[i].trimStart().startsWith("#")) continue;
    const m = lines[i].match(scalarLineRegex(key));
    if (!m) continue;
    const [, indent, k, colonSpacing, , trailingSpace, comment] = m;
    lines[i] = `${indent}${k}${colonSpacing}${value}${trailingSpace}${comment ?? ""}`;
    return end;
  }
  lines.splice(end, 0, `  ${key}: ${value}`);
  return end + 1;
}

function setObjectField(lines: string[], start: number, end: number, key: "front" | "back", value: string): number {
  for (let i = start; i < end; i++) {
    if (lines[i].trimStart().startsWith("#")) continue;
    const m = lines[i].match(objectLineRegex(key));
    if (!m) continue;
    const [, indent, k, colonSpacing, trailingSpace, comment] = m;
    lines[i] = `${indent}${k}${colonSpacing}${value}${trailingSpace}${comment ?? ""}`;
    return end;
  }
  lines.splice(end, 0, `  ${key}: ${value}`);
  return end + 1;
}

// Writes only the fields marked true in `selection` (default: all of
// v/ang/pow_n/rad and, when the turn type's offset is implemented,
// front/back) into the yaml text's `type:` block, in place. Throws if the
// block doesn't exist - this only edits an existing turn's parameters, it
// doesn't create a new one.
export function applySimResultToYaml(
  yamlText: string,
  type: TurnType,
  fields: SlalomYamlFields,
  result: SlalomSimResult,
  selection: SlalomApplySelection = APPLY_ALL_SELECTION
): string {
  const lines = yamlText.split("\n");
  const block = findBlock(lines, type);
  if (!block) {
    throw new Error(`YAML内に "${type}:" ブロックが見つかりませんでした`);
  }

  let end = block.end;
  if (selection.v) end = setScalarField(lines, block.start, end, "v", fields.v);
  if (selection.ang) end = setScalarField(lines, block.start, end, "ang", fields.ang);
  if (selection.n) end = setScalarField(lines, block.start, end, "pow_n", fields.n);
  if (selection.rad) end = setScalarField(lines, block.start, end, "rad", fields.rad);

  if (result.offsetSupported) {
    if (selection.front) {
      const front = `{ left: ${result.front.toFixed(4)}, right: ${result.front.toFixed(4)} }`;
      end = setObjectField(lines, block.start, end, "front", front);
    }
    if (selection.back) {
      const back = `{ left: ${result.back.toFixed(4)}, right: ${result.back.toFixed(4)} }`;
      setObjectField(lines, block.start, end, "back", back);
    }
  }

  return lines.join("\n");
}
