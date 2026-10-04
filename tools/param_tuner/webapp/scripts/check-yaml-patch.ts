// lib/yaml-patch.ts の検査: 機体どうしの差を、ファイルを書き換えずに(メモリ上で)全部写してみて、
// 1 つも失敗せず、写したあと差が残らず、コメント行が減っていないことを確かめる。
//
//   cd tools/param_tuner/webapp && npx --yes tsx scripts/check-yaml-patch.ts [比べる profile のフォルダ ...]
//
// 引数なし: machines.yaml の機体どうしを全部の組で比べる。
// 引数あり: そのフォルダ(profile の並び)と、登録済みの各機体を比べる
//   (例: git archive で取り出した別ブランチの profile)。
// yaml-patch.ts や yaml-compare.ts を変えたら回す。
import fs from "node:fs";
import path from "node:path";
import { load as loadYaml } from "js-yaml";
import { compareTrees } from "../lib/yaml-compare";
import { copyPaths } from "../lib/yaml-patch";

const ROOT = path.join(__dirname, "..", "..");
const BASE_FILES = ["system.yaml", "hardware.yaml", "am32.yaml"];
const MODE = "hf";

function machineDirs(): { name: string; dir: string }[] {
  const reg = loadYaml(fs.readFileSync(path.join(ROOT, "machines.yaml"), "utf-8")) as { machines?: { id: string }[] };
  return (reg.machines ?? []).map((m) => ({ name: m.id, dir: path.join(ROOT, "machines", m.id, "profile") }));
}

function filesOf(dir: string): string[] {
  const out = BASE_FILES.filter((f) => fs.existsSync(path.join(dir, f)));
  const modeDir = path.join(dir, MODE);
  if (fs.existsSync(modeDir)) {
    for (const f of fs.readdirSync(modeDir).sort()) if (f.endsWith(".yaml")) out.push(`${MODE}/${f}`);
  }
  return out;
}

const commentLines = (t: string) => t.split("\n").filter((l) => l.trim().startsWith("#")).length;

let ops = 0;
let failed = 0;
function checkPair(a: { name: string; dir: string }, b: { name: string; dir: string }) {
  for (const f of filesOf(a.dir)) {
    const pa = path.join(a.dir, f);
    const pb = path.join(b.dir, f);
    if (!fs.existsSync(pb)) continue;
    const srcText = fs.readFileSync(pa, "utf-8");
    const dstText = fs.readFileSync(pb, "utf-8");
    const src = loadYaml(srcText);
    const todo = compareTrees([src, loadYaml(dstText)]).entries.filter((e) => e.values[0] !== null);
    if (todo.length === 0) continue;
    const r = copyPaths(srcText, dstText, todo.map((e) => e.segs));
    const left = compareTrees([src, loadYaml(r.text)]).entries.filter((e) => e.values[0] !== null);
    const lostComments = commentLines(r.text) < commentLines(dstText);
    ops += todo.length;
    const bad = r.errors.length > 0 || left.length > 0 || lostComments;
    if (bad) failed++;
    console.log(
      `${bad ? "NG" : "ok"}  ${a.name} -> ${b.name}  ${f.padEnd(20)} 写した ${String(r.applied.length).padStart(3)}  失敗 ${r.errors.length}  残り ${left.length}` +
        (lostComments ? "  コメント行が減った" : ""),
    );
    for (const e of r.errors) console.log(`      ${e.path}: ${e.error}`);
  }
}

const machines = machineDirs();
const extra = process.argv.slice(2).map((dir) => ({ name: path.basename(path.resolve(dir)), dir: path.resolve(dir) }));
const pairs: [typeof machines[number], typeof machines[number]][] = [];
if (extra.length > 0) {
  for (const m of machines) for (const x of extra) pairs.push([m, x], [x, m]);
} else {
  for (const a of machines) for (const b of machines) if (a !== b) pairs.push([a, b]);
}
if (pairs.length === 0) console.log("比べる相手がありません(機体が 1 つだけ。フォルダを引数で渡すと、それと比べる)");
for (const [a, b] of pairs) checkPair(a, b);
console.log(`\n${ops} 個のキーを写して、NG のファイル ${failed}`);
process.exit(failed > 0 ? 1 : 0);
