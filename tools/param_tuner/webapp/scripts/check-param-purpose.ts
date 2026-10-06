// 用途別パラメータの検査。ファイルは書き換えない(メモリ上だけ)。
//
//   cd tools/param_tuner/webapp && npx --yes tsx scripts/check-param-purpose.ts [-v]
//
// 機体ごとに:
//   1. 分類(tools/param_tuner/param_groups.yaml)が読めること。
//   2. 未分類のキー(yaml にあって分類に無い)と、分類にあって yaml に無いキーを数える
//      (0 でなければ一覧を出して失敗にする)。
//   3. 値の書き換え(lib/yaml-patch.ts の setValueText)を全部の葉で試す:
//      同じ文字を入れ直すとファイルが 1 文字も変わらないこと、別の値を入れると
//      そのキーだけが変わり、コメント行が 1 行も減らないこと。
// param_groups.yaml・lib/yaml-outline.ts・setValueText を変えたら回す。
import fs from "node:fs";
import path from "node:path";
import { load as loadYaml } from "js-yaml";
import { parseCatalog, resolveLayout, sidePartner } from "../lib/param-purpose-shared";
import { canonical, getAt, withValueAt } from "../lib/yaml-compare";
import { outlineYaml, type Outline } from "../lib/yaml-outline";
import { setValueText } from "../lib/yaml-patch";

const ROOT = path.join(__dirname, "..", "..");
const verbose = process.argv.includes("-v");
const commentLines = (t: string) => t.split("\n").filter((l) => l.trimStart().startsWith("#")).length;

const catalog = parseCatalog(loadYaml(fs.readFileSync(path.join(ROOT, "param_groups.yaml"), "utf-8")));
const reg = loadYaml(fs.readFileSync(path.join(ROOT, "machines.yaml"), "utf-8")) as { machines?: { id: string }[] };
let failed = 0;

for (const m of reg.machines ?? []) {
  const dir = path.join(ROOT, "machines", m.id, "profile");
  const outlines: Record<string, Outline | null> = {};
  const texts: Record<string, string> = {};
  for (const rel of new Set(Object.values(catalog.files))) {
    const p = path.join(dir, rel);
    if (!fs.existsSync(p)) {
      outlines[rel] = null;
      continue;
    }
    texts[rel] = fs.readFileSync(p, "utf-8");
    outlines[rel] = outlineYaml(texts[rel]);
  }
  const layout = resolveLayout(catalog, outlines);
  const leafCount = Object.values(outlines).reduce((n, o) => n + (o?.leaves.length ?? 0), 0);
  let shown = 0;
  let tables = 0;
  for (const p of layout.purposes) {
    let n = 0;
    for (const s of p.sections) {
      for (const e of s.entries) {
        if (e.type === "leaf") n++;
        else {
          n += e.rows.length;
          tables++;
        }
      }
    }
    shown += n;
    if (verbose) console.log(`  ${p.label}: ${n}`);
  }
  console.log(
    `${m.id}: 葉 ${leafCount} / 用途に出る ${shown}(表 ${tables})/ 未分類 ${layout.unclassified.length} / yaml に無い ${layout.unmatched.length}`,
  );
  for (const u of layout.unclassified) console.log(`  未分類: ${u.file}  ${u.path}`);
  for (const u of layout.unmatched) console.log(`  yaml に無い: ${u}`);
  failed += layout.unclassified.length + layout.unmatched.length;

  // 書き換えの往復
  let tried = 0;
  let pairs = 0;
  for (const [rel, outline] of Object.entries(outlines)) {
    if (!outline) continue;
    const text = texts[rel];
    const doc = loadYaml(text);
    const names = new Set(outline.leaves.map((l) => l.path));
    for (const leaf of outline.leaves) {
      // 読んだ値の文字が、読み直した値と合っていること
      const want = canonical(getAt(doc, leaf.segs));
      if (leaf.kind === "other") {
        console.log(`  直せない形: ${rel} ${leaf.path}`);
        failed++;
        continue;
      }
      const name = String(leaf.segs[leaf.segs.length - 1]);
      const sp = sidePartner(name);
      if (sp?.side === "l" && names.has([...leaf.segs.slice(0, -1), sp.partner].join("."))) pairs++;
      try {
        const got = canonical((loadYaml(`v: ${leaf.raw}`) as { v: unknown }).v ?? null);
        if (got !== want) throw new Error(`値の文字が合いません: ${leaf.raw} → ${got}(yaml は ${want})`);
        const same = setValueText(text, leaf.segs, leaf.raw);
        if (same !== text) throw new Error("同じ値を入れ直したらファイルが変わりました");
        const next = leaf.kind === "list" ? `[${[...(leaf.items ?? []), "7"].join(", ")}]` : "123.5";
        const changed = setValueText(text, leaf.segs, next);
        const nextValue = (loadYaml(`v: ${next}`) as { v: unknown }).v;
        if (canonical(loadYaml(changed)) !== canonical(withValueAt(doc, leaf.segs, nextValue))) {
          throw new Error("ほかのキーまで変わりました");
        }
        if (commentLines(changed) !== commentLines(text)) throw new Error("コメント行の数が変わりました");
        if (changed.split("\n").length !== text.split("\n").length) throw new Error("行数が変わりました");
        tried++;
      } catch (err) {
        console.log(`  書き換え失敗: ${rel} ${leaf.path}: ${(err as Error).message}`);
        failed++;
      }
    }
  }
  console.log(`  書き換えの往復 ${tried} 個 OK、左右の組 ${pairs}`);
}

if (failed > 0) {
  console.log(`NG: ${failed} 件`);
  process.exit(1);
}
console.log("OK");
