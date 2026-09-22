import { NextRequest, NextResponse } from "next/server";
import { listLogFiles, readLogFile } from "@/lib/logs";
import { parseCsv } from "@/lib/trajectory";
import { analyzeTurnExits, summarizeTurnExits, type TurnExitRow } from "@/lib/turn-exit";

export const runtime = "nodejs";

// 複数ログの旋回出口集計(lib/turn-exit.ts を Node 側で回す)。1本 3MB 級の CSV を
// 何本もブラウザへ送るより、サーバー側で解析して行だけ返す方が軽い。
//   GET /api/logs/turn-exit?limit=6          直近 N 本(latest.csv は複製なので除外)
//   GET /api/logs/turn-exit?names=a.csv,b.csv 指定ファイル
// 解析結果は (ファイル名, mtime) でキャッシュする。dev の HMR で消えても再計算するだけ。
const cache = new Map<string, { mtimeMs: number; rows: TurnExitRow[] }>();
const MAX_FILES = 40;

export async function GET(request: NextRequest) {
  const params = request.nextUrl.searchParams;
  const all = listLogFiles().filter((f) => f.name !== "latest.csv");
  let targets = all;
  const names = params.get("names");
  if (names) {
    const wanted = new Set(names.split(",").map((s) => s.trim()).filter(Boolean));
    targets = all.filter((f) => wanted.has(f.name));
  } else {
    const limit = Math.max(1, Math.min(MAX_FILES, parseInt(params.get("limit") ?? "6", 10) || 6));
    targets = all.slice(0, limit);
  }
  if (targets.length > MAX_FILES) {
    return NextResponse.json({ error: `一度に扱えるのは ${MAX_FILES} 本までです` }, { status: 400 });
  }

  const rows: TurnExitRow[] = [];
  const skipped: string[] = [];
  for (const f of targets) {
    const hit = cache.get(f.name);
    if (hit && hit.mtimeMs === f.mtimeMs) {
      rows.push(...hit.rows);
      continue;
    }
    try {
      const parsed = parseCsv(readLogFile(f.name));
      const r = analyzeTurnExits(parsed, { log: f.name.replace(/\.csv$/, "") });
      cache.set(f.name, { mtimeMs: f.mtimeMs, rows: r });
      rows.push(...r);
    } catch {
      skipped.push(f.name);
    }
  }
  return NextResponse.json({
    files: targets.map((f) => f.name),
    skipped,
    rows,
    summary: summarizeTurnExits(rows),
  });
}
