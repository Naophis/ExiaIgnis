import { NextRequest, NextResponse } from "next/server";
import { copyWholeFile, syncKeys, undoSync } from "@/lib/machine-compare";
import type { PathSeg } from "@/lib/machine-shared";

export const runtime = "nodejs";

// POST {from, to, file, paths: PathSeg[][]}   from の機体の値を to の機体へ写す(コメントは残す)
// POST {from, to, file, whole: true}          ファイルごと写す
// POST {undo: token}                           直前の書き換えを元に戻す
export async function POST(request: NextRequest) {
  const body = await request.json();
  try {
    if (typeof body?.undo === "string") {
      return NextResponse.json({ ok: true, label: undoSync(body.undo) });
    }
    const from = String(body?.from ?? "");
    const to = String(body?.to ?? "");
    const file = String(body?.file ?? "");
    if (body?.whole === true) return NextResponse.json(copyWholeFile({ from, to, file }));
    if (!Array.isArray(body?.paths)) throw new Error("paths is required");
    return NextResponse.json(syncKeys({ from, to, file, paths: body.paths as PathSeg[][] }));
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
