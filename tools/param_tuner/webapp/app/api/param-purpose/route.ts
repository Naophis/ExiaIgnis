import { NextRequest, NextResponse } from "next/server";
import { machineOf } from "@/lib/api-util";
import { applyPurposeEdits, readPurposeData } from "@/lib/param-purpose";
import type { PurposeEdit } from "@/lib/param-purpose-shared";

export const runtime = "nodejs";

// 用途別パラメータ。GET = 分類(param_groups.yaml)を当てはめた一覧 + 値とコメント、
// POST {edits} = 値の書き換え(コメントは残す)。
export async function GET(request: NextRequest) {
  try {
    return NextResponse.json(readPurposeData(machineOf(request)));
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}

export async function POST(request: NextRequest) {
  const body = await request.json();
  const edits = body?.edits as PurposeEdit[] | undefined;
  if (!Array.isArray(edits) || edits.length === 0) {
    return NextResponse.json({ error: "edits is required" }, { status: 400 });
  }
  try {
    return NextResponse.json(applyPurposeEdits(machineOf(request, body?.machine), edits));
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
