import { NextRequest, NextResponse } from "next/server";
import { readParamMatrix, writeParamMatrix, type ParamMatrixRow } from "@/lib/param-matrix";

export const runtime = "nodejs";

export async function GET() {
  try {
    return NextResponse.json({ rows: readParamMatrix() });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}

export async function POST(request: NextRequest) {
  const body = await request.json();
  const rows = body?.rows as ParamMatrixRow[] | undefined;
  const newVMaxFiles = (body?.newVMaxFiles as number[] | undefined) ?? [];
  if (!rows) {
    return NextResponse.json({ error: "rows is required" }, { status: 400 });
  }
  try {
    writeParamMatrix(rows, newVMaxFiles);
    return NextResponse.json({ ok: true });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
