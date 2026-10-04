import { NextRequest, NextResponse } from "next/server";
import { machineOf } from "@/lib/api-util";
import { readParamMatrix, writeParamMatrix, type ParamMatrixRow } from "@/lib/param-matrix";

export const runtime = "nodejs";

export async function GET(request: NextRequest) {
  try {
    return NextResponse.json({ rows: readParamMatrix(machineOf(request)) });
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
    writeParamMatrix(machineOf(request), rows, newVMaxFiles);
    return NextResponse.json({ ok: true });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
