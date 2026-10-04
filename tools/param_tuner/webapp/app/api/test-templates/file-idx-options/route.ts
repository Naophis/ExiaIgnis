import { NextRequest, NextResponse } from "next/server";
import { machineOf } from "@/lib/api-util";
import { readFileIdxOptions } from "@/lib/test-templates";

export const runtime = "nodejs";

export async function GET(request: NextRequest) {
  try {
    return NextResponse.json({ options: readFileIdxOptions(machineOf(request)) });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
