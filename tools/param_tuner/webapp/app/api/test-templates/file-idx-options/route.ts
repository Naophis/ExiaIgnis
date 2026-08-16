import { NextResponse } from "next/server";
import { readFileIdxOptions } from "@/lib/test-templates";

export const runtime = "nodejs";

export async function GET() {
  try {
    return NextResponse.json({ options: readFileIdxOptions() });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
