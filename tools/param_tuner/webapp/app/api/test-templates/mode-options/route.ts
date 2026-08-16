import { NextResponse } from "next/server";
import { readModeOptions } from "@/lib/test-templates";

export const runtime = "nodejs";

export async function GET() {
  try {
    return NextResponse.json({ options: readModeOptions() });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
