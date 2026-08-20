import { NextResponse } from "next/server";
import { openLogsFolder } from "@/lib/logs";

export const runtime = "nodejs";

export async function POST() {
  try {
    openLogsFolder();
    return NextResponse.json({ ok: true });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
