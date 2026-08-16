import { NextResponse } from "next/server";
import { serialManager } from "@/lib/serial-manager";

export const runtime = "nodejs";

export async function POST() {
  serialManager.disconnect();
  return NextResponse.json({ ok: true });
}
