import { NextResponse } from "next/server";
import { serialManager } from "@/lib/serial-manager";

export const runtime = "nodejs";

// Re-arms auto-connect (e.g. after a manual disconnect). Connection itself
// happens automatically as soon as a matching ttyACM* device is found.
export async function POST() {
  serialManager.enableAutoConnect();
  return NextResponse.json({ ok: true });
}
