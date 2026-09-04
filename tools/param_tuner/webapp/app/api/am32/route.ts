import { NextRequest, NextResponse } from "next/server";
import { serialManager } from "@/lib/serial-manager";

export const runtime = "nodejs";

// "sync"   = am32.yaml を /am32.txt として送信 + AM32WRITE (send_file.py am32sync 相当)
// "write"  = アップロード済みの /am32.txt を AM32WRITE
// "read"   = AM32READ (ESCの現在値をコンソールへダンプ)
export async function POST(request: NextRequest) {
  const body = await request.json();
  const action = body?.action;

  try {
    if (action === "sync") {
      const mode = body?.mode;
      if (!mode || typeof mode !== "string") {
        return NextResponse.json({ error: "mode is required" }, { status: 400 });
      }
      await serialManager.syncAm32(mode);
    } else if (action === "write" || action === "read") {
      await serialManager.runAm32Command(action);
    } else {
      return NextResponse.json({ error: "action must be sync/write/read" }, { status: 400 });
    }
    return NextResponse.json({ ok: true });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 500 });
  }
}
