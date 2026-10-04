import { NextRequest, NextResponse } from "next/server";
import { machineOf } from "@/lib/api-util";
import { serialManager } from "@/lib/serial-manager";

export const runtime = "nodejs";

// GET ?mode=hf(&machine=)   その機体のファイル一覧 + 接続中の基板へ未送信のファイル
export async function GET(request: NextRequest) {
  const mode = request.nextUrl.searchParams.get("mode");
  if (!mode) {
    return NextResponse.json({ error: "mode is required" }, { status: 400 });
  }
  try {
    return NextResponse.json(serialManager.listProfiles(machineOf(request), mode));
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
