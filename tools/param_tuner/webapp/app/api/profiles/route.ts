import { NextRequest, NextResponse } from "next/server";
import { serialManager } from "@/lib/serial-manager";

export const runtime = "nodejs";

export async function GET(request: NextRequest) {
  const mode = request.nextUrl.searchParams.get("mode");
  if (!mode) {
    return NextResponse.json({ error: "mode is required" }, { status: 400 });
  }
  return NextResponse.json(serialManager.listProfiles(mode));
}
