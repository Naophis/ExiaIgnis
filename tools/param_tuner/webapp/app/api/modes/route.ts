import { NextRequest, NextResponse } from "next/server";
import { machineOf } from "@/lib/api-util";
import { serialManager } from "@/lib/serial-manager";

export const runtime = "nodejs";

export async function GET(request: NextRequest) {
  try {
    return NextResponse.json({ modes: serialManager.listModes(machineOf(request)) });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
