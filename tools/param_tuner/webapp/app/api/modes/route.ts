import { NextResponse } from "next/server";
import { serialManager } from "@/lib/serial-manager";

export const runtime = "nodejs";

export async function GET() {
  return NextResponse.json({ modes: serialManager.listModes() });
}
