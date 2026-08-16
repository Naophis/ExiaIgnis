import { NextResponse } from "next/server";
import { serialManager } from "@/lib/serial-manager";

export const runtime = "nodejs";

export async function GET() {
  const ports = await serialManager.listPorts();
  return NextResponse.json({ ports });
}
