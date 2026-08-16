import { NextResponse } from "next/server";
import { listLogFiles } from "@/lib/logs";

export const runtime = "nodejs";

export async function GET() {
  return NextResponse.json({ files: listLogFiles() });
}
