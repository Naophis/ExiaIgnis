import { NextRequest, NextResponse } from "next/server";
import { readLogFile } from "@/lib/logs";

export const runtime = "nodejs";

export async function GET(request: NextRequest) {
  const name = request.nextUrl.searchParams.get("name");
  if (!name) {
    return NextResponse.json({ error: "name is required" }, { status: 400 });
  }
  try {
    return new NextResponse(readLogFile(name), {
      headers: { "Content-Type": "text/csv; charset=utf-8" },
    });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
