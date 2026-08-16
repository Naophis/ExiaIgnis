import { NextRequest, NextResponse } from "next/server";
import { killPlotJuggler, openInPlotJuggler } from "@/lib/logs";

export const runtime = "nodejs";

export async function POST(request: NextRequest) {
  const body = await request.json();
  const action = body?.action as "open" | "kill" | undefined;
  try {
    if (action === "open") {
      const name = body?.name as string | undefined;
      if (!name) return NextResponse.json({ error: "name is required" }, { status: 400 });
      openInPlotJuggler(name);
      return NextResponse.json({ ok: true });
    }
    if (action === "kill") {
      killPlotJuggler();
      return NextResponse.json({ ok: true });
    }
    return NextResponse.json({ error: "unknown action" }, { status: 400 });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
