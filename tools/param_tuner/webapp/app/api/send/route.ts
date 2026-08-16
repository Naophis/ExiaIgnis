import { NextRequest, NextResponse } from "next/server";
import { serialManager, type SendScope } from "@/lib/serial-manager";

export const runtime = "nodejs";

export async function POST(request: NextRequest) {
  const body = await request.json();
  const mode = body?.mode;
  if (!mode || typeof mode !== "string") {
    return NextResponse.json({ error: "mode is required" }, { status: 400 });
  }

  try {
    if (body.all) {
      await serialManager.sendAll(mode);
    } else {
      const scope = body.scope as SendScope | undefined;
      const file = body.file as string | undefined;
      if (!scope || !file) {
        return NextResponse.json({ error: "scope and file are required" }, { status: 400 });
      }
      await serialManager.sendFile(mode, scope, file);
    }
    return NextResponse.json({ ok: true });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 500 });
  }
}
