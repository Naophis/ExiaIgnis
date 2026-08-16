import { NextRequest, NextResponse } from "next/server";
import { serialManager, type SendScope } from "@/lib/serial-manager";

export const runtime = "nodejs";

export async function GET(request: NextRequest) {
  const mode = request.nextUrl.searchParams.get("mode");
  const scope = request.nextUrl.searchParams.get("scope") as SendScope | null;
  const file = request.nextUrl.searchParams.get("file");
  if (!mode || !scope || !file) {
    return NextResponse.json({ error: "mode, scope, file is required" }, { status: 400 });
  }
  try {
    const content = serialManager.readProfileFile(mode, scope, file);
    return NextResponse.json({ content });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}

export async function POST(request: NextRequest) {
  const body = await request.json();
  const mode = body?.mode as string | undefined;
  const scope = body?.scope as SendScope | undefined;
  const file = body?.file as string | undefined;
  const content = body?.content as string | undefined;
  if (!mode || !scope || !file || typeof content !== "string") {
    return NextResponse.json({ error: "mode, scope, file, content is required" }, { status: 400 });
  }
  try {
    serialManager.writeProfileFile(mode, scope, file, content);
    return NextResponse.json({ ok: true });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
