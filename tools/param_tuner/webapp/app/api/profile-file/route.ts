import fs from "node:fs";
import { NextRequest, NextResponse } from "next/server";
import { machineOf } from "@/lib/api-util";
import { suggestPropagation } from "@/lib/machine-compare";
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
    const content = serialManager.readProfileFile(machineOf(request), mode, scope, file);
    return NextResponse.json({ content });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}

// 保存。propagate = 変えたキーのうち、ほかの機体も同じ値だったもの
// (画面が「ほかの機体にも入れる」を出す)。
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
    const machine = machineOf(request, body?.machine);
    const filePath = serialManager.resolveYamlPath(machine, mode, scope, file);
    const before = fs.existsSync(filePath) ? fs.readFileSync(filePath, "utf-8") : null;
    serialManager.writeProfileFile(machine, mode, scope, file, content);
    const rel = scope === "base" ? file : `${mode}/${file}`;
    return NextResponse.json({ ok: true, propagate: suggestPropagation(machine, rel, before, content) });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
