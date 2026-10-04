import { NextRequest, NextResponse } from "next/server";
import { machineOf, sendErrorResponse } from "@/lib/api-util";
import { serialManager, type SendScope } from "@/lib/serial-manager";

export const runtime = "nodejs";

// POST {machine?, mode, scope, file}        1 ファイル送信
// POST {machine?, mode, all: true}          全て送信
// POST {machine?, mode, unsent: true}       接続中の基板へ未送信のファイルだけ送信
// force: true = 送り先の基板が別の機体・未登録でも送る(画面で確認済み)
export async function POST(request: NextRequest) {
  const body = await request.json();
  const mode = body?.mode;
  if (!mode || typeof mode !== "string") {
    return NextResponse.json({ error: "mode is required" }, { status: 400 });
  }

  try {
    const machine = machineOf(request, body?.machine);
    const force = body?.force === true;
    if (body.all || body.unsent) {
      const count = await serialManager.sendAll(machine, mode, force, body.unsent === true);
      return NextResponse.json({ ok: true, count });
    }
    const scope = body.scope as SendScope | undefined;
    const file = body.file as string | undefined;
    if (!scope || !file) {
      return NextResponse.json({ error: "scope and file are required" }, { status: 400 });
    }
    await serialManager.sendFile(machine, mode, scope, file, force);
    return NextResponse.json({ ok: true, count: 1 });
  } catch (err) {
    return sendErrorResponse(err);
  }
}
