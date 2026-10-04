import { NextRequest, NextResponse } from "next/server";
import { optionalMachineOf } from "@/lib/api-util";
import { openLogsFolder } from "@/lib/logs";

export const runtime = "nodejs";

// POST   その機体のログのフォルダ(machines/<機体>/logs)を開く。?common=1 か、機体の指定が
//        無ければ共通の logs/。
export async function POST(request: NextRequest) {
  try {
    const common = request.nextUrl.searchParams.get("common") === "1";
    return NextResponse.json({ ok: true, dir: openLogsFolder(common ? null : optionalMachineOf(request)) });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
