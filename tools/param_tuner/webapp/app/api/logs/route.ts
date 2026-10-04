import { NextRequest, NextResponse } from "next/server";
import { optionalMachineOf } from "@/lib/api-util";
import { listLogFiles } from "@/lib/logs";

export const runtime = "nodejs";

// GET   その機体(ヘッダー / ?machine=)のログと、共通のログ(common: true)。
//       機体の指定が無ければ全機体 + 共通。
export async function GET(request: NextRequest) {
  try {
    return NextResponse.json({ files: listLogFiles(optionalMachineOf(request)) });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
