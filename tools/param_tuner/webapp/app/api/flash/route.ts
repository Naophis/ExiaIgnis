import { NextResponse } from "next/server";
import { flashDevice } from "@/lib/flash";

export const runtime = "nodejs";

export async function POST() {
  try {
    const result = await flashDevice();
    if (!result.ok) {
      return NextResponse.json({ error: result.output || "flashに失敗しました" }, { status: 400 });
    }
    return NextResponse.json({ ok: true, output: result.output });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
