import { NextRequest, NextResponse } from "next/server";
import { compareMachines, readFileValues } from "@/lib/machine-compare";

export const runtime = "nodejs";

// GET ?mode=hf            全機体・全ファイルの差(差のあるキーだけ)
// GET ?mode=hf&file=      1 ファイルだけ
// GET ?values=1&file=     1 ファイルの機体ごとの値(編集画面が下書きと比べる)
export async function GET(request: NextRequest) {
  const sp = request.nextUrl.searchParams;
  try {
    const file = sp.get("file") ?? undefined;
    if (sp.get("values") === "1") {
      if (!file) throw new Error("file is required");
      return NextResponse.json(readFileValues(file));
    }
    return NextResponse.json(compareMachines(sp.get("mode") ?? "hf", file));
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
