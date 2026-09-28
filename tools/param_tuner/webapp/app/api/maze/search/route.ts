import { NextRequest, NextResponse } from "next/server";
import { checkWalls } from "@/lib/maze";
import { runSearchSim } from "@/lib/search-sim";

export const runtime = "nodejs";

// POST {walls, goals}   SearchController::exec()(mode_num == 0 の探索)を再現(tools/path_sim の search_sim)
export async function POST(request: NextRequest) {
  const body = await request.json();
  try {
    const walls = checkWalls(body?.walls);
    const goals = Array.isArray(body?.goals) ? body.goals : null;
    if (Array.isArray(goals) && goals.length === 0) throw new Error("ゴールがありません(ツールバーの G: でゴールを置いてください)");
    return NextResponse.json(await runSearchSim({ walls, goals }));
  } catch (err) {
    return NextResponse.json({ ok: false, error: (err as Error).message, log: "" }, { status: 500 });
  }
}
