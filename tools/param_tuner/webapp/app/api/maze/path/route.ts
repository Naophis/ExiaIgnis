import { NextRequest, NextResponse } from "next/server";
import { checkWalls } from "@/lib/maze";
import { readExecOptions, runPathSim } from "@/lib/path-sim";

export const runtime = "nodejs";

// GET                                     走行パラメータの選択肢(run_prf の exec_prof)
export async function GET() {
  try {
    return NextResponse.json({ options: readExecOptions() });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}

// POST {walls, goals, exec, direction}   MainTask::path_run() の経路生成を再現(tools/path_sim)
export async function POST(request: NextRequest) {
  const body = await request.json();
  try {
    const walls = checkWalls(body?.walls);
    const goals = Array.isArray(body?.goals) ? body.goals : null;
    const exec = Number(body?.exec ?? 0);
    const direction = body?.direction === "left" ? "left" : "right";
    return NextResponse.json(await runPathSim({ walls, goals, exec, direction }));
  } catch (err) {
    return NextResponse.json({ ok: false, error: (err as Error).message, log: "" }, { status: 500 });
  }
}
