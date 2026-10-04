import { NextRequest, NextResponse } from "next/server";
import { machineOf } from "@/lib/api-util";
import { checkWalls } from "@/lib/maze";
import { mazeIndex, mazeSizeOf, reachableCells } from "@/lib/maze-shared";
import { readExecOptions, runPathSim } from "@/lib/path-sim";

export const runtime = "nodejs";

// GET                                     走行パラメータの選択肢(run_prf の exec_prof)
export async function GET(request: NextRequest) {
  try {
    return NextResponse.json({ options: readExecOptions(machineOf(request)) });
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
    if (Array.isArray(goals) && goals.length === 0) throw new Error("ゴールがありません(ツールバーの G: でゴールを置いてください)");
    if (goals) {
      // 行けないゴールだと path_create が失敗するだけで理由が分からない(maze_data/32_fake.yaml がこれ)
      const size = mazeSizeOf(walls.length);
      const reach = reachableCells(walls, size);
      const ok = goals.some((g: unknown) => Array.isArray(g) && reach[mazeIndex(size, Number(g[0]), Number(g[1]))] === 1);
      if (!ok) throw new Error("スタートからゴールへ行けません(ゴールが壁で閉じています)");
    }
    const exec = Number(body?.exec ?? 0);
    const direction = body?.direction === "left" ? "left" : "right";
    return NextResponse.json(await runPathSim({ machine: machineOf(request), walls, goals, exec, direction }));
  } catch (err) {
    return NextResponse.json({ ok: false, error: (err as Error).message, log: "" }, { status: 500 });
  }
}
