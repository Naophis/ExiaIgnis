import { NextRequest, NextResponse } from "next/server";
import { machineOf } from "@/lib/api-util";
import { checkWalls, listMazeFiles, readMaze, readSystemMaze, saveMazeAs, writeMaze } from "@/lib/maze";
import { mazeSizeOf } from "@/lib/maze-shared";
import { serialManager } from "@/lib/serial-manager";

export const runtime = "nodejs";

// GET ?action=list        迷路ファイル一覧 + system.yaml の goals / maze_size
// GET ?action=read&id=    1 ファイルの壁(.maze の並び)
export async function GET(request: NextRequest) {
  const sp = request.nextUrl.searchParams;
  const action = sp.get("action");
  try {
    if (action === "list") {
      const machine = machineOf(request);
      return NextResponse.json({ files: listMazeFiles(machine), system: readSystemMaze(machine) });
    }
    if (action === "read") {
      const id = sp.get("id");
      if (!id) return NextResponse.json({ error: "id is required" }, { status: 400 });
      return NextResponse.json(readMaze(machineOf(request), id));
    }
    return NextResponse.json({ error: "unknown action" }, { status: 400 });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}

// POST {action:"save", id, walls}     編集用ファイルを上書き
// POST {action:"saveAs", name, walls} maze_logs/<name>.maze へ新規保存
// POST {action:"send", walls, label}  /maze.txt として機体へ送信
export async function POST(request: NextRequest) {
  const body = await request.json();
  try {
    if (body?.action === "save") {
      writeMaze(machineOf(request), String(body.id ?? ""), body.walls);
      return NextResponse.json({ id: body.id });
    }
    if (body?.action === "saveAs") {
      return NextResponse.json({ id: saveMazeAs(String(body.name ?? ""), body.walls) });
    }
    if (body?.action === "send") {
      const walls = checkWalls(body.walls);
      const size = mazeSizeOf(walls.length);
      // ファームは system.yaml の maze_size の並びで読むので、違う大きさを送ると
      // 壁が全部ずれる。
      const { mazeSize } = readSystemMaze(machineOf(request));
      if (mazeSize !== null && mazeSize !== size) {
        throw new Error(`迷路は ${size}x${size}、system.yaml の maze_size は ${mazeSize} です。揃えてから送ってください`);
      }
      try {
        await serialManager.sendMaze(walls, String(body.label ?? "maze"));
      } catch (err) {
        return NextResponse.json({ error: (err as Error).message }, { status: 500 });
      }
      return NextResponse.json({ ok: true });
    }
    return NextResponse.json({ error: "unknown action" }, { status: 400 });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
