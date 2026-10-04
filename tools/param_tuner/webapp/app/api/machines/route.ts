import { NextRequest, NextResponse } from "next/server";
import {
  assignSerial,
  createMachine,
  legacyProfileExists,
  listGitSources,
  readRegistry,
  removeSerial,
  setDefaultMachine,
  setSpecific,
  updateMachine,
  type MachineSource,
} from "@/lib/machines";
import { serialManager } from "@/lib/serial-manager";

export const runtime = "nodejs";

// GET                  登録簿(機体の一覧・固有の登録)+ 接続中の基板
// GET ?action=sources  「機体を追加」の取り込み元(ブランチ・以前の profile/)
export async function GET(request: NextRequest) {
  try {
    if (request.nextUrl.searchParams.get("action") === "sources") {
      return NextResponse.json({ git: listGitSources(), legacy: legacyProfileExists() });
    }
    return NextResponse.json({ registry: readRegistry(), board: serialManager.getBoard() });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}

// POST {action:"create", id, label?, color?, source, registerBoard?}  機体を追加(パラメータ一式をコピー)
// POST {action:"update", id, label?, color?, note?}
// POST {action:"assignSerial", id, serial?}   基板を機体に登録(serial 省略 = 接続中の基板)
// POST {action:"removeSerial", id, serial}
// POST {action:"setDefault", id}
// POST {action:"specific", file, paths: true | string[], on}          「固有」の登録 / 解除
export async function POST(request: NextRequest) {
  const body = await request.json();
  try {
    switch (body?.action) {
      case "create": {
        const serial = body.registerBoard ? serialManager.getBoard().serial : null;
        createMachine({
          id: String(body.id ?? ""),
          label: body.label,
          color: body.color,
          source: body.source as MachineSource,
          serial,
        });
        break;
      }
      case "update":
        updateMachine(String(body.id ?? ""), { label: body.label, color: body.color, note: body.note });
        break;
      case "assignSerial": {
        const serial = typeof body.serial === "string" && body.serial ? body.serial : serialManager.getBoard().serial;
        if (!serial) throw new Error("基板がつながっていません");
        assignSerial(String(body.id ?? ""), serial);
        break;
      }
      case "removeSerial":
        removeSerial(String(body.id ?? ""), String(body.serial ?? ""));
        break;
      case "setDefault":
        setDefaultMachine(String(body.id ?? ""));
        break;
      case "specific":
        setSpecific(String(body.file ?? ""), body.paths === true ? true : (body.paths as string[]), body.on !== false);
        break;
      default:
        return NextResponse.json({ error: "unknown action" }, { status: 400 });
    }
    // 基板 ↔ 機体の対応が変わったかもしれないので、接続表示を更新させる
    serialManager.refreshStatus();
    return NextResponse.json({ registry: readRegistry(), board: serialManager.getBoard() });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
