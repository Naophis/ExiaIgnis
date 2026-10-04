import { NextRequest, NextResponse } from "next/server";
import { machineOf, sendErrorResponse } from "@/lib/api-util";
import {
  listCalibDirs,
  loadCalibDir,
  patchSensorYaml,
  readSensorGains,
  saveCalibSession,
} from "@/lib/sensor-calib";
import { serialManager } from "@/lib/serial-manager";

export const runtime = "nodejs";

// GET ?action=gains            現在の sensor.yaml の距離換算ゲイン
// GET ?action=dirs             csv/ 配下の l_/r_/f_*.csv を含むディレクトリ
// GET ?action=load&dir=&rec=1  ディレクトリの csv を行として読む
export async function GET(request: NextRequest) {
  const sp = request.nextUrl.searchParams;
  const action = sp.get("action");
  try {
    if (action === "gains") return NextResponse.json({ gains: readSensorGains(machineOf(request)) });
    if (action === "dirs") return NextResponse.json({ dirs: listCalibDirs() });
    if (action === "load") {
      const dir = sp.get("dir");
      if (!dir) return NextResponse.json({ error: "dir is required" }, { status: 400 });
      return NextResponse.json({ rows: loadCalibDir(dir, sp.get("rec") === "1") });
    }
    return NextResponse.json({ error: "unknown action" }, { status: 400 });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}

// POST {action:"save", rows}           csv/calib_<日時>/ に旧形式で保存
// POST {action:"apply", gains, send}   その機体の sensor.yaml の該当行を置換(+送信)
//   force: true = 送り先の基板が別の機体・未登録でも送る(画面で確認済み)
export async function POST(request: NextRequest) {
  const body = await request.json();
  try {
    if (body?.action === "save") {
      if (!Array.isArray(body.rows)) throw new Error("rows is required");
      const dir = saveCalibSession(body.rows);
      return NextResponse.json({ dir });
    }
    if (body?.action === "apply") {
      const machine = machineOf(request);
      // 送るつもりなら、yaml を書き換える前に送り先の基板を確かめる(別の機体の基板なら何も書かない)
      if (body.send) {
        try {
          serialManager.checkSendTarget(machine, body.force === true);
        } catch (err) {
          return sendErrorResponse(err);
        }
      }
      const patched = patchSensorYaml(machine, body.gains ?? {});
      if (body.send) {
        try {
          await serialManager.sendFile(machine, "hf", "mode", "sensor.yaml", body.force === true);
        } catch (err) {
          return NextResponse.json(
            { patched, error: `保存済み・送信失敗: ${(err as Error).message}` },
            { status: 500 },
          );
        }
      }
      return NextResponse.json({ patched });
    }
    return NextResponse.json({ error: "unknown action" }, { status: 400 });
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
}
