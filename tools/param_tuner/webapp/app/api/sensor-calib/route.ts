import { NextRequest, NextResponse } from "next/server";
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
    if (action === "gains") return NextResponse.json({ gains: readSensorGains() });
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
// POST {action:"apply", gains, send}   sensor.yaml の該当行を置換(+送信)
export async function POST(request: NextRequest) {
  const body = await request.json();
  try {
    if (body?.action === "save") {
      if (!Array.isArray(body.rows)) throw new Error("rows is required");
      const dir = saveCalibSession(body.rows);
      return NextResponse.json({ dir });
    }
    if (body?.action === "apply") {
      const patched = patchSensorYaml(body.gains ?? {});
      if (body.send) {
        try {
          await serialManager.sendFile("hf", "mode", "sensor.yaml");
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
