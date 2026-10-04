import { NextResponse } from "next/server";
import { getMachine, machineOfRequest } from "./machines";
import { SendTargetError } from "./serial-manager";

// API ルート共通: 対象の機体。body に machine があればそれ(編集画面・機体比較は
// 「今の機体」以外も触る)、無ければヘッダー / クエリ。
export function machineOf(request: Request, bodyMachine?: unknown): string {
  if (typeof bodyMachine === "string" && bodyMachine) return getMachine(bodyMachine).id;
  return machineOfRequest(request);
}

// 送信の失敗。送り先の基板が別の機体・未登録のときは 409 と code を返す
// (画面が確認を取って force 付きで送り直す)。
export function sendErrorResponse(err: unknown, extra: Record<string, unknown> = {}) {
  if (err instanceof SendTargetError) {
    return NextResponse.json({ ...extra, error: err.message, code: err.code, board: err.board }, { status: 409 });
  }
  return NextResponse.json({ ...extra, error: (err as Error).message }, { status: 500 });
}
