"use client";

import { createContext, useContext } from "react";
import {
  MACHINE_HEADER,
  type BoardInfo,
  type Machine,
  type MachineRegistry,
} from "./machine-shared";

// クライアント側の「今の機体」。API を呼ぶたびにヘッダーで伝える(サーバーは覚えない)。
// React の state とは別に持つのは、子のパネルがマウント直後の effect で fetch するため
// (親の effect より先に走るので、state 経由だと最初の 1 回に間に合わない)。
// app/page.tsx が、機体を切り替える処理の中で setState と同時に書き換える。
let current: string | null = null;

export function setCurrentMachine(id: string | null): void {
  current = id;
}

// 今の機体を添えて API を呼ぶ。機体のパラメータを読む・書く API はこれを使う。
export function apiFetch(input: string, init: RequestInit = {}): Promise<Response> {
  const headers = new Headers(init.headers);
  if (current && !headers.has(MACHINE_HEADER)) headers.set(MACHINE_HEADER, current);
  return fetch(input, { ...init, headers });
}

// 送り先の基板が別の機体・未登録のとき(API が 409 を返したとき)に確認を取り、
// 必要なら force 付きでやり直す。run(force) が実際の fetch。
export type GuardedSend = (run: (force: boolean) => Promise<Response>) => Promise<Response>;

export interface MachineContextValue {
  registry: MachineRegistry;
  current: string | null; // 画面に出している機体
  board: BoardInfo; // 接続中の基板
  guardedSend: GuardedSend;
}

export const MachineContext = createContext<MachineContextValue>({
  registry: { default: null, machines: [], specific: {} },
  current: null,
  board: { serial: null, machine: null },
  guardedSend: (run) => run(false),
});

export function useMachines(): MachineContextValue {
  return useContext(MachineContext);
}

export function findMachine(registry: MachineRegistry, id: string | null | undefined): Machine | null {
  return registry.machines.find((m) => m.id === id) ?? null;
}

// 確認でやめたとき。呼ぶ側はエラーを出さずに黙って終わる
export class SendCancelled extends Error {
  constructor() {
    super("送信をやめました");
  }
}
