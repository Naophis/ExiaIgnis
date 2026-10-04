"use client";

import { Usb } from "lucide-react";
import { Button } from "@/components/ui/button";
import { contrastText, shortSerial, type BoardInfo, type MachineRegistry } from "@/lib/machine-shared";
import { findMachine } from "@/lib/machine-client";

interface Props {
  registry: MachineRegistry;
  current: string | null;
  board: BoardInfo;
  compareActive: boolean;
  onSelect: (id: string) => void;
  onRegisterBoard: (id: string) => void;
  onOpenManage: () => void;
  onToggleCompare: () => void;
}

// ヘッダーバーの機体の切り替え。どの機体のパラメータを見ているか(塗りつぶし)と、
// つないでいる基板がどの機体か(USB の印)を常に出す。
export function MachineBar({
  registry,
  current,
  board,
  compareActive,
  onSelect,
  onRegisterBoard,
  onOpenManage,
  onToggleCompare,
}: Props) {
  const machines = registry.machines;
  const boardMachine = findMachine(registry, board.machine);
  const unregistered = board.serial !== null && board.machine === null;
  const mismatch = boardMachine !== null && current !== null && boardMachine.id !== current;

  return (
    <div className="flex min-w-0 items-center gap-1" data-machine-bar>
      {machines.map((m) => {
        const selected = m.id === current;
        const connected = m.id === board.machine;
        return (
          <button
            key={m.id}
            type="button"
            data-machine={m.id}
            data-selected={selected}
            onClick={() => onSelect(m.id)}
            title={
              `${m.label} のパラメータを表示(machines/${m.id}/profile)` +
              (connected ? "\nいまつないでいる基板はこの機体" : "") +
              (m.serials.length === 0 ? "\n基板が未登録(つないで登録すると自動で見分ける)" : "")
            }
            style={
              selected
                ? { backgroundColor: m.color, color: contrastText(m.color), borderColor: m.color }
                : { borderColor: `${m.color}99` }
            }
            className="inline-flex h-7 items-center gap-1 rounded-md border px-2 text-[0.8rem] font-semibold whitespace-nowrap transition-colors hover:brightness-110"
          >
            {!selected && <span className="size-2 rounded-full" style={{ backgroundColor: m.color }} />}
            {m.label}
            {connected && <Usb className="size-3.5" aria-label="接続中" />}
          </button>
        );
      })}
      <Button size="sm" variant="ghost" className="px-1.5" title="機体の追加・名前・色・基板の登録" onClick={onOpenManage}>
        {machines.length === 0 ? "+ 機体を追加" : "機体設定"}
      </Button>
      {machines.length >= 2 && (
        <Button
          size="sm"
          variant={compareActive ? "default" : "outline"}
          title="機体どうしのパラメータの違いを一覧し、値をそろえる"
          onClick={onToggleCompare}
          data-compare-button
        >
          機体比較
        </Button>
      )}
      {unregistered && (
        <span
          className="ml-1 inline-flex items-center gap-1 rounded-md border border-amber-500/60 bg-amber-500/10 px-1.5 py-0.5 text-xs text-amber-300"
          data-unregistered-board
        >
          未登録の基板 {shortSerial(board.serial!)}
          {machines.length <= 3 ? (
            machines.map((m) => (
              <Button
                key={m.id}
                size="xs"
                variant="outline"
                title={`この基板 (${board.serial}) を ${m.label} として登録する`}
                onClick={() => onRegisterBoard(m.id)}
              >
                {m.label} に登録
              </Button>
            ))
          ) : (
            <Button size="xs" variant="outline" onClick={onOpenManage}>
              登録...
            </Button>
          )}
          <Button size="xs" variant="outline" title="別の機体として追加する" onClick={onOpenManage}>
            新しい機体
          </Button>
        </span>
      )}
      {mismatch && (
        <button
          type="button"
          className="ml-1 inline-flex items-center gap-1 rounded-md border border-amber-500/60 bg-amber-500/10 px-1.5 py-0.5 text-xs text-amber-300 hover:bg-amber-500/20"
          title="表示中の機体と、つないでいる基板の機体が違います。クリックで接続中の機体の表示に切り替える"
          onClick={() => onSelect(boardMachine.id)}
          data-machine-mismatch
        >
          <Usb className="size-3.5" />
          接続中は {boardMachine.label}
        </button>
      )}
    </div>
  );
}
