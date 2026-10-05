"use client";

import { useMemo, useState, type ReactNode } from "react";
import { MachineChip } from "@/components/machine-chip";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { AM32_FILE, type Am32Action } from "@/lib/am32-shared";
import type { Machine } from "@/lib/machine-shared";
import type { ProfileList, SendScope } from "@/lib/serial-manager";

const ALL_SENTINEL = "__all__";
const UNSENT_SENTINEL = "__unsent__";

interface Props {
  profiles: ProfileList;
  sending: string | null;
  am32Action: Am32Action | null;
  // 表示している機体(このパネルのファイルはすべてこの機体のもの)
  machine: Machine | null;
  // 接続中の基板が別の機体のとき、その機体(送信ボタンに注意を出す)
  boardMachine: Machine | null;
  onSendFile: (scope: SendScope, file: string) => void;
  onSendAll: () => void;
  onSendUnsent: () => void;
  onEditFile: (scope: SendScope, file: string) => void;
  onOpenTemplates: () => void;
  onOpenMatrix: () => void;
  onAm32Sync: () => void;
  onAm32Read: () => void;
}

export function ProfilePanel({
  profiles,
  sending,
  am32Action,
  machine,
  boardMachine,
  onSendFile,
  onSendAll,
  onSendUnsent,
  onEditFile,
  onOpenTemplates,
  onOpenMatrix,
  onAm32Sync,
  onAm32Read,
}: Props) {
  const isBusy = sending !== null || am32Action !== null;
  const total = profiles.base.length + profiles.mode.length;
  const mismatch = boardMachine !== null && machine !== null && boardMachine.id !== machine.id;

  const [query, setQuery] = useState("");
  const needle = query.trim().toLowerCase();
  const base = useMemo(
    () => (needle ? profiles.base.filter((f) => f.toLowerCase().includes(needle)) : profiles.base),
    [profiles.base, needle]
  );
  const mode = useMemo(
    () => (needle ? profiles.mode.filter((f) => f.toLowerCase().includes(needle)) : profiles.mode),
    [profiles.mode, needle]
  );
  const shown = base.length + mode.length;
  const unsent = useMemo(() => new Set(profiles.unsent ?? []), [profiles.unsent]);
  const unsentCount = profiles.unsent?.length ?? 0;

  return (
    <Card
      className="flex h-full min-w-0 flex-col overflow-hidden"
      style={machine ? { boxShadow: `inset 0 3px 0 ${machine.color}` } : undefined}
      data-profile-panel
    >
      <CardHeader className="gap-1">
        {/* CardHeader は grid なので、min-w-0 が無いと狭い幅で見出しが縮まず右のボタンが切れる */}
        <div className="flex min-w-0 items-center justify-between gap-1">
          <CardTitle className="flex min-w-0 items-center gap-1.5">
            {machine && <MachineChip machine={machine} title={`machines/${machine.id}/profile`} />}
            <span className="truncate">パラメータ (hf)</span>
          </CardTitle>
          <Button size="sm" variant="ghost" onClick={onOpenMatrix}>
            パラメータ表
          </Button>
        </div>
        <span className="text-xs text-muted-foreground">
          {needle ? `${shown} / ${total} ファイル` : `${total} ファイル`}
          {mismatch && (
            <span className="ml-1.5 text-amber-300" data-send-warning>
              接続中の基板は {boardMachine.label}(送信時に確認します)
            </span>
          )}
        </span>
      </CardHeader>
      <CardContent className="flex flex-1 flex-col gap-1.5 overflow-hidden">
        <div className="flex gap-1.5">
          <Button onClick={onSendAll} disabled={isBusy} className="min-w-0 flex-1">
            {sending === ALL_SENTINEL ? "送信中..." : "全て送信"}
          </Button>
          {profiles.unsent !== null && (
            <Button
              variant={unsentCount > 0 ? "secondary" : "outline"}
              onClick={onSendUnsent}
              disabled={isBusy || unsentCount === 0}
              className="min-w-0 flex-1"
              title="いまつないでいる基板へ最後に送った中身と違うファイルだけ送る(● の付いたファイル)。この Param Console から送った分だけを覚えているので、一度も送っていないファイルは数えない"
              data-send-unsent
            >
              {sending === UNSENT_SENTINEL ? "送信中..." : unsentCount > 0 ? `未送信 ${unsentCount} を送信` : "未送信なし"}
            </Button>
          )}
        </div>
        <Input
          placeholder="ファイルを絞り込み..."
          value={query}
          onChange={(e) => setQuery(e.target.value)}
        />
        <Separator />
        <ScrollArea className="min-h-0 flex-1">
          <div className="flex flex-col gap-0.5 pr-1.5">
            {shown === 0 && (
              <span className="px-1.5 py-0.5 text-sm text-muted-foreground">該当するファイルがありません</span>
            )}
            {base.map((file) => (
              <FileRow
                key={file}
                file={file}
                sending={sending}
                disabled={isBusy}
                unsent={unsent.has(`base/${file}`)}
                onSend={() => onSendFile("base", file)}
                onEdit={() => onEditFile("base", file)}
                extra={
                  file === "system.yaml" ? (
                    <Button
                      size="sm"
                      variant="ghost"
                      disabled={isBusy}
                      onClick={(e) => {
                        e.stopPropagation();
                        onOpenTemplates();
                      }}
                    >
                      テンプレート
                    </Button>
                  ) : file === AM32_FILE ? (
                    // Plain "Send" only drops am32.yaml into the device's
                    // LittleFS; nothing reaches the ESC until write_am32_param()
                    // runs. These two are that missing half (send_file.py's
                    // am32sync / am32read).
                    <>
                      <Button
                        size="sm"
                        variant="ghost"
                        disabled={isBusy}
                        title="ESCの現在値を読み出してコンソールへ表示 (AM32READ)"
                        onClick={(e) => {
                          e.stopPropagation();
                          onAm32Read();
                        }}
                      >
                        {am32Action === "read" ? "読出中..." : "ESC読出"}
                      </Button>
                      <Button
                        size="sm"
                        variant="secondary"
                        disabled={isBusy}
                        title="am32.yaml を送信してESCのflashへ書き込む (送信 + AM32WRITE)"
                        onClick={(e) => {
                          e.stopPropagation();
                          onAm32Sync();
                        }}
                      >
                        {am32Action === "sync" ? "書込中..." : "ESC書込"}
                      </Button>
                    </>
                  ) : undefined
                }
              />
            ))}
            {base.length > 0 && mode.length > 0 && <Separator className="my-1" />}
            {mode.map((file) => (
              <FileRow
                key={file}
                file={file}
                sending={sending}
                disabled={isBusy}
                unsent={unsent.has(`mode/${file}`)}
                onSend={() => onSendFile("mode", file)}
                onEdit={() => onEditFile("mode", file)}
              />
            ))}
          </div>
        </ScrollArea>
      </CardContent>
    </Card>
  );
}

function FileRow({
  file,
  sending,
  disabled,
  unsent,
  onSend,
  onEdit,
  extra,
}: {
  file: string;
  sending: string | null;
  disabled: boolean;
  unsent: boolean;
  onSend: () => void;
  onEdit: () => void;
  extra?: ReactNode;
}) {
  const editable = file.endsWith(".yaml");
  return (
    <div
      role={editable ? "button" : undefined}
      tabIndex={editable ? 0 : undefined}
      onClick={editable ? onEdit : undefined}
      onKeyDown={
        editable
          ? (e) => {
              if (e.key === "Enter" || e.key === " ") onEdit();
            }
          : undefined
      }
      className={`flex items-center justify-between gap-2 rounded px-1.5 py-0.5 hover:bg-muted ${editable ? "cursor-pointer" : ""}`}
      data-file-row={file}
    >
      <span className="flex min-w-0 items-center gap-1 text-sm">
        <span className="truncate">{file}</span>
        {unsent && (
          <span
            className="shrink-0 text-[0.6rem] text-amber-300"
            title="いまつないでいる基板へ最後に送ったあと、中身が変わっている(まだ送っていない)"
            data-unsent
          >
            ●
          </span>
        )}
      </span>
      <div className="flex shrink-0 items-center gap-1">
        {extra}
        <Button
          size="sm"
          variant="outline"
          disabled={disabled}
          onClick={(e) => {
            e.stopPropagation();
            onSend();
          }}
        >
          {sending === file ? "..." : "Send"}
        </Button>
      </div>
    </div>
  );
}

export { ALL_SENTINEL, UNSENT_SENTINEL };
