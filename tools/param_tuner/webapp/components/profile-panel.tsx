"use client";

import { useMemo, useState, type ReactNode } from "react";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import { AM32_FILE, type Am32Action } from "@/lib/am32-shared";
import type { ProfileList, SendScope } from "@/lib/serial-manager";

const ALL_SENTINEL = "__all__";

interface Props {
  profiles: ProfileList;
  sending: string | null;
  am32Action: Am32Action | null;
  onSendFile: (scope: SendScope, file: string) => void;
  onSendAll: () => void;
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
  onSendFile,
  onSendAll,
  onEditFile,
  onOpenTemplates,
  onOpenMatrix,
  onAm32Sync,
  onAm32Read,
}: Props) {
  const isBusy = sending !== null || am32Action !== null;
  const total = profiles.base.length + profiles.mode.length;

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

  return (
    <Card className="flex h-full min-w-0 flex-col overflow-hidden">
      <CardHeader className="gap-1">
        <div className="flex items-center justify-between">
          <CardTitle>パラメータ送信 (hf)</CardTitle>
          <Button size="sm" variant="ghost" onClick={onOpenMatrix}>
            パラメータ表
          </Button>
        </div>
        <span className="text-xs text-muted-foreground">
          {needle ? `${shown} / ${total} ファイル` : `${total} ファイル`}
        </span>
      </CardHeader>
      <CardContent className="flex flex-1 flex-col gap-2 overflow-hidden">
        <Button onClick={onSendAll} disabled={isBusy} className="w-full">
          {sending === ALL_SENTINEL ? "送信中..." : "全て送信"}
        </Button>
        <Input
          placeholder="ファイルを絞り込み..."
          value={query}
          onChange={(e) => setQuery(e.target.value)}
        />
        <Separator />
        <ScrollArea className="min-h-0 flex-1">
          <div className="flex flex-col gap-1 pr-2">
            {shown === 0 && (
              <span className="px-2 py-1 text-sm text-muted-foreground">該当するファイルがありません</span>
            )}
            {base.map((file) => (
              <FileRow
                key={file}
                file={file}
                sending={sending}
                disabled={isBusy}
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
  onSend,
  onEdit,
  extra,
}: {
  file: string;
  sending: string | null;
  disabled: boolean;
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
      className={`flex items-center justify-between gap-2 rounded px-2 py-1 hover:bg-muted ${editable ? "cursor-pointer" : ""}`}
    >
      <span className="truncate text-sm">{file}</span>
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

export { ALL_SENTINEL };
