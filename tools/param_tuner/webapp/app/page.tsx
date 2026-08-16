"use client";

import { useCallback, useEffect, useState } from "react";
import { toast } from "sonner";
import { Card } from "@/components/ui/card";
import { ConsoleLog } from "@/components/console-log";
import { ALL_SENTINEL, ProfilePanel } from "@/components/profile-panel";
import { PortPanel } from "@/components/port-panel";
import { YamlEditor } from "@/components/yaml-editor";
import type { ConnectionStatus, PortInfo, ProfileList, SendScope } from "@/lib/serial-manager";

const MAX_LOG_LINES = 2000;
const MODE = "hf";

interface EditTarget {
  scope: SendScope;
  file: string;
}

export default function Home() {
  const [ports, setPorts] = useState<PortInfo[]>([]);
  const [status, setStatus] = useState<ConnectionStatus>("disconnected");
  const [connectedPath, setConnectedPath] = useState<string | null>(null);
  const [autoConnect, setAutoConnect] = useState(true);

  const [profiles, setProfiles] = useState<ProfileList>({ base: [], mode: [] });

  const [lines, setLines] = useState<string[]>([]);
  const [paused, setPaused] = useState(false);
  // Snapshot taken at pause time; keeps the displayed console frozen while
  // `lines` keeps accumulating live in the background, so resuming catches
  // up instantly instead of losing anything that arrived while paused.
  const [frozenLines, setFrozenLines] = useState<string[] | null>(null);
  const [sending, setSending] = useState<string | null>(null);

  const [editing, setEditing] = useState<EditTarget | null>(null);
  const [editorContent, setEditorContent] = useState<string | null>(null);
  const [saving, setSaving] = useState(false);

  const refreshPorts = useCallback(async () => {
    const res = await fetch("/api/ports");
    const data = await res.json();
    setPorts(data.ports as PortInfo[]);
  }, []);

  const refreshProfiles = useCallback(async () => {
    const res = await fetch(`/api/profiles?mode=${encodeURIComponent(MODE)}`);
    const data = await res.json();
    setProfiles(data);
  }, []);

  useEffect(() => {
    // react-hooks/set-state-in-effect flags any effect that fetches on
    // mount, but this project doesn't run the React Compiler; the standard
    // fetch-on-mount pattern (see react.dev/learn/you-might-not-need-an-effect)
    // is correct here.
    // eslint-disable-next-line react-hooks/set-state-in-effect
    void refreshPorts();
    void refreshProfiles();
    const interval = setInterval(() => void refreshPorts(), 3000);
    return () => clearInterval(interval);
  }, [refreshPorts, refreshProfiles]);

  useEffect(() => {
    const es = new EventSource("/api/stream");

    es.addEventListener("log", (e) => {
      const { line } = JSON.parse((e as MessageEvent).data) as { line: string };
      setLines((prev) => {
        const next = prev.length >= MAX_LOG_LINES ? prev.slice(prev.length - MAX_LOG_LINES + 1) : prev;
        return [...next, line];
      });
    });

    es.addEventListener("status", (e) => {
      const data = JSON.parse((e as MessageEvent).data) as {
        status: ConnectionStatus;
        path: string | null;
        autoConnect: boolean;
      };
      setStatus(data.status);
      setConnectedPath(data.path);
      setAutoConnect(data.autoConnect);
    });

    es.addEventListener("saved", (e) => {
      const data = JSON.parse((e as MessageEvent).data) as { type: string; file: string };
      toast.success(`保存しました (${data.type}): ${data.file}`);
    });

    // Device sent a clear-screen escape (e.g. dump1()'s live redraw loop):
    // reset the scrollback so it renders as a refreshing dashboard.
    es.addEventListener("clear", () => {
      setLines([]);
    });

    return () => es.close();
  }, []);

  const togglePause = () => {
    setPaused((prev) => {
      if (!prev) setFrozenLines(lines);
      else setFrozenLines(null);
      return !prev;
    });
  };

  const clearConsole = () => {
    setLines([]);
    setFrozenLines((prev) => (prev !== null ? [] : prev));
  };

  const handleEnableAutoConnect = async () => {
    await fetch("/api/connect", { method: "POST" });
  };

  const handleDisconnect = async () => {
    await fetch("/api/disconnect", { method: "POST" });
  };

  const sendOne = async (scope: SendScope, file: string) => {
    setSending(file);
    try {
      const res = await fetch("/api/send", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode: MODE, scope, file }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "送信に失敗しました");
      toast.success(`${file}: 送信完了`);
    } catch (err) {
      toast.error(`${file}: ${(err as Error).message}`);
    } finally {
      setSending(null);
    }
  };

  const openEditor = async (scope: SendScope, file: string) => {
    setEditing({ scope, file });
    setEditorContent(null);
    try {
      const res = await fetch(
        `/api/profile-file?mode=${encodeURIComponent(MODE)}&scope=${scope}&file=${encodeURIComponent(file)}`
      );
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "読み込みに失敗しました");
      setEditorContent(data.content as string);
    } catch (err) {
      toast.error(`${file}: ${(err as Error).message}`);
      setEditing(null);
    }
  };

  const closeEditor = () => {
    setEditing(null);
    setEditorContent(null);
  };

  const saveEditor = async (content: string) => {
    if (!editing) return;
    setSaving(true);
    try {
      const res = await fetch("/api/profile-file", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode: MODE, scope: editing.scope, file: editing.file, content }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "保存に失敗しました");
      toast.success(`${editing.file}: 保存しました`);
      closeEditor();
    } catch (err) {
      toast.error(`${editing.file}: ${(err as Error).message}`);
    } finally {
      setSaving(false);
    }
  };

  const sendAll = async () => {
    setSending(ALL_SENTINEL);
    try {
      const res = await fetch("/api/send", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode: MODE, all: true }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "送信に失敗しました");
      toast.success("全ファイル送信完了");
    } catch (err) {
      toast.error(`全送信エラー: ${(err as Error).message}`);
    } finally {
      setSending(null);
    }
  };

  return (
    <div className="flex h-screen flex-col gap-4 p-4">
      <PortPanel
        ports={ports}
        connectedPath={connectedPath}
        status={status}
        autoConnect={autoConnect}
        onDisconnect={handleDisconnect}
        onEnableAutoConnect={handleEnableAutoConnect}
      />
      <div className="flex flex-1 gap-4 overflow-hidden">
        <ProfilePanel
          profiles={profiles}
          sending={sending}
          onSendFile={sendOne}
          onSendAll={sendAll}
          onEditFile={openEditor}
        />
        {editing ? (
          editorContent === null ? (
            <Card className="flex flex-1 items-center justify-center overflow-hidden">
              <span className="text-sm text-muted-foreground">読み込み中...</span>
            </Card>
          ) : (
            <YamlEditor
              key={`${editing.scope}:${editing.file}`}
              file={editing.file}
              content={editorContent}
              saving={saving}
              onSave={saveEditor}
              onClose={closeEditor}
            />
          )
        ) : (
          <ConsoleLog
            lines={paused && frozenLines !== null ? frozenLines : lines}
            paused={paused}
            onClear={clearConsole}
            onTogglePause={togglePause}
          />
        )}
      </div>
    </div>
  );
}
