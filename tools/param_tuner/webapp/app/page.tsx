"use client";

import { useCallback, useEffect, useState } from "react";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { ConsoleLog } from "@/components/console-log";
import { LogPlotPanel } from "@/components/log-plot-panel";
import { ParamMatrixPanel } from "@/components/param-matrix-panel";
import { ALL_SENTINEL, ProfilePanel } from "@/components/profile-panel";
import { PortPanel } from "@/components/port-panel";
import { TestTemplatePanel } from "@/components/test-template-panel";
import { YamlEditor } from "@/components/yaml-editor";
import type { ConnectionStatus, PortInfo, ProfileList, SendScope } from "@/lib/serial-manager";
import type { TestTemplate, TestTemplateValues } from "@/lib/test-template-shared";

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

  const [rightTab, setRightTab] = useState<"console" | "plot">("console");

  const [editing, setEditing] = useState<EditTarget | null>(null);
  const [editorContent, setEditorContent] = useState<string | null>(null);
  const [saving, setSaving] = useState(false);

  const [templates, setTemplates] = useState<TestTemplate[]>([]);
  const [showTemplates, setShowTemplates] = useState(false);
  const [applyingTemplate, setApplyingTemplate] = useState<string | null>(null);
  const [savingTemplate, setSavingTemplate] = useState(false);

  const [showMatrix, setShowMatrix] = useState(false);

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

  const refreshTemplates = useCallback(async () => {
    const res = await fetch("/api/test-templates");
    const data = await res.json();
    setTemplates(data.templates as TestTemplate[]);
  }, []);

  useEffect(() => {
    // react-hooks/set-state-in-effect flags any effect that fetches on
    // mount, but this project doesn't run the React Compiler; the standard
    // fetch-on-mount pattern (see react.dev/learn/you-might-not-need-an-effect)
    // is correct here.
    // eslint-disable-next-line react-hooks/set-state-in-effect
    void refreshPorts();
    void refreshProfiles();
    void refreshTemplates();
    const interval = setInterval(() => void refreshPorts(), 3000);
    return () => clearInterval(interval);
  }, [refreshPorts, refreshProfiles, refreshTemplates]);

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
    setShowTemplates(false);
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
      // Stay in the editor; sync editorContent so the dirty flag clears
      // instead of staying stuck true (YamlEditor keeps its own draft state).
      setEditorContent(content);
    } catch (err) {
      toast.error(`${editing.file}: ${(err as Error).message}`);
    } finally {
      setSaving(false);
    }
  };

  const openTemplates = () => {
    setEditing(null);
    setEditorContent(null);
    setShowMatrix(false);
    setShowTemplates(true);
  };

  const openMatrix = () => {
    setEditing(null);
    setEditorContent(null);
    setShowTemplates(false);
    setShowMatrix(true);
  };

  const applyTemplate = async (id: string) => {
    setApplyingTemplate(id);
    try {
      const res = await fetch("/api/test-templates/apply", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ id }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "適用に失敗しました");
      toast.success(`system.yaml に適用しました: ${data.name}`);
    } catch (err) {
      toast.error((err as Error).message);
    } finally {
      setApplyingTemplate(null);
    }
  };

  const saveTemplate = async (id: string | undefined, name: string, values: TestTemplateValues) => {
    setSavingTemplate(true);
    try {
      const res = await fetch("/api/test-templates", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ id, name, values }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "保存に失敗しました");
      toast.success(`テンプレートを保存しました: ${name}`);
      await refreshTemplates();
    } catch (err) {
      toast.error((err as Error).message);
    } finally {
      setSavingTemplate(false);
    }
  };

  const deleteTemplate = async (id: string) => {
    try {
      const res = await fetch(`/api/test-templates?id=${encodeURIComponent(id)}`, { method: "DELETE" });
      if (!res.ok) throw new Error("削除に失敗しました");
      await refreshTemplates();
    } catch (err) {
      toast.error((err as Error).message);
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
      {showMatrix ? (
        <div className="flex flex-1 overflow-hidden">
          <ParamMatrixPanel onClose={() => setShowMatrix(false)} />
        </div>
      ) : (
      <div className="flex flex-1 gap-4 overflow-hidden">
        <ProfilePanel
          profiles={profiles}
          sending={sending}
          onSendFile={sendOne}
          onSendAll={sendAll}
          onEditFile={openEditor}
          onOpenTemplates={openTemplates}
          onOpenMatrix={openMatrix}
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
        ) : showTemplates ? (
          <TestTemplatePanel
            templates={templates}
            applying={applyingTemplate}
            saving={savingTemplate}
            onApply={applyTemplate}
            onSave={saveTemplate}
            onDelete={deleteTemplate}
            onClose={() => setShowTemplates(false)}
          />
        ) : (
          <div className="flex min-h-0 flex-1 flex-col gap-2 overflow-hidden">
            <div className="flex shrink-0 gap-1">
              <Button
                size="sm"
                variant={rightTab === "console" ? "default" : "outline"}
                onClick={() => setRightTab("console")}
              >
                コンソール
              </Button>
              <Button
                size="sm"
                variant={rightTab === "plot" ? "default" : "outline"}
                onClick={() => setRightTab("plot")}
              >
                プロット
              </Button>
            </div>
            {/* Both tabs render inside the same flex column; only their
                visibility toggles so the SSE-fed `lines` state above keeps
                accumulating in the background regardless of which tab is
                showing. */}
            <div className={`min-h-0 flex-1 ${rightTab === "console" ? "flex" : "hidden"}`}>
              <ConsoleLog
                lines={paused && frozenLines !== null ? frozenLines : lines}
                paused={paused}
                onClear={clearConsole}
                onTogglePause={togglePause}
              />
            </div>
            <div className={`min-h-0 flex-1 ${rightTab === "plot" ? "flex" : "hidden"}`}>
              <LogPlotPanel />
            </div>
          </div>
        )}
      </div>
      )}
    </div>
  );
}
