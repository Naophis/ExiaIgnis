"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { toast } from "sonner";
import { ChoiceDialog, type Choice, type ChoiceRequest } from "@/components/choice-dialog";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { ConsoleLog } from "@/components/console-log";
import { LogPlotPanel } from "@/components/log-plot-panel";
import { MachineBar } from "@/components/machine-bar";
import { MachineChip } from "@/components/machine-chip";
import { MachineComparePanel } from "@/components/machine-compare-panel";
import { MachineDialog } from "@/components/machine-dialog";
import { MazePanel } from "@/components/maze-panel";
import { ParamMatrixPanel } from "@/components/param-matrix-panel";
import { ALL_SENTINEL, ProfilePanel, UNSENT_SENTINEL } from "@/components/profile-panel";
import { PortPanel } from "@/components/port-panel";
import { ResizableHandle, ResizablePanel, ResizablePanelGroup } from "@/components/ui/resizable";
import { SensorCalibPanel } from "@/components/sensor-calib-panel";
import { SlalomSimPanel } from "@/components/slalom-sim-panel";
import { TestTemplatePanel } from "@/components/test-template-panel";
import { ThemePicker } from "@/components/theme-picker";
import { YamlEditor } from "@/components/yaml-editor";
import { AM32_FILE, type Am32Action } from "@/lib/am32-shared";
import {
  MachineContext,
  SendCancelled,
  apiFetch,
  findMachine,
  setCurrentMachine,
  type GuardedSend,
  type MachineContextValue,
} from "@/lib/machine-client";
import type { PropagateSuggestion, SyncResult } from "@/lib/machine-compare-shared";
import { shortSerial, type BoardInfo, type MachineRegistry } from "@/lib/machine-shared";
import type { ConnectionStatus, PortInfo, ProfileList, SendScope } from "@/lib/serial-manager";
import type { TestTemplate, TestTemplateArrayValues, TestTemplateValues } from "@/lib/test-template-shared";
import { DEFAULT_PREFS, commitTheme, loadThemePrefs, saveThemePrefs, type ThemePrefs } from "@/lib/theme";
import { useMachineDiff } from "@/lib/use-machine-diff";

const MAX_LOG_LINES = 2000;
const MODE = "hf";
const TURN_PROFILE_FILE_RE = /^t_\d+\.yaml$/i;
const MACHINE_KEY = "exia-machine-v1";
const NO_BOARD: BoardInfo = { serial: null, machine: null };

// 編集中のファイル。どの機体のファイルかを自分で持つ(表示中の機体を切り替えても、
// 開いているファイルの保存先は変わらない)。
interface EditTarget {
  machine: string;
  scope: SendScope;
  file: string;
}

const relFile = (t: { scope: SendScope; file: string }) => (t.scope === "base" ? t.file : `${MODE}/${t.file}`);

export default function Home() {
  const [ports, setPorts] = useState<PortInfo[]>([]);
  const [status, setStatus] = useState<ConnectionStatus>("disconnected");
  const [connectedPath, setConnectedPath] = useState<string | null>(null);
  const [autoConnect, setAutoConnect] = useState(true);

  // 機体: 登録簿(null = 読み込み中)、表示中の機体、接続中の基板
  const [registry, setRegistry] = useState<MachineRegistry | null>(null);
  // 登録簿(machines.yaml)を読めなかった理由。手で直して壊したときに、空の登録簿として
  // 扱って上書きしてしまわないよう、読めるまで何も出さない
  const [registryError, setRegistryError] = useState<string | null>(null);
  const [machine, setMachine] = useState<string | null>(null);
  const [board, setBoard] = useState<BoardInfo>(NO_BOARD);
  const [showMachineDialog, setShowMachineDialog] = useState(false);
  const [showCompare, setShowCompare] = useState(false);
  const [choice, setChoice] = useState<ChoiceRequest | null>(null);
  const machineRef = useRef<string | null>(null);
  const registryRef = useRef<MachineRegistry | null>(null);
  // つないだ基板の機体へ表示を切り替えるのは、つないだときの 1 回だけ
  // (そのあと手で別の機体を見に行っても、勝手に戻さない)
  const autoSwitchedSerial = useRef<string | null>(null);

  const [profiles, setProfiles] = useState<ProfileList>({ base: [], mode: [], unsent: null });

  const [lines, setLines] = useState<string[]>([]);
  const [paused, setPaused] = useState(false);
  // Snapshot taken at pause time; keeps the displayed console frozen while
  // `lines` keeps accumulating live in the background, so resuming catches
  // up instantly instead of losing anything that arrived while paused.
  const [frozenLines, setFrozenLines] = useState<string[] | null>(null);
  const [sending, setSending] = useState<string | null>(null);
  const [am32Action, setAm32Action] = useState<Am32Action | null>(null);

  const [rightTab, setRightTab] = useState<"console" | "plot" | "calib" | "maze">("console");
  const [flashing, setFlashing] = useState(false);
  const [plotAutoOpen, setPlotAutoOpen] = useState<{ file: string; nonce: number } | null>(null);
  const [mazeAutoOpen, setMazeAutoOpen] = useState<{ id: string; nonce: number } | null>(null);
  // LogPlotPanel は memo なので、渡すコールバックは固定する
  const handlePlotAutoOpenHandled = useCallback(() => setPlotAutoOpen(null), []);
  const [mazeRefreshNonce, setMazeRefreshNonce] = useState(0);

  const [editing, setEditing] = useState<EditTarget | null>(null);
  const [editorContent, setEditorContent] = useState<string | null>(null);
  const [liveDraft, setLiveDraft] = useState<string | null>(null);
  // Bumped whenever the slalom sim panel patches liveDraft externally, to
  // force YamlEditor to remount and re-seed its internal draft from the new
  // liveDraft (see YamlEditor's initialDraft prop) instead of keeping the
  // stale buffer it mounted with.
  const [draftPatchNonce, setDraftPatchNonce] = useState(0);
  const [saving, setSaving] = useState(false);
  // 保存したとき「ほかの機体も同じ値だった」キー(同じ変更を入れる候補)
  const [propagate, setPropagate] = useState<PropagateSuggestion[]>([]);
  // ほかの機体の値を取り直す合図(保存・同期のあと)
  const [diffNonce, setDiffNonce] = useState(0);

  const [templates, setTemplates] = useState<TestTemplate[]>([]);
  const [showTemplates, setShowTemplates] = useState(false);
  const [applyingTemplate, setApplyingTemplate] = useState<string | null>(null);
  const [savingTemplate, setSavingTemplate] = useState(false);

  const [showMatrix, setShowMatrix] = useState(false);

  // テーマカラー(lib/theme.ts)。null = まだ localStorage を読んでいない
  // (読む前に既定の設定で上書きしないため)
  const [themePrefs, setThemePrefs] = useState<ThemePrefs | null>(null);

  const refreshPorts = useCallback(async () => {
    const res = await fetch("/api/ports");
    const data = await res.json();
    const next = data.ports as PortInfo[];
    // 3 秒ごとの確認で変わっていなければ入れ替えない(画面全体の描き直しを起こさない)
    setPorts((prev) => (JSON.stringify(prev) === JSON.stringify(next) ? prev : next));
  }, []);

  const refreshProfiles = useCallback(async () => {
    if (!machineRef.current) return;
    const res = await apiFetch(`/api/profiles?mode=${encodeURIComponent(MODE)}`);
    const data = await res.json();
    if (res.ok) setProfiles(data);
  }, []);

  const refreshTemplates = useCallback(async () => {
    if (!machineRef.current) return;
    const res = await apiFetch("/api/test-templates");
    const data = await res.json();
    if (res.ok) setTemplates(data.templates as TestTemplate[]);
  }, []);

  // 表示する機体を切り替える。API 用の値(setCurrentMachine)を先に書き換えてから
  // state を変えるので、作り直されるパネルの最初の fetch から新しい機体になる。
  const selectMachine = useCallback(
    (id: string | null) => {
      machineRef.current = id;
      setCurrentMachine(id);
      setMachine(id);
      if (id) {
        try {
          localStorage.setItem(MACHINE_KEY, id);
        } catch {
          // 覚えられなくても動く
        }
      }
      setShowTemplates(false);
      setMazeRefreshNonce((n) => n + 1);
      void refreshProfiles();
      void refreshTemplates();
    },
    [refreshProfiles, refreshTemplates],
  );

  const applyRegistry = useCallback((next: MachineRegistry, nextBoard?: BoardInfo) => {
    registryRef.current = next;
    setRegistry(next);
    if (nextBoard) setBoard(nextBoard);
  }, []);

  useEffect(() => {
    // react-hooks/set-state-in-effect flags any effect that fetches on
    // mount, but this project doesn't run the React Compiler; the standard
    // fetch-on-mount pattern (see react.dev/learn/you-might-not-need-an-effect)
    // is correct here.
    // eslint-disable-next-line react-hooks/set-state-in-effect
    void refreshPorts();
    const interval = setInterval(() => void refreshPorts(), 3000);
    return () => clearInterval(interval);
  }, [refreshPorts]);

  useEffect(() => {
    // localStorage は描画後にしか読めない(SSR)。
    // eslint-disable-next-line react-hooks/set-state-in-effect
    setThemePrefs(loadThemePrefs());
  }, []);

  const changeTheme = useCallback((next: ThemePrefs) => {
    setThemePrefs(next);
    saveThemePrefs(next);
  }, []);

  // テーマカラーを当てる。「機体ごとの色」のときは表示中の機体の色
  // (機体がまだ分からない間は、前回当てた色のままにしておく)。
  const themeMachineColor = registry && machine ? (findMachine(registry, machine)?.color ?? null) : null;
  const registryLoaded = registry !== null;
  useEffect(() => {
    if (!themePrefs) return;
    if (themePrefs.mode === "machine") {
      if (themeMachineColor) commitTheme(themeMachineColor);
      // 機体が 1 つも無いときは、全機体で 1 色のときと同じ色
      else if (registryLoaded && machine === null) commitTheme(themePrefs.color);
      return;
    }
    commitTheme(themePrefs.color);
  }, [themePrefs, themeMachineColor, registryLoaded, machine]);

  // テーマの窓から機体の色を変える。画面はすぐ変え、machines.yaml へは手が止まってから書く
  // (色を選んでいる間は input イベントが連続で来る)。
  const colorSaveTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const setMachineColor = useCallback(
    (id: string, color: string) => {
      setRegistry((prev) =>
        prev ? { ...prev, machines: prev.machines.map((m) => (m.id === id ? { ...m, color } : m)) } : prev,
      );
      if (registryRef.current) {
        registryRef.current = {
          ...registryRef.current,
          machines: registryRef.current.machines.map((m) => (m.id === id ? { ...m, color } : m)),
        };
      }
      if (colorSaveTimer.current) clearTimeout(colorSaveTimer.current);
      colorSaveTimer.current = setTimeout(() => {
        void fetch("/api/machines", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ action: "update", id, color }),
        })
          .then((r) => r.json())
          .then((d) => {
            if (d.error) toast.error(`機体の色を保存できません: ${d.error}`);
          })
          .catch((err) => toast.error(`機体の色を保存できません: ${(err as Error).message}`));
      }, 400);
    },
    [],
  );

  // 別のエディタ(VSCode など)で yaml を直して戻ってきたとき、未送信の印を取り直す
  useEffect(() => {
    const onFocus = () => void refreshProfiles();
    window.addEventListener("focus", onFocus);
    return () => window.removeEventListener("focus", onFocus);
  }, [refreshProfiles]);

  // 最初に登録簿を読み、表示する機体を決める: つないでいる基板の機体 → 前回見ていた機体 →
  // 登録簿の default。
  useEffect(() => {
    void (async () => {
      try {
        const res = await fetch("/api/machines");
        const data = (await res.json()) as { registry: MachineRegistry; board: BoardInfo; error?: string };
        if (!res.ok) throw new Error(data.error ?? "機体の一覧を読めません");
        applyRegistry(data.registry, data.board);
        const ids = data.registry.machines.map((m) => m.id);
        let saved: string | null = null;
        try {
          saved = localStorage.getItem(MACHINE_KEY);
        } catch {
          saved = null;
        }
        const first =
          (data.board.machine && ids.includes(data.board.machine) ? data.board.machine : null) ??
          (saved && ids.includes(saved) ? saved : null) ??
          data.registry.default ??
          ids[0] ??
          null;
        if (data.board.machine) autoSwitchedSerial.current = data.board.serial;
        selectMachine(first);
        if (ids.length === 0) setShowMachineDialog(true);
      } catch (err) {
        setRegistryError((err as Error).message);
      }
    })();
  }, [applyRegistry, selectMachine]);

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
        serial: string | null;
        machine: string | null;
      };
      setStatus(data.status);
      setConnectedPath(data.path);
      setAutoConnect(data.autoConnect);
      setBoard((prev) =>
        prev.serial === data.serial && prev.machine === data.machine ? prev : { serial: data.serial, machine: data.machine },
      );
      if (data.status !== "connected") {
        autoSwitchedSerial.current = null;
      } else if (data.serial && autoSwitchedSerial.current !== data.serial) {
        // つないだ直後の 1 回: 基板の機体へ表示を合わせる
        const reg = registryRef.current;
        if (data.machine && reg?.machines.some((m) => m.id === data.machine)) {
          autoSwitchedSerial.current = data.serial;
          if (machineRef.current !== data.machine) {
            const label = findMachine(reg, data.machine)?.label ?? data.machine;
            selectMachine(data.machine);
            toast.info(`機体 ${label} をつなぎました。表示を ${label} に切り替えました`);
          }
        } else if (reg && !data.machine) {
          autoSwitchedSerial.current = data.serial;
          toast.warning(`未登録の基板 (${shortSerial(data.serial)}) です。ヘッダーでどの機体かを登録してください`);
        }
      }
      // 未送信の印は「つないでいる基板」に対するものなので取り直す
      void refreshProfiles();
    });

    es.addEventListener("saved", (e) => {
      const data = JSON.parse((e as MessageEvent).data) as { type: string; file: string };
      if (data.type === "csv") {
        toast.success(
          <button
            type="button"
            className="w-full cursor-pointer text-left hover:underline"
            onClick={() => {
              setRightTab("plot");
              setPlotAutoOpen({ file: data.file, nonce: Date.now() });
            }}
          >
            保存しました (csv): {data.file}
            <span className="block text-xs text-muted-foreground">クリックでPlotJugglerを開く</span>
          </button>,
        );
      } else if (data.type === "maze") {
        setMazeRefreshNonce((n) => n + 1);
        toast.success(
          <button
            type="button"
            className="w-full cursor-pointer text-left hover:underline"
            onClick={() => {
              setRightTab("maze");
              setMazeAutoOpen({ id: `log/${data.file}`, nonce: Date.now() });
            }}
          >
            保存しました (maze): {data.file}
            <span className="block text-xs text-muted-foreground">クリックで迷路タブに開く</span>
          </button>,
        );
      } else {
        toast.success(`保存しました (${data.type}): ${data.file}`);
      }
    });

    es.addEventListener("dumpFailed", (e) => {
      const { reason, deviceError } = JSON.parse((e as MessageEvent).data) as {
        reason: string;
        deviceError?: boolean;
      };
      // deviceError: firmware refused to dump (dumperr_ line, e.g. PSRAM FAIL).
      // Retrying won't help, so don't tell the user to.
      toast.error(
        deviceError
          ? `デバイスがダンプを拒否しました: ${reason}`
          : `ダンプ受信が壊れました。もう一度お試しください (${reason})`,
      );
    });

    // Device sent a clear-screen escape (e.g. dump1()'s live redraw loop):
    // reset the scrollback so it renders as a refreshing dashboard.
    es.addEventListener("clear", () => {
      setLines([]);
    });

    return () => es.close();
  }, [refreshProfiles, selectMachine]);

  // ===== 機体まわり =====

  const ask = useCallback(
    (title: string, body: ChoiceRequest["body"], choices: Choice[]) =>
      new Promise<string | null>((resolve) => {
        setChoice({
          title,
          body,
          choices,
          resolve: (key) => {
            setChoice(null);
            resolve(key);
          },
        });
      }),
    [],
  );

  const registerBoard = useCallback(
    async (id: string): Promise<boolean> => {
      try {
        const res = await fetch("/api/machines", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ action: "assignSerial", id }),
        });
        const data = await res.json();
        if (!res.ok) throw new Error(data.error ?? "登録に失敗しました");
        applyRegistry(data.registry, data.board);
        const label = findMachine(data.registry, id)?.label ?? id;
        toast.success(`つないでいる基板を ${label} に登録しました`);
        if (machineRef.current !== id) selectMachine(id);
        return true;
      } catch (err) {
        toast.error((err as Error).message);
        return false;
      }
    },
    [applyRegistry, selectMachine],
  );

  // 送信の API が 409(送り先の基板が別の機体・未登録)を返したら、何が起きるかを
  // 見せて選ばせる。黙って別の機体のパラメータを書き込まないための最後の関門。
  const guardedSend: GuardedSend = useCallback(
    async (run) => {
      const res = await run(false);
      if (res.status !== 409) return res;
      const data = (await res.clone().json()) as { code?: string; board?: BoardInfo };
      const reg = registryRef.current;
      const sendingId = machineRef.current;
      const sendingLabel = (reg && findMachine(reg, sendingId)?.label) ?? sendingId ?? "?";
      if (data.code === "mismatch" && data.board?.machine) {
        const boardId = data.board.machine;
        const boardLabel = (reg && findMachine(reg, boardId)?.label) ?? boardId;
        const key = await ask(
          "送り先の機体が違います",
          <>
            <span>
              つないでいる基板は <b className="text-foreground">{boardLabel}</b> です。送ろうとしているのは{" "}
              <b className="text-foreground">{sendingLabel}</b> のパラメータです。
            </span>
          </>,
          [
            { key: "switch", label: `${boardLabel} の表示に切り替える(何も送らない)`, variant: "default" },
            {
              key: "force",
              label: `このまま ${sendingLabel} のパラメータを ${boardLabel} の基板へ送る`,
              variant: "destructive",
            },
          ],
        );
        if (key === "force") return run(true);
        if (key === "switch") selectMachine(boardId);
        throw new SendCancelled();
      }
      if (data.code === "unregistered" && data.board?.serial) {
        const key = await ask(
          "未登録の基板です",
          <span>
            つないでいる基板 ({data.board.serial}) は、どの機体にも登録されていません。登録しておくと、次からは
            つないだときに機体を自動で見分けます。
          </span>,
          [
            { key: "register", label: `この基板を ${sendingLabel} として登録して送る`, variant: "default" },
            { key: "force", label: "登録せずに今回だけ送る", variant: "outline" },
          ],
        );
        if (key === "register" && sendingId) {
          if (await registerBoard(sendingId)) return run(false);
          throw new SendCancelled();
        }
        if (key === "force") return run(true);
        throw new SendCancelled();
      }
      return res;
    },
    [ask, registerBoard, selectMachine],
  );

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

  const handleFlash = async () => {
    setFlashing(true);
    try {
      const res = await fetch("/api/flash", { method: "POST" });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "flashに失敗しました");
      toast.success("flash完了");
    } catch (err) {
      toast.error(`flash失敗: ${(err as Error).message}`);
    } finally {
      setFlashing(false);
    }
  };

  const postSend = (body: Record<string, unknown>) =>
    guardedSend((force) =>
      apiFetch("/api/send", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode: MODE, ...body, force }),
      }),
    );

  const sendOne = async (scope: SendScope, file: string) => {
    setSending(file);
    try {
      const res = await postSend({ scope, file });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "送信に失敗しました");
      toast.success(`${file}: 送信完了`);
    } catch (err) {
      if (!(err instanceof SendCancelled)) toast.error(`${file}: ${(err as Error).message}`);
    } finally {
      setSending(null);
      void refreshProfiles();
    }
  };

  const openEditor = async (scope: SendScope, file: string, machineId: string | null = machineRef.current) => {
    if (!machineId) return;
    setShowTemplates(false);
    setShowCompare(false);
    setShowMatrix(false);
    setEditing({ machine: machineId, scope, file });
    setEditorContent(null);
    setPropagate([]);
    try {
      const res = await fetch(
        `/api/profile-file?machine=${encodeURIComponent(machineId)}&mode=${encodeURIComponent(MODE)}&scope=${scope}&file=${encodeURIComponent(file)}`
      );
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "読み込みに失敗しました");
      setEditorContent(data.content as string);
      setLiveDraft(data.content as string);
    } catch (err) {
      toast.error(`${file}: ${(err as Error).message}`);
      setEditing(null);
    }
  };

  const closeEditor = () => {
    setEditing(null);
    setEditorContent(null);
    setLiveDraft(null);
    setPropagate([]);
  };

  // The slalom sim panel patched the draft (not the saved file) - just
  // update the buffer and force YamlEditor to pick it up; the user still
  // reviews and explicitly saves via Ctrl+S like any other edit.
  const applySimResultToDraft = (nextDraft: string) => {
    setLiveDraft(nextDraft);
    setDraftPatchNonce((n) => n + 1);
  };

  const saveEditor = async (content: string): Promise<boolean> => {
    if (!editing) return false;
    setSaving(true);
    try {
      const res = await fetch("/api/profile-file", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ machine: editing.machine, mode: MODE, scope: editing.scope, file: editing.file, content }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "保存に失敗しました");
      const label = (registry && findMachine(registry, editing.machine)?.label) ?? editing.machine;
      toast.success(`${label} の ${editing.file}: 保存しました`);
      // Stay in the editor; sync editorContent so the dirty flag clears
      // instead of staying stuck true (YamlEditor keeps its own draft state).
      setEditorContent(content);
      setPropagate((data.propagate as PropagateSuggestion[] | undefined) ?? []);
      setDiffNonce((n) => n + 1);
      void refreshProfiles();
      return true;
    } catch (err) {
      toast.error(`${editing.file}: ${(err as Error).message}`);
      return false;
    } finally {
      setSaving(false);
    }
  };

  // 保存した変更を、同じ値だったほかの機体にも入れる
  const applyPropagation = async (s: PropagateSuggestion) => {
    if (!editing) return;
    const file = relFile(editing);
    try {
      const res = await fetch("/api/machines/sync", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ from: editing.machine, to: s.machine, file, paths: s.paths.map((p) => p.segs) }),
      });
      const data = (await res.json()) as SyncResult & { error?: string };
      if (!res.ok) throw new Error(data.error ?? "書き換えに失敗しました");
      const label = (registry && findMachine(registry, s.machine)?.label) ?? s.machine;
      if (data.applied.length > 0) {
        toast.success(`${label} の ${file} にも入れました: ${data.applied.join(", ")}`, {
          description: "機体へはまだ送っていません(その機体をつないで送信)",
          action: data.undo
            ? {
                label: "元に戻す",
                onClick: () =>
                  void fetch("/api/machines/sync", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ undo: data.undo }),
                  })
                    .then((r) => r.json())
                    .then((d) => {
                      if (d.error) toast.error(d.error);
                      else toast.success(`${d.label}: 元に戻しました`);
                      setDiffNonce((n) => n + 1);
                    }),
              }
            : undefined,
        });
      }
      if (data.errors.length > 0) {
        toast.error(`${label}: ${data.errors.map((e) => `${e.path}: ${e.error}`).join(" / ")}`);
      }
      setPropagate((prev) => prev.filter((x) => x.machine !== s.machine));
      setDiffNonce((n) => n + 1);
    } catch (err) {
      toast.error((err as Error).message);
    }
  };

  // Uploading am32.yaml only writes it to the device's LittleFS - the ESC
  // itself is untouched until write_am32_param() runs. "sync" does both
  // (send_file.py am32sync); "read" dumps the ESC's current values instead.
  const runAm32 = async (action: Am32Action) => {
    setAm32Action(action);
    const label = action === "sync" ? "ESC書込" : "ESC読出";
    try {
      const res = await guardedSend((force) =>
        apiFetch("/api/am32", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ action, mode: MODE, force }),
        }),
      );
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? `${label}に失敗しました`);
      toast.success(`${label}: 完了 (詳細はコンソール)`);
    } catch (err) {
      if (!(err instanceof SendCancelled)) toast.error(`${label}: ${(err as Error).message}`);
    } finally {
      setAm32Action(null);
      void refreshProfiles();
    }
  };

  // am32.yaml の編集画面から「保存 → 送信 → ESC書込」を1操作で回すためのもの。
  // 未保存のドラフトがあるときは先に保存する(ESCへ送られるのは保存済みの
  // ファイル内容なので、保存を挟まないと編集が反映されないままになる)。
  const saveAndSyncAm32 = async () => {
    const draft = liveDraft ?? editorContent;
    if (draft !== null && draft !== editorContent) {
      const ok = await saveEditor(draft);
      if (!ok) return;
    }
    await runAm32("sync");
  };

  const openTemplates = () => {
    setEditing(null);
    setEditorContent(null);
    setShowMatrix(false);
    setShowCompare(false);
    setShowTemplates(true);
  };

  const openMatrix = () => {
    setEditing(null);
    setEditorContent(null);
    setShowTemplates(false);
    setShowCompare(false);
    setShowMatrix(true);
  };

  const toggleCompare = () => {
    setShowMatrix(false);
    setShowTemplates(false);
    setEditing(null);
    setEditorContent(null);
    setShowCompare((v) => !v);
  };

  const applyTemplate = async (id: string) => {
    setApplyingTemplate(id);
    try {
      const res = await apiFetch("/api/test-templates/apply", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ id }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "適用に失敗しました");
      toast.success(`system.yaml に適用しました: ${data.name}`);
      void refreshProfiles();
    } catch (err) {
      toast.error((err as Error).message);
    } finally {
      setApplyingTemplate(null);
    }
  };

  const saveTemplate = async (
    id: string | undefined,
    name: string,
    values: TestTemplateValues,
    arrayValues: TestTemplateArrayValues
  ) => {
    setSavingTemplate(true);
    try {
      const res = await apiFetch("/api/test-templates", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ id, name, values, arrayValues }),
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
      const res = await apiFetch(`/api/test-templates?id=${encodeURIComponent(id)}`, { method: "DELETE" });
      if (!res.ok) throw new Error("削除に失敗しました");
      await refreshTemplates();
    } catch (err) {
      toast.error((err as Error).message);
    }
  };

  const sendMany = async (sentinel: string, body: Record<string, unknown>, done: (n: number) => string) => {
    setSending(sentinel);
    try {
      const res = await postSend(body);
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "送信に失敗しました");
      toast.success(done(Number(data.count ?? 0)));
    } catch (err) {
      if (!(err instanceof SendCancelled)) toast.error(`送信エラー: ${(err as Error).message}`);
    } finally {
      setSending(null);
      void refreshProfiles();
    }
  };
  const sendAll = () => sendMany(ALL_SENTINEL, { all: true }, () => "全ファイル送信完了");
  const sendUnsent = () => sendMany(UNSENT_SENTINEL, { unsent: true }, (n) => `未送信の ${n} ファイルを送信しました`);

  // LogPlotPanel など重いパネルが、機体と関係ない描き直し(ログ 1 行ごと)で
  // 作り直されないように、コンテキストの値は中身が変わったときだけ入れ替える。
  const machineContext = useMemo<MachineContextValue>(
    () => ({
      registry: registry ?? { default: null, machines: [], specific: {} },
      current: machine,
      board,
      guardedSend,
    }),
    [registry, machine, board, guardedSend],
  );

  const currentMachine = registry ? findMachine(registry, machine) : null;
  const boardMachine = registry ? findMachine(registry, board.machine) : null;
  const editingMachine = registry && editing ? findMachine(registry, editing.machine) : null;

  const diff = useMachineDiff(
    registry,
    editing?.machine ?? null,
    editing ? relFile(editing) : null,
    editing ? (liveDraft ?? editorContent) : null,
    diffNonce,
  );

  const editorMachineProps = editing
    ? {
        machine: editingMachine,
        marks: diff.marks,
        markSummary:
          diff.todo + diff.specific > 0 ? (
            <span
              className="flex items-center gap-1.5 text-xs"
              title="ほかの機体と値が違うキー。行の左の帯(琥珀 = 未整理、紫 = 固有)にマウスを載せると相手の値が出る"
              data-editor-diff-summary
            >
              {diff.todo > 0 && <span className="text-amber-300">未整理 {diff.todo}</span>}
              {diff.specific > 0 && <span className="text-violet-300">固有 {diff.specific}</span>}
            </span>
          ) : undefined,
        banner: (
          <>
            {editing.machine !== machine && (
              <div
                className="flex items-center gap-2 rounded-md border border-amber-500/60 bg-amber-500/10 px-2 py-1 text-xs text-amber-200"
                data-editor-other-machine
              >
                <span>
                  表示中の機体は {currentMachine?.label ?? "?"} ですが、このファイルは{" "}
                  <b>{editingMachine?.label ?? editing.machine}</b> のものです。保存先は{" "}
                  {editingMachine?.label ?? editing.machine} です。
                </span>
              </div>
            )}
            {diff.missingHere.length > 0 && (
              <div className="rounded-md border border-amber-500/40 px-2 py-1 text-xs text-amber-200" data-editor-missing>
                ほかの機体にあって、この機体に無いキー: {diff.missingHere.slice(0, 8).join(", ")}
                {diff.missingHere.length > 8 && ` ほか ${diff.missingHere.length - 8} 個`}
                (「機体比較」で追加できます)
              </div>
            )}
            {propagate.map((s) => {
              const target = registry ? findMachine(registry, s.machine) : null;
              return (
                <div
                  key={s.machine}
                  className="flex flex-wrap items-center gap-2 rounded-md border border-primary-bright/50 bg-primary-bright/10 px-2 py-1 text-xs"
                  data-propagate={s.machine}
                >
                  <MachineChip machine={target} fallback={s.machine} />
                  <span className="min-w-0 flex-1" title={s.paths.map((p) => `${p.path}: ${p.from ?? "(なし)"} → ${p.to}`).join("\n")}>
                    も同じ値でした: <span className="font-mono">{s.paths.slice(0, 6).map((p) => p.path).join(", ")}</span>
                    {s.paths.length > 6 && ` ほか ${s.paths.length - 6} 個`}
                  </span>
                  <Button size="xs" onClick={() => void applyPropagation(s)}>
                    {target?.label ?? s.machine} にも同じ変更を入れる
                  </Button>
                  <Button
                    size="xs"
                    variant="ghost"
                    title="この機体だけの変更にする"
                    onClick={() => setPropagate((prev) => prev.filter((x) => x.machine !== s.machine))}
                  >
                    入れない
                  </Button>
                </div>
              );
            })}
          </>
        ),
      }
    : {};

  const fullWidth = showMatrix || showCompare;

  return (
    <MachineContext.Provider value={machineContext}>
    <div className="flex h-screen flex-col gap-2 p-2">
      <PortPanel
        ports={ports}
        connectedPath={connectedPath}
        status={status}
        autoConnect={autoConnect}
        flashing={flashing}
        onDisconnect={handleDisconnect}
        onEnableAutoConnect={handleEnableAutoConnect}
        onFlash={handleFlash}
        accentColor={currentMachine?.color}
        actions={
          <ThemePicker
            prefs={themePrefs ?? DEFAULT_PREFS}
            onChange={changeTheme}
            machine={currentMachine}
            onMachineColor={setMachineColor}
          />
        }
        machines={
          registry ? (
            <MachineBar
              registry={registry}
              current={machine}
              board={board}
              compareActive={showCompare}
              onSelect={selectMachine}
              onRegisterBoard={(id) => void registerBoard(id)}
              onOpenManage={() => setShowMachineDialog(true)}
              onToggleCompare={toggleCompare}
            />
          ) : undefined
        }
        tabs={
          !fullWidth && !showTemplates && !editing ? (
            <>
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
              <Button
                size="sm"
                variant={rightTab === "calib" ? "default" : "outline"}
                onClick={() => setRightTab("calib")}
                title="テストモード15の生値からセンサー距離換算(sensor.yaml gain)を求める"
              >
                センサ校正
              </Button>
              <Button
                size="sm"
                variant={rightTab === "maze" ? "default" : "outline"}
                onClick={() => setRightTab("maze")}
                title="迷路ファイル(受信ログ・編集用 .maze・大会迷路)の表示と壁の編集"
              >
                迷路
              </Button>
            </>
          ) : undefined
        }
      />
      {registry === null ? (
        <Card className="flex flex-1 flex-col items-center justify-center gap-2">
          {registryError ? (
            <>
              <span className="text-sm text-destructive">機体の登録簿(tools/param_tuner/machines.yaml)を読めません</span>
              <pre className="max-w-3xl text-xs whitespace-pre-wrap text-muted-foreground">{registryError}</pre>
              <span className="text-xs text-muted-foreground">直してからページを読み込み直してください。</span>
            </>
          ) : (
            <span className="text-sm text-muted-foreground">機体の一覧を読み込み中...</span>
          )}
        </Card>
      ) : machine === null ? (
        <Card className="flex flex-1 flex-col items-center justify-center gap-2">
          <span className="text-sm text-muted-foreground">機体がまだ登録されていません。</span>
          <Button onClick={() => setShowMachineDialog(true)}>機体を追加</Button>
        </Card>
      ) : showCompare ? (
        <div className="flex flex-1 overflow-hidden">
          <MachineComparePanel
            mode={MODE}
            onClose={() => setShowCompare(false)}
            onFilesChanged={() => {
              void refreshProfiles();
              setDiffNonce((n) => n + 1);
            }}
            onRegistry={applyRegistry}
            onEdit={(m, file) => {
              const slash = file.indexOf("/");
              void openEditor(slash < 0 ? "base" : "mode", slash < 0 ? file : file.slice(slash + 1), m);
            }}
          />
        </div>
      ) : showMatrix ? (
        <div className="flex flex-1 overflow-hidden">
          <ParamMatrixPanel key={machine} onClose={() => setShowMatrix(false)} />
        </div>
      ) : (
      <ResizablePanelGroup direction="horizontal" autoSaveId="param-console-main" className="flex-1 overflow-hidden">
        <ResizablePanel defaultSize={22} minSize={15} maxSize={40} className="min-w-0">
          <ProfilePanel
            profiles={profiles}
            sending={sending}
            machine={currentMachine}
            boardMachine={boardMachine}
            onSendFile={sendOne}
            onSendAll={() => void sendAll()}
            onSendUnsent={() => void sendUnsent()}
            onEditFile={(scope, file) => void openEditor(scope, file)}
            onOpenTemplates={openTemplates}
            onOpenMatrix={openMatrix}
            am32Action={am32Action}
            onAm32Sync={() => void runAm32("sync")}
            onAm32Read={() => void runAm32("read")}
          />
        </ResizablePanel>
        <ResizableHandle withHandle />
        <ResizablePanel defaultSize={78} minSize={30} className="min-w-0 overflow-hidden">
          {editing ? (
            editorContent === null ? (
              <Card className="flex h-full items-center justify-center overflow-hidden">
                <span className="text-sm text-muted-foreground">読み込み中...</span>
              </Card>
            ) : editing.scope === "mode" && TURN_PROFILE_FILE_RE.test(editing.file) ? (
              <ResizablePanelGroup direction="horizontal" autoSaveId="param-console-editor" className="h-full">
                <ResizablePanel defaultSize={60} minSize={30} className="min-w-0">
                  <YamlEditor
                    key={`${editing.machine}:${editing.scope}:${editing.file}:${draftPatchNonce}`}
                    file={editing.file}
                    content={editorContent}
                    initialDraft={liveDraft ?? undefined}
                    saving={saving}
                    onSave={saveEditor}
                    onClose={closeEditor}
                    onDraftChange={setLiveDraft}
                    {...editorMachineProps}
                  />
                </ResizablePanel>
                <ResizableHandle withHandle />
                <ResizablePanel defaultSize={40} minSize={20} className="min-w-0">
                  <SlalomSimPanel
                    file={editing.file}
                    draft={liveDraft ?? editorContent}
                    onApply={applySimResultToDraft}
                  />
                </ResizablePanel>
              </ResizablePanelGroup>
            ) : (
              <YamlEditor
                key={`${editing.machine}:${editing.scope}:${editing.file}:${draftPatchNonce}`}
                file={editing.file}
                content={editorContent}
                initialDraft={liveDraft ?? undefined}
                saving={saving}
                onSave={saveEditor}
                onClose={closeEditor}
                onDraftChange={setLiveDraft}
                {...editorMachineProps}
                headerActions={
                  editing.scope === "base" && editing.file === AM32_FILE && editing.machine === machine ? (
                    <Button
                      size="sm"
                      variant="secondary"
                      disabled={saving || am32Action !== null}
                      onClick={() => void saveAndSyncAm32()}
                    >
                      {am32Action === "sync" ? "書込中..." : "保存してESC書込"}
                    </Button>
                  ) : undefined
                }
              />
            )
          ) : showTemplates ? (
            <TestTemplatePanel
              key={machine}
              templates={templates}
              applying={applyingTemplate}
              saving={savingTemplate}
              onApply={applyTemplate}
              onSave={saveTemplate}
              onDelete={deleteTemplate}
              onClose={() => setShowTemplates(false)}
            />
          ) : (
            <div className="flex h-full min-h-0 flex-col overflow-hidden">
              {/* タブの切り替えボタンは PortPanel(ヘッダーバー)側に出している。
                  Both tabs render inside the same flex column; only their
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
                <LogPlotPanel autoOpen={plotAutoOpen} onAutoOpenHandled={handlePlotAutoOpenHandled} />
              </div>
              {/* 校正パネルは Space キーを記録に使うので、表示中だけマウントする
                  (位置表は localStorage に機体ごとに残るのでタブを切り替えても消えない)。
                  sensor.yaml は機体ごとなので、機体を切り替えたら作り直す。 */}
              {rightTab === "calib" && (
                <div className="flex min-h-0 flex-1">
                  <SensorCalibPanel
                    key={machine}
                    machine={machine}
                    legacyStorage={registry.machines[0]?.id === machine}
                    connected={status === "connected"}
                  />
                </div>
              )}
              {/* 編集中の迷路を残すため、迷路タブは隠すだけでマウントしたままにする。 */}
              <div className={`min-h-0 flex-1 ${rightTab === "maze" ? "flex" : "hidden"}`}>
                <MazePanel
                  active={rightTab === "maze"}
                  autoOpen={mazeAutoOpen}
                  onAutoOpenHandled={() => setMazeAutoOpen(null)}
                  refreshNonce={mazeRefreshNonce}
                />
              </div>
            </div>
          )}
        </ResizablePanel>
      </ResizablePanelGroup>
      )}
      {showMachineDialog && registry && (
        <MachineDialog
          registry={registry}
          board={board}
          onClose={() => setShowMachineDialog(false)}
          onChanged={(next, nextBoard, created) => {
            applyRegistry(next, nextBoard);
            // 追加した機体をそのまま表示する(最初の 1 台、または基板を登録して足したとき)
            if (created && (machineRef.current === null || nextBoard.machine === created)) selectMachine(created);
          }}
        />
      )}
      <ChoiceDialog request={choice} />
    </div>
    </MachineContext.Provider>
  );
}
