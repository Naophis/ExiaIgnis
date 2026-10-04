"use client";

import { useEffect, useRef, useState } from "react";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import {
  MACHINE_ID_RE,
  shortSerial,
  type BoardInfo,
  type MachineRegistry,
} from "@/lib/machine-shared";

interface GitSource {
  ref: string;
  date: string;
  hash: string;
  subject: string;
}

interface Props {
  registry: MachineRegistry;
  board: BoardInfo;
  onClose: () => void;
  // 登録簿を書き換えたあと(機体を足したときは created にその id)
  onChanged: (registry: MachineRegistry, board: BoardInfo, created?: string) => void;
}

async function post(body: Record<string, unknown>): Promise<{ registry: MachineRegistry; board: BoardInfo }> {
  const res = await fetch("/api/machines", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const data = await res.json();
  if (!res.ok) throw new Error(data.error ?? "失敗しました");
  return data;
}

// 色の選択。選んでいる間は input イベントが連続で来るので、手が止まってから 1 回だけ保存する。
function ColorInput({ value, onCommit }: { value: string; onCommit: (color: string) => void }) {
  const [draft, setDraft] = useState(value);
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);
  useEffect(() => () => void (timer.current && clearTimeout(timer.current)), []);
  return (
    <input
      type="color"
      value={draft}
      title="この機体の色。テーマを「機体ごとに色を変える」にしているとき(既定)は、この機体を表示中の画面全体の色になる"
      className="h-6 w-7 cursor-pointer rounded border-0 bg-transparent p-0"
      onChange={(e) => {
        const next = e.target.value;
        setDraft(next);
        if (timer.current) clearTimeout(timer.current);
        timer.current = setTimeout(() => {
          if (next !== value) onCommit(next);
        }, 400);
      }}
    />
  );
}

// 機体の追加と、名前・色・基板(USB シリアル番号)の登録。
export function MachineDialog({ registry, board, onClose, onChanged }: Props) {
  const [sources, setSources] = useState<{ git: GitSource[]; legacy: boolean } | null>(null);
  const [newId, setNewId] = useState("");
  // "machine:<id>" / "git:<ref>" / "legacy"
  const [source, setSource] = useState<string>(registry.machines[0] ? `machine:${registry.machines[0].id}` : "");
  const unregistered = board.serial !== null && board.machine === null;
  const [registerBoard, setRegisterBoard] = useState(unregistered);
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    void fetch("/api/machines?action=sources")
      .then((r) => r.json())
      .then((d) => {
        setSources({ git: d.git ?? [], legacy: Boolean(d.legacy) });
        setSource((cur) => cur || (d.legacy ? "legacy" : d.git?.[0] ? `git:${d.git[0].ref}` : ""));
      })
      .catch(() => setSources({ git: [], legacy: false }));
  }, []);

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") onClose();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [onClose]);

  const run = async (body: Record<string, unknown>, done?: string, created?: string) => {
    setBusy(true);
    try {
      const data = await post(body);
      onChanged(data.registry, data.board, created);
      if (done) toast.success(done);
      return true;
    } catch (err) {
      toast.error((err as Error).message);
      return false;
    } finally {
      setBusy(false);
    }
  };

  const idOk = MACHINE_ID_RE.test(newId) && !registry.machines.some((m) => m.id === newId);
  const create = async () => {
    const src = source.startsWith("machine:")
      ? { type: "machine", id: source.slice(8) }
      : source.startsWith("git:")
        ? { type: "git", ref: source.slice(4) }
        : { type: "legacy" };
    const ok = await run(
      { action: "create", id: newId, source: src, registerBoard },
      `機体 ${newId} を追加しました(machines/${newId}/profile)`,
      newId,
    );
    if (ok) setNewId("");
  };

  return (
    <div
      className="fixed inset-0 z-50 flex items-center justify-center bg-black/60 p-4"
      onMouseDown={(e) => {
        if (e.target === e.currentTarget) onClose();
      }}
    >
      <div
        role="dialog"
        aria-modal="true"
        data-machine-dialog
        className="flex max-h-full w-full max-w-2xl flex-col gap-3 overflow-auto rounded-xl bg-card p-4 text-sm shadow-xl ring-1 ring-foreground/20"
      >
        <div className="flex items-center justify-between">
          <span className="text-base font-semibold">機体</span>
          <Button size="sm" variant="outline" onClick={onClose}>
            閉じる
          </Button>
        </div>
        <p className="text-xs text-muted-foreground">
          機体ごとにパラメータ一式(machines/&lt;名前&gt;/profile)を持ちます。基板を登録しておくと、つないだときに
          USB のシリアル番号で機体を見分け、別の機体のパラメータを送ろうとすると止めます。
        </p>

        <div className="flex flex-col gap-1.5">
          {registry.machines.length === 0 && (
            <span className="text-muted-foreground">機体がまだありません。下で追加してください。</span>
          )}
          {registry.machines.map((m) => (
            <div key={m.id} className="flex flex-wrap items-center gap-2 rounded-md border px-2 py-1.5" data-machine-row={m.id}>
              <ColorInput
                value={m.color}
                onCommit={(color) => void run({ action: "update", id: m.id, color })}
              />
              <Input
                key={`${m.id}:${m.label}`}
                defaultValue={m.label}
                className="h-7 w-32"
                title="画面に出す名前"
                onBlur={(e) => {
                  const v = e.target.value.trim();
                  if (v && v !== m.label) void run({ action: "update", id: m.id, label: v });
                }}
              />
              <span className="font-mono text-xs text-muted-foreground">machines/{m.id}/</span>
              <div className="flex min-w-0 flex-1 flex-wrap items-center gap-1">
                {m.serials.length === 0 && <span className="text-xs text-amber-300">基板が未登録</span>}
                {m.serials.map((s) => (
                  <span
                    key={s}
                    title={s}
                    className={`inline-flex items-center gap-1 rounded border px-1 font-mono text-xs ${s === board.serial ? "border-primary-bright text-primary-bright" : ""}`}
                  >
                    {shortSerial(s)}
                    <button
                      type="button"
                      className="text-muted-foreground hover:text-destructive"
                      title="この基板の登録を外す"
                      onClick={() => void run({ action: "removeSerial", id: m.id, serial: s })}
                    >
                      ×
                    </button>
                  </span>
                ))}
                {board.serial && !m.serials.includes(board.serial) && (
                  <Button
                    size="xs"
                    variant="outline"
                    disabled={busy}
                    title={`いまつないでいる基板 (${board.serial}) をこの機体に登録する`}
                    onClick={() =>
                      void run({ action: "assignSerial", id: m.id }, `接続中の基板を ${m.label} に登録しました`)
                    }
                  >
                    接続中の基板を登録
                  </Button>
                )}
              </div>
              <label
                className="flex cursor-pointer items-center gap-1 text-xs text-muted-foreground"
                title="機体を指定しないスクリプト(update_param.sh・check_*.py など)が使う機体"
              >
                <input
                  type="radio"
                  name="default-machine"
                  checked={registry.default === m.id}
                  onChange={() => void run({ action: "setDefault", id: m.id })}
                />
                CLI の既定
              </label>
            </div>
          ))}
        </div>

        <div className="flex flex-col gap-1.5 rounded-md border border-dashed px-2 py-2">
          <span className="font-medium">機体を追加</span>
          <div className="flex flex-wrap items-center gap-2">
            <Input
              placeholder="名前 (例: 1st)"
              value={newId}
              className="h-7 w-36"
              onChange={(e) => setNewId(e.target.value.trim())}
              data-new-machine-id
            />
            <span className="text-xs text-muted-foreground">パラメータの元</span>
            <select
              value={source}
              onChange={(e) => setSource(e.target.value)}
              className="h-7 min-w-0 flex-1 rounded-md border border-input bg-background px-1.5 text-sm"
              data-new-machine-source
            >
              {registry.machines.map((m) => (
                <option key={m.id} value={`machine:${m.id}`}>
                  機体 {m.label} をコピー
                </option>
              ))}
              {sources?.legacy && <option value="legacy">以前の tools/param_tuner/profile を取り込む</option>}
              {sources?.git.map((g) => (
                <option key={g.ref} value={`git:${g.ref}`} title={g.subject}>
                  ブランチ {g.ref} の profile を取り込む ({g.date} {g.hash})
                </option>
              ))}
            </select>
          </div>
          <div className="flex flex-wrap items-center gap-2">
            {board.serial && (
              <label className="flex cursor-pointer items-center gap-1 text-xs">
                <input type="checkbox" checked={registerBoard} onChange={(e) => setRegisterBoard(e.target.checked)} />
                いまつないでいる基板 ({shortSerial(board.serial)}) をこの機体に登録
              </label>
            )}
            <div className="flex-1" />
            {newId && !idOk && (
              <span className="text-xs text-destructive">
                {registry.machines.some((m) => m.id === newId) ? "同じ名前の機体があります" : "英数字・_・- で、先頭は英数字"}
              </span>
            )}
            <Button size="sm" disabled={busy || !idOk || !source} onClick={() => void create()} data-create-machine>
              追加
            </Button>
          </div>
          <span className="text-xs text-muted-foreground">
            追加するとパラメータ一式がコピーされ、その機体のものとして別々に編集できます。違いは「機体比較」で確認・同期できます。
            機体を消すときは machines/&lt;名前&gt;/ フォルダと machines.yaml の該当行を消してください。
          </span>
        </div>
      </div>
    </div>
  );
}
