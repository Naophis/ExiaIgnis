"use client";

import { useEffect, useState } from "react";
import { toast } from "sonner";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { ScrollArea } from "@/components/ui/scroll-area";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { Separator } from "@/components/ui/separator";
import { cn } from "@/lib/utils";
import {
  TEST_TEMPLATE_KEY_OPTIONS,
  TEST_TEMPLATE_KEYS,
  type NamedOption,
  type TestTemplate,
  type TestTemplateKey,
  type TestTemplateValues,
} from "@/lib/test-template-shared";

// Keys exposed in the "quick apply" strip: no saved template needed, just
// punch in a value (or nudge it by 1) and it's written to system.yaml right
// away. Meant for sweeps like trying sla_type/sla_type2 through several
// turn types at a fixed file_idx, where saving/deleting a named template
// per value would just be clutter.
const QUICK_APPLY_KEYS: TestTemplateKey[] = ["mode", "file_idx", "sla_type", "sla_type2", "sla_return"];

interface Props {
  templates: TestTemplate[];
  applying: string | null;
  saving: boolean;
  onApply: (id: string) => void;
  onSave: (id: string | undefined, name: string, values: TestTemplateValues) => void;
  onDelete: (id: string) => void;
  onClose: () => void;
}

type FormState = Record<TestTemplateKey, string>;

const emptyForm = (): FormState =>
  Object.fromEntries(TEST_TEMPLATE_KEYS.map((k) => [k, ""])) as FormState;

const formFromValues = (values: TestTemplateValues): FormState => {
  const form = emptyForm();
  for (const key of TEST_TEMPLATE_KEYS) {
    if (values[key] !== undefined) form[key] = String(values[key]);
  }
  return form;
};

export function TestTemplatePanel({
  templates,
  applying,
  saving,
  onApply,
  onSave,
  onDelete,
  onClose,
}: Props) {
  const [editingId, setEditingId] = useState<string | "new" | null>(null);
  const [name, setName] = useState("");
  const [form, setForm] = useState<FormState>(emptyForm());

  const [quickValues, setQuickValues] = useState<TestTemplateValues>({});
  const [quickApplying, setQuickApplying] = useState<TestTemplateKey | null>(null);
  // file_idx/mode options are loaded from the server (file_idx indexes into
  // profiles.yaml's list; mode's menu is read from system.yaml's own
  // comments) rather than living in the static TEST_TEMPLATE_KEY_OPTIONS.
  const [fileIdxOptions, setFileIdxOptions] = useState<NamedOption[]>([]);
  const [modeOptions, setModeOptions] = useState<NamedOption[]>([]);

  useEffect(() => {
    void fetch("/api/test-templates/quick-apply")
      .then((res) => res.json())
      .then((data) => setQuickValues(data.values as TestTemplateValues));
    void fetch("/api/test-templates/file-idx-options")
      .then((res) => res.json())
      .then((data) => setFileIdxOptions((data.options as NamedOption[]) ?? []));
    void fetch("/api/test-templates/mode-options")
      .then((res) => res.json())
      .then((data) => setModeOptions((data.options as NamedOption[]) ?? []));
  }, []);

  const quickApply = async (key: TestTemplateKey, next: number) => {
    if (Number.isNaN(next)) return;
    setQuickApplying(key);
    try {
      const res = await fetch("/api/test-templates/quick-apply", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ values: { [key]: next } }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? "適用に失敗しました");
      setQuickValues(data.values as TestTemplateValues);
      toast.success(`${key}: ${next} を system.yaml に適用しました`);
    } catch (err) {
      toast.error((err as Error).message);
    } finally {
      setQuickApplying(null);
    }
  };

  const startNew = () => {
    setEditingId("new");
    setName("");
    setForm(emptyForm());
  };

  const startEdit = (t: TestTemplate) => {
    setEditingId(t.id);
    setName(t.name);
    setForm(formFromValues(t.values));
  };

  const submit = () => {
    const values: TestTemplateValues = {};
    for (const key of TEST_TEMPLATE_KEYS) {
      const raw = form[key].trim();
      if (raw !== "") values[key] = Number(raw);
    }
    onSave(editingId === "new" ? undefined : (editingId ?? undefined), name, values);
    setEditingId(null);
  };

  return (
    <Card className="flex flex-1 flex-col overflow-hidden">
      <CardHeader className="flex-row items-center justify-between space-y-0">
        <CardTitle>system.yaml テストテンプレート</CardTitle>
        <Button size="sm" variant="outline" onClick={onClose}>
          閉じる
        </Button>
      </CardHeader>
      <CardContent className="flex flex-1 flex-col gap-3 overflow-hidden">
        <div className="flex flex-col gap-2 rounded-md border p-3">
          <span className="text-xs font-medium text-muted-foreground">クイック適用</span>
          {QUICK_APPLY_KEYS.map((key) => {
            const options =
              key === "file_idx"
                ? fileIdxOptions
                : key === "mode"
                  ? modeOptions
                  : TEST_TEMPLATE_KEY_OPTIONS[key];
            return options ? (
              <QuickApplySelectRow
                key={key}
                label={key}
                options={options}
                value={quickValues[key]}
                applying={quickApplying === key}
                onApply={(next) => quickApply(key, next)}
              />
            ) : (
              <QuickApplyRow
                // Remount whenever the live value changes (initial load, or
                // after a successful apply) so the draft input re-seeds from
                // it - without remounting on every keystroke while the value
                // itself hasn't changed yet.
                key={`${key}:${quickValues[key]}`}
                label={key}
                value={quickValues[key]}
                applying={quickApplying === key}
                onApply={(next) => quickApply(key, next)}
              />
            );
          })}
        </div>
        <Separator />
        {editingId ? (
          <div className="flex flex-col gap-2 rounded-md border p-3">
            <Input
              placeholder="テンプレート名"
              value={name}
              onChange={(e) => setName(e.target.value)}
            />
            <div className="grid grid-cols-3 gap-2">
              {TEST_TEMPLATE_KEYS.map((key) => {
                const options = TEST_TEMPLATE_KEY_OPTIONS[key];
                return (
                  <label key={key} className="flex flex-col gap-1 text-xs text-muted-foreground">
                    {key}
                    {options ? (
                      <Select
                        value={form[key]}
                        onValueChange={(v) =>
                          setForm((prev) => ({ ...prev, [key]: v ?? "" }))
                        }
                      >
                        <SelectTrigger className="w-full">
                          <SelectValue placeholder="(未指定)" />
                        </SelectTrigger>
                        <SelectContent>
                          {options.map((opt) => (
                            <SelectItem key={opt.value} value={String(opt.value)}>
                              {opt.label} ({opt.value})
                            </SelectItem>
                          ))}
                        </SelectContent>
                      </Select>
                    ) : (
                      <Input
                        inputMode="decimal"
                        placeholder="(未指定)"
                        value={form[key]}
                        onChange={(e) => setForm((prev) => ({ ...prev, [key]: e.target.value }))}
                      />
                    )}
                  </label>
                );
              })}
            </div>
            <div className="flex justify-end gap-2">
              <Button size="sm" variant="outline" onClick={() => setEditingId(null)}>
                キャンセル
              </Button>
              <Button size="sm" onClick={submit} disabled={!name.trim() || saving}>
                {saving ? "保存中..." : "保存"}
              </Button>
            </div>
          </div>
        ) : (
          <Button onClick={startNew} className="w-full">
            ＋ 新規テンプレート
          </Button>
        )}
        <Separator />
        <ScrollArea className="min-h-0 flex-1">
          <div className="flex flex-col gap-2 pr-2">
            {templates.map((t) => (
              <div
                key={t.id}
                role="button"
                tabIndex={0}
                onClick={() => startEdit(t)}
                onKeyDown={(e) => {
                  if (e.key === "Enter" || e.key === " ") startEdit(t);
                }}
                className="flex cursor-pointer flex-col gap-1 rounded-md border p-2 hover:bg-muted"
              >
                <div className="flex items-center justify-between gap-2">
                  <span className="text-sm font-medium">{t.name}</span>
                  <div className="flex shrink-0 gap-1">
                    <Button
                      size="sm"
                      variant="ghost"
                      onClick={(e) => {
                        e.stopPropagation();
                        onDelete(t.id);
                      }}
                    >
                      削除
                    </Button>
                    <Button
                      size="sm"
                      onClick={(e) => {
                        e.stopPropagation();
                        onApply(t.id);
                      }}
                      disabled={applying !== null}
                    >
                      {applying === t.id ? "適用中..." : "適用"}
                    </Button>
                  </div>
                </div>
                <span className="text-xs text-muted-foreground">
                  {TEST_TEMPLATE_KEYS.filter((k) => t.values[k] !== undefined)
                    .map((k) => `${k}: ${t.values[k]}`)
                    .join(", ") || "(値なし)"}
                </span>
              </div>
            ))}
            {templates.length === 0 && (
              <span className="px-2 py-1 text-sm text-muted-foreground">テンプレートがありません</span>
            )}
          </div>
        </ScrollArea>
      </CardContent>
    </Card>
  );
}

function QuickApplyRow({
  label,
  value,
  applying,
  onApply,
}: {
  label: string;
  value: number | undefined;
  applying: boolean;
  onApply: (next: number) => void;
}) {
  const [draft, setDraft] = useState(value !== undefined ? String(value) : "");
  const disabled = applying || value === undefined;

  return (
    <div className="flex items-center gap-2">
      <span className="w-20 shrink-0 text-sm">{label}</span>
      <span className="w-24 shrink-0 text-xs text-muted-foreground">
        現在: {value ?? "-"}
      </span>
      <Button size="sm" variant="outline" disabled={disabled} onClick={() => onApply((value ?? 0) - 1)}>
        -1
      </Button>
      <Input
        className="w-20"
        inputMode="numeric"
        value={draft}
        onChange={(e) => setDraft(e.target.value)}
      />
      <Button size="sm" variant="outline" disabled={disabled} onClick={() => onApply((value ?? 0) + 1)}>
        +1
      </Button>
      <Button size="sm" disabled={applying || draft.trim() === ""} onClick={() => onApply(Number(draft))}>
        {applying ? "..." : "適用"}
      </Button>
    </div>
  );
}

function QuickApplySelectRow({
  label,
  options,
  value,
  applying,
  onApply,
}: {
  label: string;
  options: NamedOption[];
  value: number | undefined;
  applying: boolean;
  onApply: (next: number) => void;
}) {
  return (
    <div className="flex items-start gap-2">
      <span className="w-20 shrink-0 pt-1.5 text-sm">{label}</span>
      {value === undefined || options.length === 0 ? (
        <span className="pt-1.5 text-xs text-muted-foreground">読み込み中...</span>
      ) : (
        <div className="flex min-w-0 flex-1 flex-wrap gap-1">
          {options.map((opt) => {
            const active = opt.value === value;
            return (
              <button
                key={opt.value}
                type="button"
                disabled={applying}
                onClick={() => onApply(opt.value)}
                className={cn(
                  "rounded-md px-3 py-1.5 text-xs font-medium whitespace-nowrap ring-1 ring-border transition-colors disabled:pointer-events-none disabled:opacity-50",
                  active
                    ? "bg-primary text-primary-foreground ring-primary"
                    : "bg-transparent text-muted-foreground hover:bg-muted hover:text-foreground"
                )}
              >
                {opt.label}
              </button>
            );
          })}
        </div>
      )}
      {applying && <span className="pt-1.5 text-xs text-muted-foreground">適用中...</span>}
    </div>
  );
}
