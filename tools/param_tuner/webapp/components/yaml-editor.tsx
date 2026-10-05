"use client";

import { useEffect, useRef, useState, type ReactNode } from "react";
import { yaml } from "@codemirror/lang-yaml";
import { EditorView } from "@codemirror/view";
import CodeMirror from "@uiw/react-codemirror";
import { MachineChip } from "@/components/machine-chip";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { cbEditorTheme } from "@/lib/cm-cb-theme";
import { machineDiffExtension, setLineMarks, type LineMark } from "@/lib/cm-machine-diff";
import type { Machine } from "@/lib/machine-shared";

interface Props {
  file: string;
  content: string;
  // Seeds the draft buffer instead of `content` when set (e.g. a sibling
  // panel patched the draft externally). Parent must bump the `key` it
  // mounts this component with whenever this changes, since it's only read
  // at mount time - see initialDraft below.
  initialDraft?: string;
  saving: boolean;
  onSave: (content: string) => void;
  onClose: () => void;
  // Fired on every keystroke so a sibling panel (e.g. the slalom simulator)
  // can react to unsaved edits without owning the draft itself.
  onDraftChange?: (content: string) => void;
  // File-specific actions rendered next to 閉じる (e.g. am32.yaml's
  // "保存してESC書込"). The parent owns them since they usually combine a save
  // with something outside the editor's scope.
  headerActions?: ReactNode;
  // どの機体のファイルか(見出しにその機体の色で出す)
  machine?: Machine | null;
  // 見出しの下に出す帯(「表示中の機体と違う」「ほかの機体にも入れる」など)
  banner?: ReactNode;
  // ほかの機体と値が違う行(lib/use-machine-diff.ts)
  marks?: LineMark[];
  // 印の内訳(見出しに出す)
  markSummary?: ReactNode;
}

const EXTENSIONS = [yaml(), ...machineDiffExtension];

// Mount this only once `content` has actually been fetched (parent shows its
// own loading placeholder until then) - draft is seeded from `content` (or
// `initialDraft`, if provided) once, at mount time, so there's no prop/state
// to keep in sync afterwards.
export function YamlEditor({
  file,
  content,
  initialDraft,
  saving,
  onSave,
  onClose,
  onDraftChange,
  headerActions,
  machine,
  banner,
  marks,
  markSummary,
}: Props) {
  const [draft, setDraft] = useState(initialDraft ?? content);
  const dirty = draft !== content;
  const viewRef = useRef<EditorView | null>(null);
  const [viewReady, setViewReady] = useState(false);

  const handleChange = (value: string) => {
    setDraft(value);
    onDraftChange?.(value);
  };

  useEffect(() => {
    const handler = (e: KeyboardEvent) => {
      if (!(e.ctrlKey || e.metaKey) || e.key.toLowerCase() !== "s") return;
      e.preventDefault();
      if (dirty && !saving) onSave(draft);
    };
    window.addEventListener("keydown", handler);
    return () => window.removeEventListener("keydown", handler);
  }, [draft, dirty, saving, onSave]);

  useEffect(() => {
    if (!viewReady || !viewRef.current) return;
    viewRef.current.dispatch({ effects: setLineMarks.of(marks ?? []) });
  }, [marks, viewReady]);

  // カーソルより後ろの最初の印へ(無ければ先頭の印へ戻る)
  const jumpToNextMark = () => {
    const view = viewRef.current;
    if (!view || !marks || marks.length === 0) return;
    const curLine = view.state.doc.lineAt(view.state.selection.main.head).number;
    const next = marks.find((m) => m.line > curLine) ?? marks[0];
    const line = view.state.doc.line(Math.min(next.line, view.state.doc.lines));
    view.dispatch({ selection: { anchor: line.from }, effects: EditorView.scrollIntoView(line.from, { y: "center" }) });
    view.focus();
  };

  return (
    <Card
      className="flex h-full min-w-0 flex-1 flex-col overflow-hidden"
      style={machine ? { boxShadow: `inset 0 3px 0 ${machine.color}` } : undefined}
      data-yaml-editor
    >
      {/* 見出しと操作を 1 行に(CardHeader は grid なので flex に替える。行を増やすと編集領域が減る) */}
      <CardHeader className="flex flex-row items-center justify-between gap-2 space-y-0">
        <CardTitle className="flex min-w-0 items-center gap-1.5">
          <span className="shrink-0">編集:</span>
          {machine && <MachineChip machine={machine} title={`machines/${machine.id}/profile のファイル`} />}
          <span className="truncate">{file}</span>
        </CardTitle>
        <div className="flex shrink-0 items-center gap-2">
          {markSummary}
          {marks && marks.length > 0 && (
            <Button size="sm" variant="ghost" title="ほかの機体と値が違う次の行へ" onClick={jumpToNextMark}>
              次の差 ↓
            </Button>
          )}
          {headerActions}
          <Button size="sm" variant="outline" onClick={onClose}>
            閉じる
          </Button>
        </div>
      </CardHeader>
      <CardContent className="flex flex-1 flex-col gap-1.5 overflow-hidden">
        {banner}
        <div className="min-h-0 flex-1 overflow-auto rounded-md border text-xs">
          <CodeMirror
            value={draft}
            onChange={handleChange}
            onCreateEditor={(view) => {
              viewRef.current = view;
              setViewReady(true);
            }}
            theme={cbEditorTheme}
            extensions={EXTENSIONS}
            basicSetup={{ foldGutter: true, highlightActiveLine: true }}
          />
        </div>
        <div className="flex items-center justify-end gap-1.5 text-xs text-muted-foreground">
          {saving ? (
            <span>保存中...</span>
          ) : dirty ? (
            <span>未保存の変更があります (Ctrl+S で保存)</span>
          ) : (
            <span>Ctrl+S で保存</span>
          )}
        </div>
      </CardContent>
    </Card>
  );
}
