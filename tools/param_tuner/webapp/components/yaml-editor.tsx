"use client";

import { useEffect, useState } from "react";
import { yaml } from "@codemirror/lang-yaml";
import { vscodeDark } from "@uiw/codemirror-theme-vscode";
import CodeMirror from "@uiw/react-codemirror";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";

interface Props {
  file: string;
  content: string;
  saving: boolean;
  onSave: (content: string) => void;
  onClose: () => void;
  // Fired on every keystroke so a sibling panel (e.g. the slalom simulator)
  // can react to unsaved edits without owning the draft itself.
  onDraftChange?: (content: string) => void;
}

const EXTENSIONS = [yaml()];

// Mount this only once `content` has actually been fetched (parent shows its
// own loading placeholder until then) - draft is seeded from `content` once,
// at mount time, so there's no prop/state to keep in sync afterwards.
export function YamlEditor({ file, content, saving, onSave, onClose, onDraftChange }: Props) {
  const [draft, setDraft] = useState(content);
  const dirty = draft !== content;

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

  return (
    <Card className="flex flex-1 flex-col overflow-hidden">
      <CardHeader className="flex-row items-center justify-between space-y-0">
        <CardTitle className="truncate">編集: {file}</CardTitle>
        <Button size="sm" variant="outline" onClick={onClose}>
          閉じる
        </Button>
      </CardHeader>
      <CardContent className="flex flex-1 flex-col gap-2 overflow-hidden">
        <div className="min-h-0 flex-1 overflow-auto rounded-md border text-xs">
          <CodeMirror
            value={draft}
            onChange={handleChange}
            theme={vscodeDark}
            extensions={EXTENSIONS}
            basicSetup={{ foldGutter: true, highlightActiveLine: true }}
          />
        </div>
        <div className="flex items-center justify-end gap-2 text-xs text-muted-foreground">
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
