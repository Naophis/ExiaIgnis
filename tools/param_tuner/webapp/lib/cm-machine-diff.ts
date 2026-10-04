import { RangeSetBuilder, StateEffect, StateField } from "@codemirror/state";
import { Decoration, EditorView, hoverTooltip, type DecorationSet } from "@codemirror/view";

// 編集画面で「ほかの機体と値が違う行」に印を付ける CodeMirror の拡張。
//   todo     = 未整理(違うのに「固有」に登録していない)。左に琥珀色の帯
//   specific = 固有(機体ごとに違ってよい)。左に紫の帯
// 行にマウスを載せると、ほかの機体の値を出す。

export interface LineMark {
  line: number; // 1 始まり
  kind: "todo" | "specific";
  tips: string[]; // ほかの機体の値(1 行ずつ)
}

export const setLineMarks = StateEffect.define<LineMark[]>();

interface MarkState {
  deco: DecorationSet;
  byLine: Map<number, LineMark>;
}

const todoLine = Decoration.line({ class: "cm-machine-todo" });
const specificLine = Decoration.line({ class: "cm-machine-specific" });

const markField = StateField.define<MarkState>({
  create: () => ({ deco: Decoration.none, byLine: new Map() }),
  update(value, tr) {
    for (const e of tr.effects) {
      if (!e.is(setLineMarks)) continue;
      const builder = new RangeSetBuilder<Decoration>();
      const byLine = new Map<number, LineMark>();
      for (const m of [...e.value].sort((a, b) => a.line - b.line)) {
        if (m.line < 1 || m.line > tr.state.doc.lines || byLine.has(m.line)) continue;
        byLine.set(m.line, m);
        const from = tr.state.doc.line(m.line).from;
        builder.add(from, from, m.kind === "todo" ? todoLine : specificLine);
      }
      return { deco: builder.finish(), byLine };
    }
    // 入力中は印を文字の移動に合わせて動かす(行番号の表は、次に印を計算し直すまで古いまま)
    if (tr.docChanged) return { deco: value.deco.map(tr.changes), byLine: value.byLine };
    return value;
  },
  provide: (f) => EditorView.decorations.from(f, (v) => v.deco),
});

const markTooltip = hoverTooltip(
  (view, pos) => {
    const line = view.state.doc.lineAt(pos);
    const mark = view.state.field(markField).byLine.get(line.number);
    if (!mark) return null;
    return {
      pos: line.from,
      end: line.to,
      above: true,
      create() {
        const dom = document.createElement("div");
        dom.className = "cm-machine-tip";
        const head = document.createElement("div");
        head.className = "cm-machine-tip-head";
        head.textContent =
          mark.kind === "todo" ? "ほかの機体と違う(未整理)" : "ほかの機体と違う(固有 = 機体ごとの値)";
        dom.appendChild(head);
        for (const t of mark.tips) {
          const row = document.createElement("div");
          row.textContent = t;
          dom.appendChild(row);
        }
        return { dom };
      },
    };
  },
  { hoverTime: 120 },
);

const markTheme = EditorView.baseTheme({
  ".cm-machine-todo": {
    boxShadow: "inset 3px 0 0 #f59e0b",
    backgroundColor: "rgba(245, 158, 11, 0.10)",
  },
  ".cm-machine-specific": {
    boxShadow: "inset 3px 0 0 #a78bfa",
    backgroundColor: "rgba(167, 139, 250, 0.07)",
  },
  ".cm-machine-tip": {
    padding: "4px 8px",
    fontFamily: "var(--font-geist-mono), monospace",
    fontSize: "12px",
    lineHeight: "1.5",
    maxWidth: "60ch",
    whiteSpace: "pre-wrap",
  },
  ".cm-machine-tip-head": {
    fontFamily: "var(--font-sans), sans-serif",
    opacity: "0.7",
  },
});

export const machineDiffExtension = [markField, markTooltip, markTheme];
