import { tags as t } from "@lezer/highlight";
import { createTheme } from "@uiw/codemirror-themes";

// yaml の編集画面の配色。ほかのパネルと同じ濃紺の地に合わせ、選択・カーソル・今の行は
// 主色(テーマカラー)に付いて変わる(色は CSS 変数で渡す)。文字の色は意味を持つので、
// 主色を変えても変わらない固定の色にしてある:
//   キー = 空色、値 = 白に近い薄緑(GN 粒子)、コメント = 沈んだ青灰、括弧・引用 = 金
// lang-yaml が付けるタグは @lezer/yaml の highlight.js を参照(数値と文字列の区別は無く、
// 引用符の無い値はすべて content)。
export const cbEditorTheme = createTheme({
  theme: "dark",
  settings: {
    background: "var(--cb-editor-bg)",
    foreground: "#dff7ee",
    caret: "var(--primary-bright)",
    selection: "color-mix(in oklch, var(--primary-bright) 32%, transparent)",
    selectionMatch: "color-mix(in oklch, var(--primary-bright) 20%, transparent)",
    lineHighlight: "color-mix(in oklch, var(--primary-bright) 9%, transparent)",
    gutterBackground: "var(--cb-editor-bg)",
    gutterForeground: "#56698a",
    gutterActiveForeground: "#cfe3ff",
    gutterBorder: "transparent",
    fontFamily: 'var(--font-geist-mono), Menlo, Monaco, Consolas, "Ubuntu Mono", monospace',
  },
  styles: [
    { tag: [t.definition(t.propertyName), t.propertyName], color: "#8ec9ff" },
    { tag: [t.content], color: "#dff7ee" },
    { tag: [t.string, t.special(t.string), t.attributeValue], color: "#f0d58c" },
    { tag: [t.lineComment, t.comment], color: "#6480a6" },
    { tag: [t.separator, t.punctuation], color: "#7391b8" },
    { tag: [t.squareBracket, t.brace], color: "#e6c36a" },
    { tag: [t.keyword, t.meta], color: "#c3a6ff" },
    { tag: [t.labelName, t.typeName], color: "#7fe7c4" },
  ],
});
