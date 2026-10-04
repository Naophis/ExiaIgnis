"use client";

import { useEffect, useLayoutEffect } from "react";
import { THEME_APPLIED_KEY, THEME_EVENT, applyStoredTheme } from "@/lib/theme";

// テーマカラーを <html> に当て続ける(全ページ共通、app/layout.tsx に置く)。
//   - 開発モードでは React が <html> の属性を JSX のものへ戻すので、描画前スクリプトが
//     付けた style が消える。ここで当て直す(本番では何も変わらない)。
//   - 別のタブで色を変えたとき(storage イベント)と、同じページで変えたとき
//     (THEME_EVENT)に当て直す。
export function ThemeSync() {
  useLayoutEffect(() => {
    applyStoredTheme();
  }, []);

  useEffect(() => {
    const onStorage = (e: StorageEvent) => {
      if (e.key === null || e.key === THEME_APPLIED_KEY) applyStoredTheme();
    };
    window.addEventListener("storage", onStorage);
    window.addEventListener(THEME_EVENT, applyStoredTheme);
    return () => {
      window.removeEventListener("storage", onStorage);
      window.removeEventListener(THEME_EVENT, applyStoredTheme);
    };
  }, []);

  return null;
}
