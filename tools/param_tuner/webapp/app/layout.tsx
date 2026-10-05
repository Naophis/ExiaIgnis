import type { Metadata } from "next";
import { Geist, Geist_Mono, Michroma } from "next/font/google";
import { ThemeSync } from "@/components/theme-sync";
import { Toaster } from "@/components/ui/sonner";
import { THEME_INIT_SCRIPT } from "@/lib/theme";
import "./globals.css";

const geistSans = Geist({
  variable: "--font-geist-sans",
  subsets: ["latin"],
});

const geistMono = Geist_Mono({
  variable: "--font-geist-mono",
  subsets: ["latin"],
});

// 見出し用(ヘッダーの銘など、英字だけ)。作中の表示に多い Eurostile 系の横に広い書体。
// 日本語の文字は持たないので、本文には使わない。
const michroma = Michroma({
  variable: "--font-michroma",
  weight: "400",
  subsets: ["latin"],
});

export const metadata: Metadata = {
  title: "Exia Console",
  description: "Pico param tuner: serial console + parameter sender",
};

export default function RootLayout({ children }: LayoutProps<"/">) {
  return (
    // suppressHydrationWarning: 下の描画前スクリプトが <html> の style(テーマカラー)を
    // 書き換えるので、サーバーの HTML と食い違う。
    <html
      lang="ja"
      className={`${geistSans.variable} ${geistMono.variable} ${michroma.variable} dark h-full antialiased`}
      suppressHydrationWarning
    >
      <head>
        {/* 覚えてあるテーマカラーを、最初の描画より前に当てる(lib/theme.ts)。 */}
        <script dangerouslySetInnerHTML={{ __html: THEME_INIT_SCRIPT }} />
      </head>
      <body className="min-h-full flex flex-col">
        <ThemeSync />
        {children}
        <Toaster theme="dark" />
      </body>
    </html>
  );
}
