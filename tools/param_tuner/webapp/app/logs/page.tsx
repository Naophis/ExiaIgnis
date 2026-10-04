import { Suspense } from "react";
import { LogDetailView } from "@/components/log-detail-view";

// 詳細ログ解析ページ。パラメータ送信/コンソールとは別画面で、1本のログを
// 画面いっぱいに使って精査する(軌跡 + 連動カーソルの時系列グラフ)。
// ?file=<name.csv> で開くファイルを指定できる(コンソール側のリンクが使う)。
// ?machine=<機体> で、その機体のログ(+ 共通のログ)だけを一覧に出す。無ければ全機体。
export const dynamic = "force-dynamic";

export default async function LogsPage({ searchParams }: PageProps<"/logs">) {
  const params = await searchParams;
  const file = typeof params.file === "string" ? params.file : undefined;
  const machine = typeof params.machine === "string" ? params.machine : undefined;
  return (
    <Suspense>
      <LogDetailView initialFile={file} machine={machine} />
    </Suspense>
  );
}
