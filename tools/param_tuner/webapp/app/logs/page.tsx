import { Suspense } from "react";
import { LogDetailView } from "@/components/log-detail-view";

// 詳細ログ解析ページ。パラメータ送信/コンソールとは別画面で、1本のログを
// 画面いっぱいに使って精査する(軌跡 + 連動カーソルの時系列グラフ)。
// ?file=<name.csv> で開くファイルを指定できる(コンソール側のリンクが使う)。
export const dynamic = "force-dynamic";

export default async function LogsPage({ searchParams }: PageProps<"/logs">) {
  const params = await searchParams;
  const file = typeof params.file === "string" ? params.file : undefined;
  return (
    <Suspense>
      <LogDetailView initialFile={file} />
    </Suspense>
  );
}
