@AGENTS.md

---

このファイルは `tools/param_tuner/webapp/`(Param Console)を扱う Claude Code へのガイダンスです。リポジトリ全体については [../../../CLAUDE.md](../../../CLAUDE.md) を参照してください。

## これは何か

`console.sh`(`rx_term.js`)と `update_param.sh`(`tx_term.js` → `send_file.py`)はそれぞれ独自にシリアルポートを exclusive lock で掴むため、**同時に使えなかった**。この Next.js アプリは1本の永続シリアル接続 (`lib/serial-manager.ts` のシングルトン)にRX監視とTX送信を多重化し、ブラウザから両方を同時に扱えるようにしたもの。`console.sh`/`update_param.sh` 自体は CLI フォールバックとして `tools/param_tuner/` に残っている。

起動は `../../../param_console.sh`(リポジトリルート)または `npm run dev`。

## アーキテクチャ概要

- **Next.js 16 App Router**、Turbopack。`serverExternalPackages: ["serialport", "@serialport/bindings-cpp"]` を `next.config.ts` に設定(ネイティブ依存をクライアントバンドルに巻き込まない)。
- **サーバー専用シングルトン**は `globalThis` 経由で dev モードの HMR を生き延びる(Prisma client パターンと同じ)。対象: `lib/serial-manager.ts` の `serialManager`。新しいメソッドを足した直後に古い `next dev` プロセスをそのまま叩くと、`globalThis` にキャッシュされた**古いクラス定義のインスタンス**が使われ「is not a function」エラーになることがある → **サーバー再起動が必要**なケースとして覚えておくこと。
- 全ての `app/api/**/route.ts` は `export const runtime = "nodejs"`(serialport 等の Node ネイティブ依存のため Edge 不可)。
- ファイルパス系のコードは共通して `path.join(process.cwd(), "..")` を `PARAM_TUNER_ROOT` とする(Next サーバーの cwd は `webapp/`)。

## RX(シリアル受信)— `lib/serial-manager.ts`

`rx_term.js` の状態機械をそのまま移植:

- 行モード(`ReadlineParser`, delimiter `\r\n`)がデフォルト。
- `ready___:<byteSize>` → 以降の `name:type:size` 行を `data_struct` に蓄積。
- `start___:<totalBytes>` → `ByteLengthParser` に切り替えてバイナリダンプを1回受信、float/int/short (LE) でパースして `logs/<timestamp>.csv` に保存(+`logs/latest.csv` へコピー)、行モードに戻る。
- `csv___` 〜 `end___` はテキストCSVをそのまま蓄積して保存。
- `map___` 〜 `end___` は迷路データ(カンマ区切り整数)を蓄積し、16x16/32x32 の上三角⇄下三角 swap 変換をして `maze_logs/` に保存。
- `ESC[2J`(画面クリア、`main_task_test_misc.cpp` の `dump1()` 等が送出)を検知すると SSE の `clear` イベントを発火し、ANSI CSI シーケンス自体は表示前に除去する。マーカー判定もこの除去後の文字列に対して行う。
- 切断時は同じ探索ループ(`trySearch`、200ms間隔)が自動的に拾って再接続する。個別の reconnect タイマーは持たない。

## TX(パラメータ送信)— `sendFile`/`sendAll`

`tx_term.js`(実体は `send_file.py::cmd_write` に委譲されていた)のプロトコルを移植: `<remote>@<content>\n` を書き込み → 応答1行を待つ。待機中はテレメトリ/デバッグ行(`ADC0:`/`Gx:`/`Enc0:` を含む行、`[` で始まる行)を読み飛ばす。`OK` で成功、それ以外は失敗(10秒タイムアウト)。バイナリダンプ中や map/csv-text 蓄積中は送信を拒否する(同時破壊防止)。

- base files (`system.yaml`/`hardware.yaml`/`am32.yaml`) は `profile/` 直下、リモート名は `.txt` 拡張子。
- mode files は `profile/hf/`、リモート名は `.hf` 拡張子。`.maze` は中身を `| 0xf0` してswap変換後 `maze.txt` として送信。
- 「全て送信」は mode dir の `*.yaml`(`*.maze` は含まない)→ base files の順。

## AM32 ESC設定の書き込み — `runAm32Command`/`syncAm32`

`am32.yaml` の送信は **`/am32.txt` をデバイスのLittleFSへ置くだけ**で、ESC自体には何も届かない。実際にESCのflashへ書くのはファーム側の `write_am32_param()` で、これは USBコマンド `AM32WRITE`(`src/main/main_task_usb.cpp`)で起動する。`send_file.py` の `am32sync`/`am32write`/`am32read` と同じ流れをアプリ内に持たせたのがこの2メソッド:

- `runAm32Command(kind)`: `AM32WRITE`/`AM32READ` を1行送信 → `OK` ackを待つ → 完了行(`== AM32 write done` / `== AM32 read done`)まで待つ。途中経過のログは通常の `log` イベントとしてコンソールにそのまま流れる。
- `syncAm32(mode)`: `am32.yaml` 送信 → `AM32WRITE`。UIの「ESC書込」ボタン(と編集画面の「保存してESC書込」)がこれ。

注意点:

- **デバイスが起動直後のボタン待ちループにいること**が前提。`rx_usb_cmd()` はフラグを立てるだけで、実行するのは `main_task.cpp` のボタン待ちループのポーリング(`consume_am32_write_request()`)。モード選択に入った後は無視される。
- 完了待ちのタイムアウトは **40秒**。電源制御GPIOが無い構成では `enterConfigMode()` が最大10秒「ESCのバッテリを挿し直せ」とpollingするため、通常のackタイムアウト(10秒)では足りない。
- ファーム側が **完了行を出さずにreturnする経路**(`enterConfigMode failed` / `am32: /am32.txt not found`)を `AM32_FAIL_PREFIXES` で拾い、40秒待たずに即エラーにしている。ファーム側のメッセージを変えたらここも合わせること。
- 「全て送信」は `am32.yaml` も送るが **`AM32WRITE` は撃たない**(ESCのflash書き込み+バッテリ抜き差しを毎回強制するのは重すぎるため)。ESCへ反映したいときは明示的に「ESC書込」を押す。

## system.yaml の編集 — `lib/test-templates.ts`

**YAMLパース+ダンプの往復は禁止。** system.yaml は goals 履歴やAM32移行メモなど150行超のコメントを持つため、パースし直すと全部消える。代わりに**テキスト行レベルの外科的置換**を行う。

- 通常キー(`v_max`/`dist`/`sla_type` 等): ファイル全体を走査し、対象キー名の**コメントアウトされていない最初の行**だけを見つけて値部分だけ置換(インデント・行末コメントは保持)。
- `mode` だけは別方式: `test:` ブロックの外(トップレベル)にあり、`# mode: N # <説明>` の形で ~20個の選択肢がコメントとして並び、1行だけ有効化されている。値を書き換えるのではなく、**現在有効な行をコメントアウトし、目的のmode番号の行のコメントを外す**(`applyModeToggle`)。理由: 値だけ書き換えると `mode: 16 # メイン` のように説明コメントと数値が食い違ってしまうため。
- mode の選択肢ラベルは system.yaml 自身のコメントから動的に読む(`readModeOptions`)。説明に `:` があれば**そこで打ち切る**運用(ボタンが長くなりすぎるのを防ぐため、ユーザーが `wall off: search_mode(1)->...` のように区切りを入れる)。
- `file_idx` の選択肢は `profile/hf/profiles.yaml` の `list` 配列から動的に読む(`readFileIdxOptions`)。
- `sla_type`/`sla_type2`/`sla_return` は `lib/test-template-shared.ts` にハードコードされた `NamedOption[]`(system.yaml のコメントを元にしたチートシート。`sla_type2` は 6/8/9 のみ有効、7=Kojima は死んだパラメータとして除外)。

テンプレートは `profile/test_templates.json` に保存(初回アクセス時に3件シードされる)。クイック適用(`/api/test-templates/quick-apply`)は名前付きテンプレートを介さず直接 `applyTestTemplateToSystemYaml` を呼ぶ一時適用。

## ログプロット — `lib/trajectory.ts` / `components/trajectory-plot.tsx`

廃止した `plot_gui.py`(Tkinter)の軌跡プロット計算をそのまま TypeScript に移植したもの。CSVのソート・`ang_kf_sum`/`ang_kf`+`ideal_ang` からの累積角度復元・タイムスタンプごとの状態グルーピング・45度壁センサーの投影・90mmグリッド線の生成ロジックは元のPython実装と1対1対応させてある(変更する場合は元の `_plot_wall_sensor`/`plot_file` のロジックとの対応を崩さないこと)。描画は Canvas(`components/trajectory-plot.tsx`)、等倍アスペクト比のワールド→キャンバス変換は `makeTransform`。データ点数が数万に及ぶため SVG ではなく Canvas を採用している。

PlotJuggler 連携(`lib/logs.ts`)は `bash -lc "source /opt/ros/jazzy/setup.bash && ros2 run plotjuggler plotjuggler -d <csv> -l <profile.xml>"` を `spawn(..., {detached:true}).unref()` で起動。`profile.xml` は `tools/param_tuner/profile.xml`。

**WALL_OFF 中の高頻度(4kHz相当)サンプル(2026-09-15〜)**: firmware(`src/sensing_task.cpp`)は 1kHz のログ行に「前1msの最大4サンプル」を `hf_d{k}`(距離)/`hf_x{k}`(行取得時の `global_pos.dist` からの進行方向オフセット、過去なので負)/`hf_cnt`/`hf_side`(0=左45, 1=右45, -1=無効) として載せる。`lib/trajectory.ts` の `hfSamplePose()` が行の姿勢を `hf_x{k}` だけ進行方向に内挿し、通常の45度投影 `projectSensorPoint()` に通して `TrajectoryData.hfWallPoints` を作る(壁切れ中は等速直進なので直線内挿、θは行の値)。`lib/log-analysis.ts` の `computeHfEdgeEvents()` は `hf_edge_rel`(検出後の各行で「行位置 − 壁切れ位置」)が最初に正になった行から壁切れ位置(◇ `hf-edge`)と、その位置に最も近いスロットの読みで投影した壁面点(□ `hf-edge-sensor`)を出す。パネルの「hf点群」チェックで点群・マーカーとも表示切替。旧ログ(hf列なし)では何も出ない。

**旋回出口解析(2026-09-23〜)**: `lib/turn-exit.ts` は `tools/param_tuner/turn_exit_check.py` の移植(純粋関数、ブラウザ/Node 共用。仕様の正本は .py の docstring で、数式を変えるときは両方を直す。`turn_exit_check.py` の出力と一致することを 3 本のログで確認済み)。SLALOM 区間ごとに追従状態(wmax/横G/w_lp 過不足/v_c 最小比/内輪速度最小比 `v_in`/duty 飽和 tick 数/終端角度遅れ)と出口残差(旋回後 2tick 目の `kim_theta`、最初に両壁が見えた tick の横ずれ `off0`、25〜50mm 区間の横ずれをヨー分 0.96mm/° で補正して外側正にした `wide`)を出す。旋回種別は角度と「斜め区間にいるか」(走行開始は直線、45°/135° の旋回ごとにトグル)で決め、斜めへ抜ける旋回は 45° センサーが柱を見るので off 系を NaN にする。パネルでは「旋回出口解析」チェックで、選択中のログを**ブラウザ側で**解析して旋回テーブル(行クリックで詳細をクリック情報欄へ。迷路プロットは正方形で横が余るので、プロットの右に横の `ResizablePanelGroup` で置く。既定はプロット 52%、表側は collapsible。旋回テーブルは横幅を抑えるため w_lp 過不足と ey40 を列に出さず行クリックの詳細に回している。下のイベント一覧には出さない)と出口マーカー(◇ `turn-exit`)を出し、あわせて `/api/logs/turn-exit?limit=N` で直近 N 本の集計テーブル((種別, 向き, v) ごとの n / mean±σ)を表示する。集計は有効中、ファイル一覧の先頭(`latest.csv` を除く)が変わるたび=新しいログが保存されるたびに再取得する。API の JSON では NaN が `null` になるので、集計側の表示・色付けは `Stat.n === 0` を先に見る(`fmtStat`/`statFlag`)。

右ペインの「コンソール / プロット」切り替えボタンは右ペイン内ではなく `PortPanel`(ヘッダーバー)の `tabs` スロットに出す(既定ビューのときだけ `app/page.tsx` が渡す)。右ペイン内に置くと1行分の高さを食うため。表示トグル(Left45/Right45/hf点群/原点X)・解析トグル(ドロップ/状態遷移/トラフ/壁切れエッジ/旋回出口/時系列)・PlotJuggler ボタンは**1行**にまとめてある(解析はチップ、有効化した解析だけパラメータ入力が横に展開、ラベルは短縮して説明は `title` ツールチップ)。行を増やす変更はプロット領域を削るので避ける。列の意味は `COLUMN_HELP`(log-plot-panel.tsx)に日本語で持ち、見出し・値セルのどちらにホバーしても `useTip`/`TipLayer`(position: fixed の即時ツールチップ。native の title は1秒待つ上に ScrollArea に切られるので不採用)で表示する。列を足すときは `COLUMN_HELP` にも説明を足す。旋回テーブルの行クリックは `selectedTurnKey`(`log|idx`)で選択し、`TrajectoryPlot` の `highlight` prop に旋回区間(SLALOM 行)と旋回後の窓(出口〜40tick、次の旋回で打ち切り)を生行オブジェクトの集合で渡す。プロット側は他の点を減光し、旋回をマゼンタ・窓をアンバーの太い点で上書きする。同じ行をもう一度クリックで解除。

## Flash — `lib/flash.ts`

リポジトリルートの `flash.sh`(picotool)を `spawn` で実行し、stdout/stderr を1行ずつ `[flash] `プレフィックス付きで `serialManager` の `log` イベントへ流す(コンソールパネルにそのまま表示される)。picotool は USB デバイスを排他的に掴み、書き込み成功後は BOOTSEL から通常ファームウェアへ再起動して CDC デバイスが一旦消える。RX 接続を持ったままだと picotool と掴み合いになるため、実行前に `serialManager.disconnect()`、完了後(成功/失敗どちらでも)に `serialManager.enableAutoConnect()` を呼ぶ。明示的な再接続はせず、既存の200ms探索ループに検出を任せる。

## 既知のハマりどころ

- **flexアイテムの折り返し**: `flex flex-wrap` な子要素がある行コンテナで、子に `min-w-0` を付け忘れると「コンテンツ基準の自動最小幅」によって折り返さずに親をはみ出す。セグメントボタン群(`test-template-panel.tsx` の `QuickApplySelectRow`)で実際に踏んだ。
- **ScrollArea が伸びきってスクロールしない**: shadcn/BaseUI の `ScrollArea` は Root 自体に `overflow` を持たない(スクロールはネストされた Viewport が担当)ため、Root に `min-h-0` を付けないと「コンテンツ基準の自動最小サイズ」でスクロール領域が全コンテンツ分に伸びきり、親の `overflow-hidden` に下側が切られる。
- **Select の controlled/uncontrolled 切り替え警告**: `value` が最初 `undefined`(データ未取得)で後から文字列になると Base UI が警告を出す。値が確定するまで `<Select>` 自体をマウントしない(`components/test-template-panel.tsx` の `QuickApplySelectRow` 参照)。
- **`react-hooks/set-state-in-effect`**: このプロジェクトは React Compiler を使っていないので、`useEffect` 内での fetch-on-mount パターン(react.dev 公式の書き方)に対するこのルールの指摘は多くの場合過検知。個別に `eslint-disable-next-line` で対応している(同じ effect 内の2つ目以降の setState は検知されないことが多いので、まず素で書いてから lint に怒られた行だけ抑制するのが早い)。
- **Turbopack のワークスペースルート誤検出**: `tools/param_tuner/` に複数のロックファイル(pnpm-lock.yaml 等、legacy CLI ツール用)があるため、`next.config.ts` で `turbopack.root` を明示していないと誤ったディレクトリをルートとして警告が出る。
- **開発サーバーの使い回し**: `next dev` は同一プロジェクトで既存サーバーがあると新しいポート指定を無視して既存サーバー(既存ポート)を使う。コード変更を確認する際、`lib/serial-manager.ts` のようなサーバー専用シングルトンを変更した直後は「サーバー再起動」を参照(上記)。

## API 一覧

| エンドポイント | メソッド | 役割 |
|---|---|---|
| `/api/ports` | GET | ttyACM* ポート一覧 |
| `/api/connect` | POST | 自動接続を再有効化 |
| `/api/disconnect` | POST | 切断・自動接続を停止 |
| `/api/status` | GET | 現在の接続状態 |
| `/api/stream` | GET (SSE) | `log`/`status`/`saved`/`clear` イベント配信 |
| `/api/modes` | GET | `profile/` 配下のモード一覧 |
| `/api/profiles` | GET | 指定モードのファイル一覧(base/mode) |
| `/api/profile-file` | GET/POST | YAMLファイルの読み込み/保存 |
| `/api/send` | POST | 個別ファイル送信 / 全送信 |
| `/api/am32` | POST | `sync`(am32.yaml送信+AM32WRITE) / `write` / `read` |
| `/api/test-templates` | GET/POST/DELETE | テンプレート一覧/作成更新/削除 |
| `/api/test-templates/apply` | POST | 保存済みテンプレートをsystem.yamlへ適用 |
| `/api/test-templates/quick-apply` | GET/POST | 現在値取得 / 単一キーの即時適用 |
| `/api/test-templates/file-idx-options` | GET | profiles.yaml由来のfile_idx選択肢 |
| `/api/test-templates/mode-options` | GET | system.yaml由来のmode選択肢 |
| `/api/logs` | GET | ログCSV一覧 |
| `/api/logs/content` | GET | ログCSVの内容 |
| `/api/logs/plotjuggler` | POST | PlotJugglerの起動/終了 |
| `/api/logs/turn-exit` | GET | 複数ログの旋回出口集計(`limit=N` 直近N本 / `names=a.csv,b.csv`)。`lib/turn-exit.ts` をサーバー側で回し (ファイル名, mtime) でキャッシュ |
| `/api/logs/open-folder` | POST | logs/フォルダをファイルマネージャで開く |
| `/api/flash` | POST | `flash.sh`(picotool)実行。実行前にシリアル切断、完了後auto-connectを再有効化 |
