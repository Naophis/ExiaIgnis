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

**解析まわりの置き場所(2026-09-23 に共通化)**: 重畳解析の設定と計算は `lib/use-analysis.ts`(`useAnalysisSettings()` / `useAnalysisEvents()`)、チップ UI は `components/analysis-toggles.tsx`、旋回テーブルは `components/turn-exit-table.tsx` にあり、**プロットタブと詳細ログ解析ページ(/logs)が同じものを使う**。解析を足すときは lib 側に設定と計算、toggles 側にチップを足せば両画面に出る。片方だけに実装しないこと。

**旋回出口解析(2026-09-23〜)**: `lib/turn-exit.ts` は `tools/param_tuner/turn_exit_check.py` の移植(純粋関数、ブラウザ/Node 共用。仕様の正本は .py の docstring で、数式を変えるときは両方を直す。`turn_exit_check.py` の出力と一致することを 3 本のログで確認済み)。SLALOM 区間ごとに追従状態(wmax/横G/w_lp 過不足/v_c 最小比/内輪速度最小比 `v_in`/duty 飽和 tick 数/終端角度遅れ)と出口残差(旋回後 2tick 目の `kim_theta`、最初に両壁が見えた tick の横ずれ `off0`、25〜50mm 区間の横ずれをヨー分 0.96mm/° で補正して外側正にした `wide`)を出す。旋回種別は角度と「斜め区間にいるか」(走行開始は直線、45°/135° の旋回ごとにトグル)で決め、斜めへ抜ける旋回は 45° センサーが柱を見るので off 系を NaN にする。パネルでは「旋回出口」チェックで、選択中のログを**ブラウザ側で**解析して旋回テーブル(行クリックで詳細をクリック情報欄へ。迷路プロットは正方形で横が余るので、プロットの右に横の `ResizablePanelGroup` で置く。既定はプロット 52%、表側は collapsible。旋回テーブルは横幅を抑えるため w_lp 過不足と ey40 を列に出さず行クリックの詳細に回している。下のイベント一覧には出さない)と出口マーカー(◇ `turn-exit`)を出し、あわせて `/api/logs/turn-exit?limit=N` で直近 N 本の集計テーブル((種別, 向き, v) ごとの n / mean±σ)を表示する。集計は有効中、ファイル一覧の先頭(`latest.csv` を除く)が変わるたび=新しいログが保存されるたびに再取得する。API の JSON では NaN が `null` になるので、集計側の表示・色付けは `Stat.n === 0` を先に見る(`fmtStat`/`statFlag`)。

右ペインの「コンソール / プロット」切り替えボタンは右ペイン内ではなく `PortPanel`(ヘッダーバー)の `tabs` スロットに出す(既定ビューのときだけ `app/page.tsx` が渡す)。右ペイン内に置くと1行分の高さを食うため。表示トグル(Left45/Right45/hf点群/原点X)・解析トグル(ドロップ/状態遷移/トラフ/壁切れエッジ/旋回出口/時系列)・PlotJuggler ボタンは**1行**にまとめてある(解析はチップ、有効化した解析だけパラメータ入力が横に展開、ラベルは短縮して説明は `title` ツールチップ)。行を増やす変更はプロット領域を削るので避ける。列の意味は `COLUMN_HELP`(log-plot-panel.tsx)に日本語で持ち、見出し・値セルのどちらにホバーしても `useTip`/`TipLayer`(position: fixed の即時ツールチップ。native の title は1秒待つ上に ScrollArea に切られるので不採用)で表示する。列を足すときは `COLUMN_HELP` にも説明を足す。旋回テーブルの行クリックは `selectedTurnKey`(`log|idx`)で選択し、`TrajectoryPlot` の `highlight` prop に旋回区間(SLALOM 行)と旋回後の窓(出口〜40tick、次の旋回で打ち切り)を生行オブジェクトの集合で渡す。プロット側は他の点を減光し、旋回をマゼンタ・窓をアンバーの太い点で上書きする。同じ行をもう一度クリックで解除。

## 詳細ログ解析ページ — `/logs`(`app/logs/page.tsx` / `components/log-detail-view.tsx`)

1本のログを画面いっぱいで精査する別ページ。`?file=<name>.csv` で開くファイルを指定でき、プロットタブの「詳細解析」ボタンがこれを使う。**プロットタブ側は残してある**(走った直後にその場で見る用途はそちらが速い)ので、機能を重複して増やさないこと。任意列の時系列は PlotJuggler が担当なので、このページはそこと張り合わない(旋回を軸にした精査が役割)。

- **左: 軌跡、右: 時系列グラフの縦積み**。全グラフが x 軸(`Domain`)と連動カーソル(CSV の `index`)を共有し、ホイール=ズーム/ドラッグ=パンがすべてのグラフに効く。軌跡には現在のカーソル位置に白いリングが出る。軌跡の点をクリックするとカーソルがそこへ移る。
- **旋回ストリップ**: `analyzeTurnExits()` の結果を並べたボタン。押すと x 軸をその旋回(前 40tick 〜 出口 +90tick)へズームし、軌跡はその旋回をマゼンタ・旋回後の窓をアンバーで強調する。ホバーで wide / yaw0 / sat が出る。
- **解析トグル**はプロットタブと同じ `AnalysisToggles`(ドロップ/状態遷移/トラフ/壁切れエッジ/旋回出口)。マーカーは軌跡と各グラフの両方へ出る。旋回出口だけこのページでは既定 ON(旋回ストリップが主な移動手段のため)。「旋回表」で旋回テーブル(行クリックでその旋回へズーム)、「イベント一覧」で検出ラベルの一覧を出す。複数ログの集計はこのページでは出さない(1本を精査する画面なので、`AnalysisToggles` の `showTurnExitSummary` を渡さない)。
- **観点(グラフ構成のプリセット)**は `lib/log-columns.ts` の `CHART_VIEWS`。**PlotJuggler のレイアウト `tools/param_tuner/profile.xml` のタブとプロットをそのまま写したもの**(旋回/概観/制御/FFトルク/位置/Kanayama/横センサー/前センサー/壁切れ/壁切れhf/IMU加速度/エンコーダ/計算時間/その他の14種)。`profile.xml` を編集したらこちらも合わせること。xy プロットと hf のスロット配列は軌跡プロット側の担当なので写していない。
- 列指定は `name` か `name*係数`(PlotJuggler の Scale 変換と同じ。`parseColumnSpec`/`columnSpecLabel`)。桁の違う列を同じグラフに重ねるため(`alpha*0.01`、`motion_state*100` など)。
- 列チップのクリックで外し、「列 +」で 150 列超から絞り込んで足せる。↑↓ で並び替え、下端のバーをドラッグで高さを変える。編集は**観点ごと**に覚えるので、観点を切り替えて戻っても編集が残る(見出しに「編集済み」と出る)。「この観点を初期状態に戻す」でその観点だけプリセットへ戻す。
- **表示設定の保存**: 観点ごとのグラフ構成(`chartsByView`: 列・高さ・並び順)、選択中の観点、表示トグルを `localStorage`(`exia-log-detail-prefs-v1`)にまとめて保存する。左右の分割幅は `ResizablePanelGroup` の `autoSaveId` が別に持つ。読めなくても既定値で動く。プリセットのグラフ id は `観点キー#連番`(ランダムにすると React の key と系列キャッシュが毎描画で変わる)。
- **状態帯**: `motion_state` の区間をグラフ背景に敷く(`MOTION_STATE_BAND`)。旋回と前後の繋ぎだけ色を付け、直進・停止は透明にして帯だらけにしない。番号は `include/enums.hpp` の `MotionType` と一致させること。
- **カーソル読み取り行**(最下部)は `CURSOR_READOUT_COLUMNS` の列を出す。

`SensorTimeseriesPlot` はこのページのために拡張済み: `domain`/`onDomainChange` を渡すと controlled(複数グラフで x 軸共有)、`cursorIndex`/`onCursorChange` で連動カーソル、`bands` で背景帯、`compact` で下部の読み取り行と凡例を省く(見出しの列チップが同じ色で凡例を兼ねる)。カーソル位置の値は各グラフの**右上に固定**で出す(位置が動くと読みにくい)。全グラフが同じ index を指しているので、1枚にマウスを載せれば全グラフの値が同時に読める。カーソルと読み取り枠は**系列とは別のキャンバス**に描くので、カーソルが動いても重い系列の再描画は起きない。controlled のときは「データが変わったら全体にフィットし直す」処理を親に任せる(`domain` を `null` にして `fullDomain` を渡す)。

## センサ校正タブ — `components/sensor-calib-panel.tsx` / `lib/sensor-calib*.ts`

旧 `csv/sensor.sh`(位置ごとに Ctrl-C)→ `merge.sh` → `pyplot.py` → sensor.yaml へ手で書き写し、の置き換え。機体はテストモード28(校正用。15 や 14 でも静止点は記録できる)。9列生値を SSE の `log` から拾う(パネル自身が `/api/stream` を購読)。

- 換算式はファームの `calc_sensor_val` と同じ `dist = a/ln(raw) − b`。u=1/ln(raw) で線形なので閉形式の最小二乗(`fitGain`)。`pyplot.py` の `curve_fit` と小数4桁まで一致を確認済み。「bのみ」は現在の a を固定して b だけ合わせる(`dump1()` の `adjust_b_to_target45/90` と同じ用途)。
- 置き方は s(横=左右の壁を同時)/l/r/f。旧 csv の `l_*.csv` と `r_*.csv` は中身が同一=左右同時測定だったので、s の1行を L45 系と R45 系の両方に使う。CSV保存は `csv/calib_<日時>/` に旧形式で書き、s は `l_`/`r_` の両方へ出す。読込はサブフォルダも含め、同じ置き方で中身が同一のファイルは1回だけ(`csv/1st` と `csv/1st/far` の重複対策)、l/r で同一の組は s にまとめる。
- 90度センサーは測定範囲が広く 1 組の a, b では表しきれないので near/mid/far に分ける。キーごとの距離範囲(既定 near/mid 42〜96、far 84〜138 = 旧手順の運用)でフィットし、係数表の「範囲」欄で変えられる。「L=R」で L90/R90 の同じ組を連動、「既定」で戻す。既定値を変えたら `RANGES_VERSION` を上げる(localStorage の古い範囲を捨てるため)。ファームは 3 組とも常に計算し、どれを使うかは使う側のコードが決める(距離で自動切り替えはしない)。L90/R90 を選ぶとプロットに同じセンサーの 3 本の曲線と範囲の帯(選択中は全幅、他は右端)と凡例を重ねる。選択中キーの範囲外の行の残差は薄く出す(外挿)。
- csv 読込は「読込」(置き換え)と「+追加」、下位フォルダは「下位」チェックで(既定オフ)。現行 sensor.yaml の a は「csv/ 直下を読込 → 2nd を追加」で 12 キーとも一致する(near/mid = 直下の 42〜96、far = 直下の 84〜96 + 2nd の 126〜138)。b は後から dump1 で合わせ直しているので一致しない。
- yaml 反映は sensor.yaml の `gain:` ブロック内の `KEY: [a, b]` 行だけを置換(コメント保持、parse→dump しない)。mode15 中はファイルを受け付けないので、「保存+送信」はボタン待ちループで。
- **テストモード28(`MainTask::test_front_sensor_sweep()`)= 校正用のモード**。横壁は置いて記録、**前壁は機体が自分で走って測る(スイープ)のが既定**(位置表の既定は横壁 4 位置 + 前壁スイープ 1 行。前壁 9 位置の置き直しは「前壁の取り方」で選ぶ従来手順)。待機中は 9 列の生値を 10Hz で出し、yaml も受信する(保存+送信が通る)。
  - **走行は必ずケーブルなし**(つないだままの走行は危険、とユーザーから強く指摘された)。人がするのは「ケーブルを抜く → 機体のボタン → スタート位置に置いて前に手をかざす → (機体が走って止まる) → ケーブルをつなぐ」だけ。ファームはケーブルがつながっている間は開始手順へ入らず、ログは走ったあと最初につながって 1 秒後に自動で送る(3 秒以上抜いて挿し直すと送り直す。UI は中身が同じスイープを二重に入れない)。ケーブルなしでは画面が見えないので、走れない理由(前壁が近い)は error 音で知らせる。
  - 進行状況の行: `sweep: ready (v= dist= d0= ctrl=)` / `unplug the cable` / `place` / `front wall too close` / `wave a hand` / `running` / `sending log (d0=)` / `dumped` / `cancelled`。UI の案内(`parseSweepStateLine`、`FW_STATE_TTL`)が読む。**ファームの文言を変えたら UI も合わせる。** 走行中はつながっていないので、実際に届くのは ready / unplug / sending / dumped。案内は `connected` prop(page.tsx の接続状態)と組み合わせて出す。
  - **壁制御は走行距離で決め打ち**(ユーザー指示): 走り出しから `sensor_sweep_wall_ctrl_dist`(100mm)だけ `sct=Straight`、残り 95mm は `sct=NONE`(前壁へ近づくと right45_d 等が前壁を見るため)。cells=2 なら前壁まで 137mm の所で切れる。前センサーの読みで切る既存の判定(`exist.front`)は校正中の係数に依るので使わない。`go_straight` を 2 本つなぐのでログの `dist` は 2 本目の頭で 0 に戻り、`parseSweepLog` が速度 × 時間で補ってつなぎ直す(実機ログを分割したもので誤差 0.01mm)。
- **前壁スイープの距離の基準は迷路の寸法(2026-09-28、ユーザー案)**: 探索の走り出しと同じ `offset_start_dist_search`(15) + 90 × `sensor_sweep_cells`(2) = 195mm 走り、区画中央 = 前壁まで 42mm で止まる。スタート区画を含めて 3 区画の直線の突き当たりに前壁。開始位置の前壁距離 d0 = 195 + 42 = 237 をファームが知らせ、UI はスイープ行の `dist` に入れる(前壁距離 = d0 − 走行距離)。人が測る値も静止点も要らない。測りたい範囲(42〜138mm)に入る前に 1 区画ぶん壁制御で姿勢が整う。
  - 実機 2 本(`20260928_222909/222947`、このときは 85mm 走行・開始位置 147 と仮定)で、同じ生値の読みの差 0.2〜0.7mm、a の差 1% 以内(置き直しを含む再現性)。
  - **移動距離の精度**はそのまま距離の誤差になる(タイヤ径の誤差率 × 走行距離。0.5% なら 42mm 地点で約 1mm)。再現性には出ない系統誤差なので、止まった位置が実際に前壁から 42mm かをスペーサーで一度確かめる。前壁の静止点を記録してあれば、スイープ行のホバーに「静止点との差」を位置ごとに出す(近いほど大きければ走行距離、一定なら開始位置)。
  - 「静止点に合わせる」(詳細、既定オフ)は、開始位置を静止点から L90/R90 別々に求め直す(`estimateSweepD0`)。既存の静止点 csv は置き損じが数 mm ある(`f_48` と `f_54` の生値がほぼ同じ、far の 3 点は near と約 10mm 食い違う)ので、既定にしていない。
  - **速度・加減速・吸引は直進テスト(`test_run`)と同じ** system.yaml の `test.v_max` / `accl` / `decel` / `suction_active`(ユーザー指示)。吸引時の hold → 吸引 → hold_settle_wait → unhold、加減速テーブル、最後の 5mm を `end_v` で詰める止まり方も `test_run` と同じ。違いは走行距離と、壁制御を途中で切ることだけ。go_straight は 3 本(壁制御あり 100mm / なし / 最後の 5mm)。値は走る直前に読むので、テンプレートを送り直せば次のスイープから変わる。校正向きなのは遅い設定(400mm/s で 1 点 0.4mm、1500mm/s だと 1 点 1.5mm)。
  - **LED の上書き**: `sensing_task.cpp` は探索モード(`test.search_mode: 1`)で `sct=NONE` の直進だと LED を全部消す。壁制御を切った区間 = 測る範囲なので、そのままだと生値が 0 になり校正できない(実機 `20260928_233335.csv` で発生。195mm は正しく走ったが 2 本目以降の生値がすべて 0)。スイープ中だけ `tgt_val->sensing_force_led = true` にして全 LED を点ける。
  - 届いたログが使えないとき(`looksLikeSweep` が false)、校正モードの機体から来たものなら理由(`SweepLog.reject`)をトーストで出す。黙って無視すると「走らせたのに何も起きない」になる。
  - 開始手順は通常走行と同じ(`reset_gyro_ref_with_check`)。ケーブルなしでモードへ入ったときは待機を飛ばしてそのまま開始手順へ進む(起動のボタン → 置く → 手をかざす)。つないで入ったときと 1 本走ったあとは待機し、次は機体のボタンで始める。
  - `sensor_sweep_guard: 1` で、前壁が近く見えるあいだは走らない。しきい値は「走行距離 − 15」と「`sensor_range_max`(180) − 10」の小さい方(= 170mm)。**距離の読みは `sensor_range_max` で頭打ち**なので、しきい値を 180 以上にすると永久に通らない(2026-09-28 に実際に踏み、スタート位置で何も起きなかった)。走れない理由は 2 秒ごとに音で知らせる: 低い音 4 回 = 前壁が近い、短い音 1 回 = ケーブル接続中と判定。
  - **前壁を手で置く手順は通常表示に出さない**(ユーザー: 手で動かすのは NG)。位置の追加(+ 位置)と「前壁を手置き 9 点にする」は「詳細」の中だけ。位置表の下は「全部やり直す」だけ。スイープ行は未走行なら `offsets: []` の空の行で、取り込むとそこへ入る。
  - 取り込み: 最初の STRAIGHT(1) か BACK_STRAIGHT(5) 区間、`offset = −dist`。保存イベントでの自動取り込みは `looksLikeSweep`(1 方向・30〜400mm・150 点以上・向きの変化 5° 以下・L90/R90 とも生値 200 以上・ADC 範囲内・両端の生値の比 3 倍以上)だけ。過去ログ 1927 本で拾うのは実機のスイープ 2 本だけ。
  - 姿勢の診断(`SweepPose`)はジャイロの向きの変化と横ずれだけ。45 度センサーの読みの傾きから「横壁に対する傾き」を出す案は、読みが向きでも約 1mm/度動くので当てにならず、やめた。
  - スイープが通っている範囲は形をスイープの点だけで決める(`sweepOnly`)。静止点も混ぜるなら「静止点も」。
- **操作の流れ(2026-09-28 に作り直し)**: ツールバー 1 行目は「受信中/未受信」「案内(いま何をするか、`data-calib-guide`)」「● 記録 (Space)」「保存」「保存+送信」「詳細」だけ。csv 読込・CSV保存・N・D0・スイープ・a,b/bのみ は「詳細」で開く 2 行目。記録先は `targetRow` = 選択行、無ければ未記録の最初の位置(行を選ばないと記録できない作りは、理由が画面に出ず「使えない」と言われた)。全位置を取り終えたら選択を外す(Space で最後の行を上書きしない)。Space は数値欄・select・textarea 以外ならどこにフォーカスがあっても記録にする(capture で拾い、フォーカスを外す。タブのボタンを押した直後は Space がそのボタンに取られていた)。Space を横取りするので、タブ表示中だけマウントする。
- 生値はテストモード28/15(9 列)とモード14(`sensor:` 行、F は (L90+R90)/2 で補う)を受ける。モード28 と 14 は実行中も yaml を受信するので「保存+送信」がそのまま通る。モード15 の実行中は機体が受信しないので、送信失敗時は「yaml は保存済み、機体を再起動して送信」と出す。
- 表の「採用」チェックは「この位置のデータを計算に使うか」で、壁の有無ではない(見出しが「使」「壁」だったとき壁の状態と誤解された)。
- 位置表は `localStorage`(`exia-sensor-calib-v2`)に保存。読み込み済みかは ref でなく state(`hydrated`)で持つ: 開発モードは effect を 2 回走らせるので、ref だと 1 回目の保存が既定の表で上書きし、再読み込みで記録が消えていた。既定の表を変えたら `ROWS_VERSION` を上げ、保存済みの表は `migrateRows` で直す。

## 迷路タブ — `components/maze-panel.tsx` / `lib/maze*.ts`

`sample/mm_maze_viewer`(VSCode 拡張)の移植(2026-09-28)。形式の扱いは `lib/maze-shared.ts`(ブラウザ/Node 共用)、ファイル操作は `lib/maze.ts`。

- **並び**: `.maze`・`maze_logs/`・`maze_data/*.yaml` の `wall` はどれも `idx = x * size + y`(テキストは 1 行 = 1 列 x)、下位 4bit が壁(N=1/E=2/W=4/S=8)。ファームの `map[x + y * size]` とは転置の関係で、受信(`map___`)と送信(`sendMaze`)で `swapMazeTriangle` が入れ替える。上位 4bit(踏破フラグ)は読込時に捨て、送信時に全マス `| 0xf0`(踏破済み)にする。
- **一覧**: 編集用(`profile/hf/*.maze` + VSCode 拡張で使っていた `profile/maze.yaml`、上書き保存可)/ 受信ログ(`maze_logs/`、新しい順、読み取り専用)/ 大会迷路(`maze_data/*.yaml`、ゴール付き、読み取り専用)。読み取り専用のものは「別名で保存」で `profile/hf/<名前>.maze` に作る(既存名は拒否)。`profile/hf/*.maze` はプロファイルパネルにも出て、行クリックで迷路タブが開く(「全て送信」には含まれない)。
- **表示**: 壁は両側の区画の言い分で決める。片側だけに壁がある壁は金の点線で出し、ツールバーに「食い違い N」と数を出す(クリックで両側がそろう)。外周は片側しかないので、外周が欠けたファイルはそのまま欠けて見える。ゴールはファイル自身のもの(大会迷路)、無ければ system.yaml の `goals`。system.yaml は読むだけ(書き換えは行置換の決まり)。
- **編集**: 区画を対角線で 4 分割していちばん近い壁をトグル(拡張と同じ)。ドラッグは最初の壁で「置く/消す」と向き(横/縦)を決め、通った壁すべてに当てる(向きを固定しないと、格子線をなぞっても柱の近くで直交する壁を拾う)。1 ストロークで元に戻す 1 回。外周は触れない。Ctrl+Z / Ctrl+Shift+Z・Ctrl+Y / Ctrl+S はタブ表示中だけ拾う。編集中の内容を残すため、タブは隠すだけでマウントしたまま。
- **送信**: 表示中の壁(未保存でも)を `/maze.txt` として送る。ファームが読むのは次に `run_main_mode()` へ入ったときの `read_maze_data()`。大きさが system.yaml の `maze_size` と違うと壁が全部ずれるので API で拒否する。
- 機体から迷路を受信すると(`saved` の `type: "maze"`)一覧を取り直し、トーストのクリックでその迷路を開く。

## Flash — `lib/flash.ts`

リポジトリルートの `flash.sh`(picotool)を `spawn` で実行し、stdout/stderr を1行ずつ `[flash] `プレフィックス付きで `serialManager` の `log` イベントへ流す(コンソールパネルにそのまま表示される)。picotool は USB デバイスを排他的に掴み、書き込み成功後は BOOTSEL から通常ファームウェアへ再起動して CDC デバイスが一旦消える。RX 接続を持ったままだと picotool と掴み合いになるため、実行前に `serialManager.disconnect()`、完了後(成功/失敗どちらでも)に `serialManager.enableAutoConnect()` を呼ぶ。明示的な再接続はせず、既存の200ms探索ループに検出を任せる。

## 既知のハマりどころ

- **余白は詰めてある(shadcn 既定より狭い)**: 情報密度優先で `components/ui/card.tsx` の `--card-spacing` を `--spacing(4)`→`--spacing(2)`(8px)、`ui/table.tsx` のセルを `px-1.5 py-1` に落とし、`app/page.tsx` のルートを `gap-2 p-2` にしてある。各パネルの `p-*`/`gap-*` もこれに合わせた。`npx shadcn add` で `ui/` を再生成すると既定値(16px)に戻るので、上書きされたら詰め直すこと。
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
| `/api/sensor-calib` | GET/POST | センサ校正: `gains`(現在値)/`dirs`/`load`、POST `save`(csv保存)/`apply`(sensor.yaml置換+任意で送信) |
| `/api/maze` | GET/POST | 迷路: `list`(一覧 + system.yaml の goals/maze_size)/`read`、POST `save`(編集用を上書き)/`saveAs`(profile/hf/へ新規)/`send`(maze.txt へ送信) |
| `/api/flash` | POST | `flash.sh`(picotool)実行。実行前にシリアル切断、完了後auto-connectを再有効化 |
