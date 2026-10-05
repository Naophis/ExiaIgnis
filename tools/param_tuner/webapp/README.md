# Param Console

ExiaIgnis の Pico と通信するための Web アプリ。旧来の `console.sh`(`rx_term.js`)と `update_param.sh`(`tx_term.js`)を1本の永続シリアル接続に統合し、ブラウザから「ログ監視」「パラメータ送信」「YAML編集」「system.yamlテストテンプレート」「走行ログの軌跡プロット」をまとめて扱えるようにしたものです。

## 起動方法

リポジトリルートから:

```bash
./param_console.sh
```

またはこのディレクトリで直接:

```bash
npm install   # 初回のみ
npm run dev
```

`http://localhost:3000` を開く。Pico を USB 接続すると `ttyACM*` を自動検出して接続します(手動操作不要)。

## 機能

- **自動接続**: `ttyACM*` かつ `serialNumber` を持つデバイスを 200ms 間隔でポーリングし、見つかり次第自動接続。切断時も自動的に再検出。
- **コンソール**: シリアル受信をリアルタイム表示。Pause/Clear、`ESC[2J`(画面クリア)を検知して自動リセット。
- **機体(複数個体)**: パラメータは機体ごとに `tools/param_tuner/machines/<機体>/profile/` に持つ(登録簿は `machines.yaml`)。ヘッダーの「機体」で表示する機体を切り替える。基板を登録しておくと、つないだときに USB のシリアル番号で機体を見分けて表示を切り替え、別の機体のパラメータを送ろうとすると確認を出す。「機体比較」で機体どうしの違いの一覧と同期(コメントを残したままキー単位でコピー)、「機体設定」で機体の追加(既存の機体をコピー / ブランチの `profile` を取り込む)。
- **ログの保存先**: 受信したログ(csv)は、つないでいる基板の機体の `tools/param_tuner/machines/<機体>/logs/` に保存される。プロットタブの一覧はその機体のログだけ。未登録の基板から受信したログは共通の `tools/param_tuner/logs/` に入り、見出しの「共通 N」で出し入れする(2026-10-04 以前のログは `machines/calibur/logs/` へ移してある)。迷路は全機体で共通。
- **見た目**: ソレスタルビーイングの作戦端末を思わせる暗いテーマ(濃紺の地、発光線と鉤括弧で縁取ったパネル、Eurostile 系の銘、ヘッダーの下を流れる GN 粒子の線)。飾りは `app/globals.css` にまとめてある。
- **テーマカラー**: ヘッダーの「テーマ」で画面全体の主色を変えられる。既定は機体ごとの色(機体を切り替えると全体の色も変わる。色は `machines.yaml` に残る)。チェックを外すと全機体で 1 色。
- **パラメータ送信**: 表示中の機体の `profile/hf/` 配下と `system.yaml`/`hardware.yaml`/`am32.yaml` を一覧表示し、個別送信・全送信・未送信だけ送信(つないでいる基板へ最後に送った中身と違うファイルに ● が付く)・検索フィルタ・クリックでのYAML編集(CodeMirror+VSCodeテーマ、Ctrl+Sで保存)に対応。編集画面では、ほかの機体と値が違う行に印が付き、保存時に「ほかの機体も同じ値だったキー」へ同じ変更を入れられる。
- **system.yaml テストテンプレート**: `test:` ブロックの特定キー(v_max/accl/decel/dia_accl/dia_decel/dist/suction_active/file_idx/sla_type/sla_type2/sla_return/ignore_opp_sen/search_mode)とトップレベルの `mode` を、コメントを一切壊さずに書き換える仕組み。名前付きテンプレートの保存/適用に加え、よく変える値(mode/file_idx/sla_type/sla_type2/sla_return)はワンクリックで即時反映できる「クイック適用」ボタンを用意。
- **AM32 ESC書き込み**: `am32.yaml` の行(と編集画面)から「ESC書込」でファイル送信+`AM32WRITE`(=`send_file.py am32sync` 相当)、「ESC読出」で `AM32READ` を実行。進捗はコンソールにそのまま流れる。デバイスが起動直後のボタン待ちループにいる必要がある。
- **ログプロット**: `tools/param_tuner/logs/` のCSVから走行軌跡を描画(状態ごとに色分け、壁センサー検出点、90mmグリッド)。PlotJugglerでの詳細解析への連携ボタンつき。

アーキテクチャ・プロトコルの詳細は [CLAUDE.md](./CLAUDE.md) を参照してください。

## 必要環境

- Node.js 18+ (開発は Node 24 で確認)
- (任意) PlotJuggler 連携を使う場合は ROS 2 + PlotJuggler (`ros2 run plotjuggler plotjuggler`) が同一マシンにインストールされていること
