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
- **パラメータ送信**: `profile/hf/` 配下と `system.yaml`/`hardware.yaml`/`am32.yaml` を一覧表示し、個別送信・全送信・検索フィルタ・クリックでのYAML編集(CodeMirror+VSCodeテーマ、Ctrl+Sで保存)に対応。
- **system.yaml テストテンプレート**: `test:` ブロックの特定キー(v_max/accl/decel/dia_accl/dia_decel/dist/suction_active/file_idx/sla_type/sla_type2/sla_return/ignore_opp_sen/search_mode)とトップレベルの `mode` を、コメントを一切壊さずに書き換える仕組み。名前付きテンプレートの保存/適用に加え、よく変える値(mode/file_idx/sla_type/sla_type2/sla_return)はワンクリックで即時反映できる「クイック適用」ボタンを用意。
- **ログプロット**: `tools/param_tuner/logs/` のCSVから走行軌跡を描画(状態ごとに色分け、壁センサー検出点、90mmグリッド)。PlotJugglerでの詳細解析への連携ボタンつき。

アーキテクチャ・プロトコルの詳細は [CLAUDE.md](./CLAUDE.md) を参照してください。

## 必要環境

- Node.js 18+ (開発は Node 24 で確認)
- (任意) PlotJuggler 連携を使う場合は ROS 2 + PlotJuggler (`ros2 run plotjuggler plotjuggler`) が同一マシンにインストールされていること
