#!/bin/bash
# 柱の谷(下に凸)検知器 include/planning/pillar_trough_detector.hpp のホストテスト。
# 1) 実ログから切り出した列(20260923_064918 idx2196-2218 等)での発火/不発火の固定検証
# 2) 引数に CSV ログ(tools/param_tuner/logs/*.csv)を渡すと、その全行を再生して
#    発火イベント(index, 側, 種別, 谷底, 発火−谷底)を列挙する(WALL_OFF 区間との
#    突き合わせ用。tools/param_tuner/logs は git 管理外なので手元で実行する)。
#   ./run.sh                       # 固定検証のみ
#   ./run.sh ../../tools/param_tuner/logs/20260923_064918.csv
set -e
cd "$(dirname "$0")"
ROOT="$(cd ../.. && pwd)"
g++ -std=gnu++20 -O2 -Wall -I"$ROOT/include" test.cpp -o /tmp/pillar_trough_host_test
/tmp/pillar_trough_host_test "$@"
