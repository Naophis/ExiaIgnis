#!/bin/bash
# 使い方: python3 で作ったサンプル列を流す(scratchpad の edge_dump.py 参照)。
# 引数 all で発火をすべて出す(ログ 1 本を丸ごと流して 2 回発火しないかを見る)
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -I../../include test.cpp -o /tmp/wall_edge_host_test
/tmp/wall_edge_host_test "$@"
