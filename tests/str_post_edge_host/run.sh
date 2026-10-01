#!/bin/bash
# 直進の柱の立ち下がり(include/planning/dia_post_edge_detector.hpp の rel_thr > 0 の経路)の
# ホスト検証。Python 版(tools/param_tuner/str_post_edge.py)と組(位置・δ・ψ0)が一致するか。
#   ./run.sh logs/*.csv
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -Wall -I../../include test.cpp -o /tmp/str_post_edge_host_test
files=()
for f in "$@"; do files+=("$(cd "$OLDPWD" && realpath "$f")"); done
python3 ../../tools/param_tuner/str_post_edge.py --dump "${files[@]}" | /tmp/str_post_edge_host_test | python3 compare.py
