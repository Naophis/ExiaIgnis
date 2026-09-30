#!/bin/bash
# DiaPostEdgeDetector(include/planning/dia_post_edge_detector.hpp)のホスト検証。
#   ./run.sh diag logs/*.csv   Python 版(tools/param_tuner/dia_post_edge.py)と δ・位置が一致するか
#   ./run.sh full logs/*.csv   ログを丸ごと流し、斜めの区間の外で組ができないか
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -Wall -I../../include test.cpp -o /tmp/dia_post_edge_host_test
mode=$1; shift
files=()
for f in "$@"; do files+=("$(cd "$OLDPWD" && realpath "$f")"); done
python3 dump.py "$mode" "${files[@]}"
