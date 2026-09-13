#!/bin/bash
# DitherPwm のリング更新ロジックをホスト(x86_64)でシミュレーションするテスト。
# Pico SDK のレジスタ/関数を stub/ で模擬し、実機と同じ src/driver/dither_pwm.cpp をそのまま使う。
#   DMA  = 1 PWM 周期ごとに read_addr のリング語を CC へ転送して read_addr++(1024 B ラップ)
#   PWM  = 周期 k に CC へ書かれた値は周期 k+1 の出力(double-buffer)
# 検証: 平均の厳密一致 / {N,N+1} 値域 / 指令レイテンシ / fail-static(tick スキップ) /
#       2 slice の同一 wrap 反映 / M=100 / force_static / stop→start / tick ジッタ
set -e
cd "$(dirname "$0")"
ROOT="$(cd ../.. && pwd)"
g++ -std=gnu++20 -O2 -Wall -Wno-attributes -DDPWM_HOST_TEST -Istub -I"$ROOT/include" \
    "$ROOT/src/driver/dither_pwm.cpp" test.cpp -o /tmp/dither_pwm_host_test
/tmp/dither_pwm_host_test
