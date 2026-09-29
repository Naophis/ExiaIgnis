#!/bin/bash
# 旋回の始まりを tick の途中へ合わせる処理のホスト検証(2026-09-30)
set -e
cd "$(dirname "$0")"
R=../..
g++ -std=c++17 -O2 -include host_limits_shim.h -I$R -I$R/include -I$R/gen_code_mpc \
  test.cpp $R/gen_code_mpc/mpc_tgt_calc.cpp $R/gen_code_mpc/mpc_tgt_calc_data.cpp \
  $R/gen_code_mpc/rt_nonfinite.cpp $R/gen_code_mpc/rtGetInf.cpp $R/gen_code_mpc/rtGetNaN.cpp \
  -o /tmp/sla_align_host_test
/tmp/sla_align_host_test "$@"
