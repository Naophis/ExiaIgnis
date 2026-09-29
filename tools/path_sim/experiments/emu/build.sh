#!/bin/bash
# 機体と同じ CPU(Cortex-M33)向けに、探索の計算だけを取り出した測定用の ELF を作る。
#   ./build.sh            bench_orig.elf(ファームの logic.cpp そのまま)と bench_opt.elf(opt.cpp の試作)
#   snap_sim              ホスト用。探索シミュレータを回し、update() の直前の状態を snap_<迷路>.bin に残す
# コンパイラとオプションはファームのビルド(build/compile_commands.json)と同じもの。
set -e
H=$(cd "$(dirname "$0")" && pwd)
R=$(cd "$H/../../../.." && pwd)
CXX=$(python3 - "$R" <<'PY'
import json, sys
for e in json.load(open(sys.argv[1] + "/build/compile_commands.json")):
    if e["file"].endswith("src/search/logic.cpp"):
        print(e["command"].split()[0]); break
PY
)
FLAGS="-mcpu=cortex-m33 -mthumb -march=armv8-m.main+fp+dsp -mfloat-abi=softfp -O3 -DNDEBUG -std=gnu++20 -fno-exceptions -fno-unwind-tables -fno-rtti -fno-use-cxa-atexit -ffunction-sections -fdata-sections -w"
INC="-I$H -I$R/tools/path_sim/stub -I$R -I$R/include -I$R/include/search -I$R/include/action"
cd "$H"
$CXX $FLAGS $INC -nostartfiles --specs=nosys.specs -T link.ld -Wl,--gc-sections bench.cpp marks.cpp $R/src/search/logic.cpp -o bench_orig.elf
$CXX $FLAGS $INC -nostartfiles --specs=nosys.specs -T link.ld -Wl,--gc-sections -DOPT bench.cpp marks.cpp opt.cpp $R/src/search/logic.cpp -o bench_opt.elf
python3 make_snap_main.py
g++ -std=gnu++20 -O2 -w -I$R/tools/path_sim/stub -I$R/tools/path_sim -I$R -I$R/include -I$R/include/search -I$R/include/action -I$R/build/_deps/arduinojson-src/src \
  $R/src/search/logic.cpp $R/src/search/adachi.cpp $R/src/action/path_creator.cpp $R/src/action/time_path_planner.cpp $R/src/action/trajectory_creator.cpp $R/tools/path_sim/host_common.cpp snap_main.cpp -o snap_sim
echo built
