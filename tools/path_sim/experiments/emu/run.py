#!/usr/bin/env python3
"""探索の計算(update() の中身と exec() の歩数マップ)の命令数を、ファームそのままと試作で比べる。
    pip install --target pylib unicorn   # 最初に 1 回(このフォルダの pylib に入れる)
    ./build.sh && python3 run.py [迷路名 ...]
"""
import json, os, subprocess, sys, statistics
H = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(H, "..", "px"))
import ev, se  # noqa: E402
import emu  # noqa: E402
names = sys.argv[1:] or ["japan2025_final", "32MM2019HX", "japan2019hes"]
for name in names:
    m = [x for x in ev.MAZES if x["name"] == name][0]
    snap = f"{H}/snap_{name}.bin"
    inp = {"files": ev._files, "truth": se.truth_of(m), "goals": m["goals"], "snap_out": snap}
    subprocess.run([f"{H}/snap_sim"], input=json.dumps(inp).encode(), capture_output=True, check=True)
    for elf, label in (("bench_orig.elf", "ファームそのまま"), ("bench_opt.elf", "試作")):
        w, p, done, mismatch = emu.run(f"{H}/{elf}", snap, 0, 40, 12)
        print(f"{name:18s} {label:10s} {done} 状態  結果の不一致 {mismatch}  update の中身 平均 {statistics.mean(w[1]):>10,.0f} 最大 {max(w[1]):>10,}  歩数マップ 平均 {statistics.mean(w[2]):>8,.0f} 最大 {max(w[2]):>8,}")
