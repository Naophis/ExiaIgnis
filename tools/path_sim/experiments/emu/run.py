#!/usr/bin/env python3
"""探索の計算(update() の中身と exec() の歩数マップ)の命令数を、変更前・試作・いまのファームで比べる。
    pip install --target pylib unicorn   # 最初に 1 回(このフォルダの pylib に入れる)
    ./build.sh && python3 run.py [迷路名 ...]
結果(表・サブゴール・歩数マップ)のチェックサムが変更前と同じかも見る。
"""
import json, os, subprocess, sys, statistics
H = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(H, "..", "px"))
import ev, se  # noqa: E402
import emu  # noqa: E402

ELFS = (("bench_orig.elf", "変更前"), ("bench_opt.elf", "試作 opt"), ("bench_opt2.elf", "試作 opt2"), ("bench_fw.elf", "いまのファーム"))
names = sys.argv[1:] or ["japan2025_final", "32MM2019HX", "japan2019hes"]
for name in names:
    m = [x for x in ev.MAZES if x["name"] == name][0]
    snap = f"{H}/snap_{name}.bin"
    inp = {"files": ev._files, "truth": se.truth_of(m), "goals": m["goals"], "snap_out": snap}
    subprocess.run([f"{H}/snap_sim"], input=json.dumps(inp).encode(), capture_output=True, check=True)
    base = None
    for elf, label in ELFS:
        w, p, done, mismatch = emu.run(f"{H}/{elf}", snap, 0, 40, 12)
        if base is None:
            base = emu.run.checksum
        same = "同じ" if emu.run.checksum == base else "違う"
        near = f"  作り直さないとき 平均 {statistics.mean(w[3]):>8,.0f} 最大 {max(w[3]):>8,}" if w[3] else ""
        print(f"{name:18s} {label:8s} {done} 状態  不一致 {mismatch}  結果は変更前と{same}  update の中身 平均 {statistics.mean(w[1]):>10,.0f} 最大 {max(w[1]):>10,}"
              f"  歩数マップ 平均 {statistics.mean(w[2]):>8,.0f} 最大 {max(w[2]):>8,}{near}")
