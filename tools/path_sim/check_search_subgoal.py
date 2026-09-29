#!/usr/bin/env python3
"""探索のサブゴールの選び方(offset.yaml の search_subgoal_mode)の確認。ファームのソースを変えたら回す。

    python3 check_search_subgoal.py

search_sim(ファームの adachi.cpp / logic.cpp そのもの)を maze_data・profile・maze_logs の迷路で回し、
  1. 検討時の実験(experiments/px/ref_search.json)と、迷路ごとの探索時間・判断の回数が同じか
  2. 探索後の地図で作った最短走行が、全区画既知のときより遅い件数(取りこぼし)
を出す。モード 0 = 重みパターン 1 を毎回(従来)、1 = 重みパターン 2 と 4、壁で塞がれたときだけ作り直す。
"""
import json, os, subprocess, sys
from multiprocessing import Pool

H = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(H, "experiments", "px"))
import ds  # noqa: E402  迷路の読み込み(重複を除く)

EXECS = [1, 3, 5, 11, 16]
KEEP = lambda m: m["group"] in ("contest32", "contest16", "regional") or m["name"] == "20260927_155341"
FILES = ds.profile_files()
import contextlib, io  # noqa: E402
with contextlib.redirect_stdout(io.StringIO()):
    MAZES = [m for m in ds.load_all() if KEEP(m)]


def truth_of(m):
    n = m["n"]
    return [m["walls"][x * n + y] & 15 for y in range(n) for x in range(n)]


def path_time(mp, goals, ex):
    inp = {"files": FILES, "map": mp, "goals": goals, "exec": ex, "direction": "right"}
    o = json.loads(subprocess.run([f"{H}/build/path_sim"], input=json.dumps(inp).encode(), capture_output=True, timeout=600).stdout)
    return o["goal_time"] if o.get("ok") else None


def run(args):
    mi, mode = args
    m = MAZES[mi]
    inp = {"files": FILES, "truth": truth_of(m), "goals": m["goals"], "subgoal_mode": mode}
    o = json.loads(subprocess.run([f"{H}/build/search_sim"], input=json.dumps(inp).encode(), capture_output=True, timeout=1200).stdout)
    # 最後の地図(判断ごとの変化を順に当てる)
    n = o["maze_size"]
    mp = [0] * (n * n)
    for st in o["steps"]:
        for i, v in st["c"]:
            mp[i] = v
    return {
        "total_time": o["total_time"], "n_steps": len(o["steps"]), "end_reason": o["end_reason"],
        "run": {ex: path_time(mp, m["goals"], ex) for ex in EXECS},
        "ideal": {ex: path_time(ds.fw_map(m), m["goals"], ex) for ex in EXECS},
    }


def main():
    subprocess.check_call(["make", "-s"], cwd=H)
    ref_file = os.path.join(H, "experiments", "px", "ref_search.json")
    ref = json.load(open(ref_file)) if os.path.exists(ref_file) else {}
    jobs = [(mi, mode) for mode in (0, 1) for mi in range(len(MAZES))]
    with Pool() as p:
        outs = p.map(run, jobs)
    bad = 0
    for mode, key in ((0, "ref_p1_only"), (1, "ref_2+4_limit1")):
        tot = 0; miss = 0; miss_ms = 0; fail = 0; diff = 0
        for (mi, md), o in zip(jobs, outs):
            if md != mode:
                continue
            name = MAZES[mi]["name"]
            tot += o["total_time"]
            fail += o["end_reason"] != "home"
            for ex in EXECS:
                t, idl = o["run"][ex], o["ideal"][ex]
                if t is None or t > idl + 1e-4:
                    miss += 1
                    miss_ms += (t - idl) * 1000 if t is not None else 0
            r = ref.get(key, {}).get(name)
            if r and (abs(r["total_time"] - o["total_time"]) > 1e-6 or r["n_steps"] != o["n_steps"]):
                diff += 1
                print(f"NG モード {mode} {name}: 実験 {r['total_time']:.3f} s / {r['n_steps']} 回、ファーム {o['total_time']:.3f} s / {o['n_steps']} 回")
        bad += diff
        print(f"モード {mode}: 探索の総時間 {tot:.0f} s  帰還できず {fail}  取りこぼし {miss} / {len(MAZES) * len(EXECS)}({miss_ms:.0f} ms)  実験との不一致 {diff}" + ("" if ref else "(基準なし)"))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
