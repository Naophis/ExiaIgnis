#!/usr/bin/env python3
"""タイム最小の経路探索(TimePathPlanner)の確認。ファームのソースを変えたら回す。

    python3 check_time_path.py            # maze_data の迷路 × 走行モード 1 / 3 / 5 / 11 / 16
    python3 check_time_path.py --cap 4096 # 節点の上限を機体の想定にして、あふれる迷路を見る

確かめること:
  1. 求めた経路を convert_large_path / diagonalPath / calc_goal_time に通したタイムが、探索の
     タイムと同じ(辺の作り方が変換の規則と合っている)
  2. 従来の方法(重みパターン 1〜5 の比較)より遅くならない
"""
import argparse, glob, json, os, subprocess, sys
from multiprocessing import Pool

import yaml

H = os.path.dirname(os.path.abspath(__file__))
PT = os.path.join(H, "..", "param_tuner")
EXECS = [1, 3, 5, 11, 16]


def profile_files():
    files = {}
    for f in ["system.yaml", "hardware.yaml"]:
        files[f.replace("yaml", "txt")] = json.dumps(yaml.safe_load(open(f"{PT}/profile/{f}")))
    for f in sorted(glob.glob(f"{PT}/profile/hf/*.yaml")):
        files[os.path.basename(f).replace("yaml", "hf")] = json.dumps(yaml.safe_load(open(f)))
    return files


def mazes():
    out = []
    for f in sorted(glob.glob(f"{PT}/maze_data/*.yaml")):
        d = yaml.safe_load(open(f))["maze_data"]
        n = d["maze_size"]
        w = d["wall"]
        # ファームの並び map[x + y * n]、全区画既知
        mp = [(w[x * n + y] & 15) | 0xF0 for y in range(n) for x in range(n)]
        out.append((os.path.basename(f)[:-5], mp, d["goal"]))
    return out


FILES = profile_files()
MAZES = mazes()


def run(args):
    mi, ex, method, cap = args
    name, mp, goals = MAZES[mi]
    inp = {"files": FILES, "map": mp, "goals": goals, "exec": ex, "direction": "right", "method": method}
    if cap:
        inp["node_cap"] = cap
    r = subprocess.run([f"{H}/build/path_sim"], input=json.dumps(inp).encode(), capture_output=True, timeout=600)
    try:
        return json.loads(r.stdout)
    except Exception:
        return {"ok": False, "error": f"rc={r.returncode}"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cap", type=int, default=0, help="節点の上限(機体は空きメモリから 1024〜8192)")
    args = ap.parse_args()
    subprocess.check_call(["make", "-s"], cwd=H)
    cases = [(mi, ex) for mi in range(len(MAZES)) for ex in EXECS]
    with Pool() as p:
        new = p.map(run, [(mi, ex, "time", args.cap) for mi, ex in cases])
        old = p.map(run, [(mi, ex, "patterns", 0) for mi, ex in cases])
    bad = 0
    n_time = n_fallback = faster = 0
    gain = []
    ms_new = ms_old = 0
    nodes_max = heap_max = seg_max = 0
    for (mi, ex), a, b in zip(cases, new, old):
        name = MAZES[mi][0]
        if not a.get("ok") or not b.get("ok"):
            bad += 1
            print(f"NG {name} exec {ex}: 経路なし {a.get('error')} / {b.get('error')}")
            continue
        pl = a["planner"]
        ms_new += a["calc_ms"]
        ms_old += b["calc_ms"]
        if a["method_used"] != "time":
            n_fallback += 1
            print(f"   {name} exec {ex}: {pl['result']}(節点 {pl['nodes']}/{pl['node_cap']})→ 従来の方法")
            continue
        n_time += 1
        nodes_max = max(nodes_max, pl["nodes"])
        heap_max = max(heap_max, pl["heap_max"])
        seg_max = max(seg_max, pl["seg_cached"])
        if abs(pl["time"] - a["goal_time"]) > 1e-4:
            bad += 1
            print(f"NG {name} exec {ex}: 探索 {pl['time']:.4f} と calc_goal_time {a['goal_time']:.4f} が違う")
        if a["goal_time"] > b["goal_time"] + 1e-4:
            bad += 1
            print(f"NG {name} exec {ex}: 従来 {b['goal_time']:.4f} より遅い {a['goal_time']:.4f}")
        if a["goal_time"] < b["goal_time"] - 1e-4:
            faster += 1
            gain.append(b["goal_time"] - a["goal_time"])
    print(f"{len(cases)} 件: タイム最小 {n_time} / 従来へ戻った {n_fallback} / NG {bad}")
    print(f"  従来より速い {faster} 件" + (f"(平均 {sum(gain) / len(gain) * 1000:.0f} ms、最大 {max(gain) * 1000:.0f} ms)" if gain else ""))
    print(f"  計算(PC): {ms_new / len(cases):.2f} ms / 件(従来 {ms_old / len(cases):.0f} ms)")
    print(f"  節点 最大 {nodes_max}  ヒープ 最大 {heap_max}  覚えた区間 最大 {seg_max}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
