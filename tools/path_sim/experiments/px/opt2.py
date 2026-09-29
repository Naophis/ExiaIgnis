"""速い版(impl 2)が試作(impl 1)と同じ答えか、計算時間はどれだけか"""
import json, subprocess, pickle, sys, os
import numpy as np
from multiprocessing import Pool
import ev, ds, opt
def run(args):
    ci, kw = args
    mi, ex = ev.CASES[ci]; m = ev.MAZES[mi]
    inp = {"files": ev._files, "map": kw.pop("map", None) or ds.fw_map(m), "goals": m["goals"], "exec": ex, "opt": True}
    inp.update(kw)
    r = subprocess.run([f"{opt.PX2}/px_opt"], input=json.dumps(inp).encode(), capture_output=True, timeout=1200)
    try: return json.loads(r.stdout)["opt"]
    except Exception: return {"found": False, "err": r.returncode}
def batch(kw, n=1):
    with Pool(n) as p: return p.map(run, [(ci, dict(kw)) for ci in range(len(ev.CASES))])
def start_map(m):
    # 探索の最初の状態に近いもの: 外周とスタート区画の壁だけ既知。残りは未知(壁なし扱い)
    n = m["n"]; mp = [0] * (32 * 32)
    for x in range(32):
        for y in range(32):
            v = 0
            if y == 31: v |= 1
            if x == 31: v |= 2
            if x == 0: v |= 4
            if y == 0: v |= 8
            mp[x + y * 32] = v
    mp[0] = 0x0e | 0xf0  # スタート: 東西南が壁
    mp[1] |= 0x04
    return mp
if __name__ == "__main__":
    d = pickle.load(open("opt.pkl", "rb")); ideal = d["opt_true"]
    a = batch(dict(impl=1), 22)
    b = batch(dict(impl=2, reps=20), 1)
    t1 = np.array([o["model_time"] for o in a]); t2 = np.array([o["model_time"] for o in b])
    print("既知の迷路: 試作と速い版の食い違い", int((np.abs(t1 - t2) > 1e-5).sum()), " 基準との食い違い", int((np.abs(t2 - ideal) > 1e-5).sum()))
    ms1 = np.array([o["ms"] for o in a]); ms2 = np.array([o["ms"] for o in b])
    print(f"  計算(PC 1 本): 試作 平均 {ms1.mean():.2f} ms / 速い版 平均 {ms2.mean() * 1000:.0f} us 最大 {ms2.max() * 1000:.0f} us  節点 平均 {np.mean([o['nodes'] for o in b]):.0f} 最大 {max(o['nodes'] for o in b)}  辺 平均 {np.mean([o['edges'] for o in b]):.0f}  ヒープ最大 {max(o['heap_max'] for o in b)}")
    c = batch(dict(impl=2, reps=20, astar=True), 1)
    t3 = np.array([o["model_time"] for o in c]); ms3 = np.array([o["ms"] for o in c])
    print(f"  下限つき(A*): 食い違い {int((np.abs(t3 - ideal) > 1e-5).sum())}  平均 {ms3.mean() * 1000:.0f} us 最大 {ms3.max() * 1000:.0f} us  節点 平均 {np.mean([o['nodes'] for o in c]):.0f}")
    # 未知だらけ(探索の最初)
    for name, kw in [("そのまま", dict()), ("下限つき(A*)", dict(astar=True)), ("下限 ×2(厳密でない)", dict(astar=True, weight=2.0)), ("下限 ×3", dict(astar=True, weight=3.0))]:
        outs = []
        for ci in range(len(ev.CASES)):
            m = ev.MAZES[ev.CASES[ci][0]]
            k = dict(impl=2, reps=1, search=True, map=start_map(m)); k.update(kw)
            outs.append(run((ci, k)))
        ms = np.array([o["ms"] for o in outs]); tt = np.array([o["time"] for o in outs])
        if name == "そのまま": base_t = tt
        print(f"未知だらけ {name:18s}: 平均 {ms.mean():.1f} ms 最大 {ms.max():.1f} ms  節点 平均 {np.mean([o['nodes'] for o in outs]):.0f} 最大 {max(o['nodes'] for o in outs)}  辺 平均 {np.mean([o['edges'] for o in outs]):.0f}  ヒープ最大 {max(o['heap_max'] for o in outs)}  タイムの悪化 平均 {(tt / base_t).mean() * 100 - 100:+.2f} % 最大 {(tt / base_t).max() * 100 - 100:+.2f} %")
