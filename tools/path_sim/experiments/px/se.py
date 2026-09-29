"""探索の比較: サブゴールの選び方ごとに、探索時間と、探索後の地図で出せる最短走行のタイム(取りこぼし)"""
import json, subprocess, pickle, sys, os
import numpy as np
from multiprocessing import Pool
import ev, ds, opt
PX2 = opt.PX2
SUB_EXECS = [3, 11, 16]
VARIANTS = {
    "予備なし(以前)": dict(sub_primary=1, sub_fallbacks=[]),
    "予備 4→3→2(いまのファーム)": dict(sub_primary=1, sub_fallbacks=[4, 3, 2]),
    "予備 4 だけ": dict(sub_primary=1, sub_fallbacks=[4]),
    "予備 = タイム最小": dict(sub_primary=1, sub_fallbacks=[], sub_dp=1, sub_execs=SUB_EXECS),
    "毎回タイム最小": dict(sub_primary=1, sub_fallbacks=[], sub_dp=2, sub_execs=SUB_EXECS),
    "予備 = タイム最小(速い版)": dict(sub_primary=1, sub_fallbacks=[], sub_dp=1, sub_execs=SUB_EXECS),
    "予備 = タイム最小 + 既知走行中も": dict(sub_primary=1, sub_fallbacks=[], sub_dp=3, sub_execs=SUB_EXECS),
    "予備 = タイム最小 + 既知走行中も(モード 1 つ)": dict(sub_primary=1, sub_fallbacks=[], sub_dp=3, sub_execs=[11]),
    "止まっている間だけ(超信地 + 帰還時)": dict(sub_primary=1, sub_fallbacks=[], sub_dp=4, sub_execs=SUB_EXECS),
    "帰還時だけ確認して出直す": dict(sub_primary=1, sub_fallbacks=[], sub_dp=5, sub_execs=SUB_EXECS),
    "予備 4→3→2 + 帰還時に確認": dict(sub_primary=1, sub_fallbacks=[4, 3, 2], sub_dp=5, sub_execs=SUB_EXECS),
    "予備 4 + 止まっている間だけ": dict(sub_primary=1, sub_fallbacks=[4], sub_dp=4, sub_execs=SUB_EXECS),
    "毎回タイム最小(速い版)": dict(sub_primary=1, sub_fallbacks=[], sub_dp=2, sub_execs=SUB_EXECS),
}
def truth_of(m):
    n = m["n"]; w = m["walls"]
    return [w[x * n + y] & 15 for y in range(n) for x in range(n)]
def search(args):
    mi, name = args
    m = ev.MAZES[mi]
    inp = {"files": ev._files, "truth": truth_of(m), "goals": m["goals"]}; inp.update(VARIANTS[name])
    r = subprocess.run([f"{PX2}/px_search"], input=json.dumps(inp).encode(), capture_output=True, timeout=1200)
    try: o = json.loads(r.stdout)
    except Exception: return {"ok": False, "error": f"rc={r.returncode}"}
    if not o.get("ok"): return o
    # 探索後の地図(既知の区画だけ)でのタイム最小
    o["run"] = {}
    for ex in ev.EXECS:
        inp2 = {"files": ev._files, "map": o["final_map"], "goals": m["goals"], "exec": ex, "opt": True, "count_final": True}
        q = json.loads(subprocess.run([f"{PX2}/px_opt"], input=json.dumps(inp2).encode(), capture_output=True, timeout=600).stdout)
        o["run"][ex] = q["opt"]["model_time"] if q.get("ok") and q["opt"]["found"] else None
    known = sum(1 for v in o["final_map"] if (v & 0xf0) == 0xf0)
    o["known"] = known
    del o["final_map"]
    return o
if __name__ == "__main__":
    d = pickle.load(open("opt.pkl", "rb")); ideal = d["opt_true"]
    idx = {(mi, ex): ci for ci, (mi, ex) in enumerate(ev.CASES)}
    res = pickle.load(open("se.pkl", "rb")) if os.path.exists("se.pkl") else {}
    redo = set(sys.argv[1:])
    jobs = [(mi, name) for name in VARIANTS for mi in range(len(ev.MAZES)) if (mi, name) not in res or name in redo]
    with Pool(22) as p: outs = p.map(search, jobs)
    for (mi, name), o in zip(jobs, outs): res[(mi, name)] = o
    pickle.dump(res, open("se.pkl", "wb"))
    print(f"{'サブゴールの選び方':28s} 探索時間の合計   帰還できず  取りこぼし(迷路×モード)  取りこぼしの合計  最大     予備の回数  計算(PC)")
    for name in VARIANTS:
        tot = 0; fail = 0; miss_n = 0; miss_s = 0; miss_max = 0; nf = 0; fb = 0; dpms = 0; dpc = 0; rs_ = 0
        for mi in range(len(ev.MAZES)):
            o = res[(mi, name)]
            if not o.get("ok"): fail += 1; continue
            tot += o["total_time"]; fb += o.get("fallback_used", 0); dpms += o.get("dp_ms", 0); dpc += o.get("dp_calls", 0); rs_ += o.get("resorties", 0)
            if o["end_reason"] != "home": fail += 1
            for ex in ev.EXECS:
                t = o["run"][ex]; idl = ideal[idx[(mi, ex)]]
                if t is None: nf += 1; continue
                if t > idl + 1e-4: miss_n += 1; miss_s += t - idl; miss_max = max(miss_max, t - idl)
        print(f"{name:28s} {tot:9.0f} s   {fail:3d}       {miss_n:3d} / {len(ev.MAZES) * len(ev.EXECS)}             {miss_s * 1000:7.0f} ms     {miss_max * 1000:4.0f} ms   {fb:4d}     {dpc} 回 {dpms:.0f} ms  出直し {rs_}")
    print()
    names = [n for n in VARIANTS if n in sys.argv[1:]] or list(VARIANTS)[:5]
    print("迷路ごとの探索時間 [s] と取りこぼし [ms](モード 1/3/5/11/16)")
    for mi, m in enumerate(ev.MAZES):
        row = f"{m['name']:18s}"
        for name in names:
            o = res[(mi, name)]
            if not o.get("ok"): row += "   ERR"; continue
            ms = [0 if o["run"][ex] is None else round((o["run"][ex] - ideal[idx[(mi, ex)]]) * 1000) for ex in ev.EXECS]
            row += f" | {o['total_time']:4.0f}{'' if o['end_reason'] == 'home' else '!' + o['end_reason']} " + "/".join(str(v) for v in ms)
        print(row)
