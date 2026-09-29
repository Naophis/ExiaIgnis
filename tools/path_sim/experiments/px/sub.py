"""重みパターンの組の総当たり(経路を覚えておき、塞がれたら作り直す版)"""
import itertools, pickle, sys, os
import numpy as np
from multiprocessing import Pool
import ev, se
def main():
    d = pickle.load(open("opt.pkl", "rb")); ideal = d["opt_true"]
    idx = {(mi, ex): ci for ci, (mi, ex) in enumerate(ev.CASES)}
    sets = [list(c) for k in range(1, 6) for c in itertools.combinations([1, 2, 3, 4, 5], k)]
    cache = pickle.load(open("sub.pkl", "rb")) if os.path.exists("sub.pkl") else {}
    jobs = []
    for s in sets:
        name = "set" + "".join(map(str, s))
        se.VARIANTS[name] = dict(sub_primary=1, sub_fallbacks=[], sub_dp=9, sub_multi=s)
        for mi in range(len(ev.MAZES)):
            if (mi, name) not in cache: jobs.append((mi, name))
    with Pool(22, initializer=init, initargs=(se.VARIANTS,)) as p: outs = p.map(se.search, jobs)
    for j, o in zip(jobs, outs): cache[j] = o
    pickle.dump(cache, open("sub.pkl", "wb"))
    rows = []
    for s in sets:
        name = "set" + "".join(map(str, s))
        tot = 0; n = 0; ms = 0; mx = 0; re = 0; calls = 0; fail = 0
        for mi in range(len(ev.MAZES)):
            o = cache[(mi, name)]
            tot += o["total_time"]; re += o["p_recompute"]; calls += o["p_calls"]; fail += o["end_reason"] != "home"
            for ex in ev.EXECS:
                t = o["run"][ex]; idl = ideal[idx[(mi, ex)]]
                if t is not None and t > idl + 1e-4: n += 1; ms += t - idl; mx = max(mx, t - idl)
        rows.append((s, tot, n, ms * 1000, mx * 1000, re / calls, fail))
    rows.sort(key=lambda r: r[1])
    print("パターンの組      探索時間   取りこぼし  遅れの合計  最大    作り直し(枚/移動)")
    for s, tot, n, ms, mx, rpc, fail in rows:
        dominated = any((r[1] <= tot and r[2] <= n and r[3] <= ms) and (r[1] < tot or r[2] < n or r[3] < ms) for r in rows)
        print(f"{'+'.join(map(str, s)):14s} {tot:7.0f} s   {n:3d}/105   {ms:6.0f} ms  {mx:4.0f} ms   {rpc:.2f}  {'' if dominated else '◎ これより両方良い組は無い'}{' 帰還できず ' + str(fail) if fail else ''}")
def init(v):
    se.VARIANTS.update(v)
if __name__ == "__main__":
    main()
