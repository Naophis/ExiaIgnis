import json, subprocess, sys, numpy as np
from multiprocessing import Pool
import ev, ds, se, opt
def run(args):
    mi, kw = args
    m = ev.MAZES[mi]
    inp = {"files": ev._files, "truth": se.truth_of(m), "goals": m["goals"], "sub_primary": 1, "sub_fallbacks": [], "sub_dp": 4, "sub_execs": [11]}
    inp.update(kw)
    o = json.loads(subprocess.run([f"{opt.PX2}/px_search"], input=json.dumps(inp).encode(), capture_output=True, timeout=1200).stdout)
    return o["dp_log"], o["total_time"], o["end_reason"]
if __name__ == "__main__":
    for name, kw in [("そのまま", dict(dp_astar=0)), ("下限つき", dict(dp_astar=1)), ("下限 ×1.2", dict(dp_astar=1, dp_weight=1.2)), ("下限 ×1.5", dict(dp_astar=1, dp_weight=1.5))]:
        with Pool(22) as p: outs = p.map(run, [(mi, kw) for mi in range(len(ev.MAZES))])
        logs = np.array([r for o in outs for r in o[0]])
        print(f"{name:10s} 呼び出し {len(logs)} 回  1 回 平均 {logs[:, 0].mean():.2f} ms 最大 {logs[:, 0].max():.2f} ms  節点 平均 {logs[:, 1].mean():.0f} 最大 {logs[:, 1].max():.0f}  辺 平均 {logs[:, 2].mean():.0f}  展開 {logs[:, 3].mean():.0f}  ヒープ最大 {logs[:, 4].max():.0f}  探索時間 {sum(o[1] for o in outs):.0f} s  帰還 {sum(o[2] == 'home' for o in outs)}")
