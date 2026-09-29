import os
import json, subprocess, ev, ds
R = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", ".."))
if __name__ == "__main__":
    res = ev.evaluate([[7, 2, 1, 4.9497475, 2.9698485, 1.979899]], fw_ids=ev.FW)
    bad = 0; tot = 0; ms = 0
    for ci, (mi, ex) in enumerate(ev.CASES):
        m = ev.MAZES[mi]
        inp = {"files": ev._files, "map": ds.fw_map(m), "goals": m["goals"], "exec": ex, "direction": "right"}
        o = json.loads(subprocess.run([f"{R}/tools/path_sim/build/path_sim"], input=json.dumps(inp).encode(), capture_output=True).stdout)
        mine = min(ev.T(r) for r in res[ci][:5])
        cand = {c["type"]: c["time"] for c in o["candidates"]}
        same = all(abs(cand[r["id"]] - r["time"]) < 1e-6 for r in res[ci][:5])
        p6 = abs(res[ci][5]["time"] - res[ci][0]["time"]) < 1e-6 and res[ci][5]["sig"] == res[ci][0]["sig"]
        tot += mine; ms += sum(r["ms"] for r in res[ci][:5])
        if abs(mine - o["goal_time"]) > 1e-6 or not same or not p6:
            bad += 1; print("NG", m["name"], ex, mine, o["goal_time"], same, p6)
    print("cases", len(ev.CASES), "不一致", bad, "合計", round(tot, 3), "s  計算", round(ms), "ms")
