import json, subprocess, sys, ev, ds, rs, t2, opt, pickle
import numpy as np
def moves_of(raw_s, raw_t):
    mv = ""
    for i, t in enumerate(raw_t):
        s = raw_s[i]
        f = (s - 3) / 2 if i == 0 else (s - 2) / 2
        if t == 255: f -= 1
        mv += "F" * int(round(f))
        if t != 255: mv += "R" if t == 1 else "L"
    return mv
def trace(ci, mv, count_final=False):
    mi, ex = ev.CASES[ci]; m = ev.MAZES[mi]
    inp = {"files": ev._files, "map": ds.fw_map(m), "goals": m["goals"], "exec": ex, "trace": mv, "count_final": count_final}
    return json.loads(subprocess.run([f"{opt.PX2}/px_opt"], input=json.dumps(inp).encode(), capture_output=True).stdout)["trace"]
if __name__ == "__main__":
    res = ev.evaluate([t2.tab6(rs.FWV[i]) for i in ev.FW], exe=t2.PX2, want_path=True)
    nok = 0; n = 0; shown = 0
    for ci in range(len(ev.CASES)):
        for r in res[ci]:
            mv = moves_of(r["raw_s"], r["raw_t"]); n += 1
            o = trace(ci, mv)
            if o["ok"] and abs(o["cost"] - r["time"]) < 1e-4: nok += 1
            elif shown < 12:
                shown += 1
                print(ev.MAZES[ev.CASES[ci][0]]["name"], ev.CASES[ci][1], "id", r["id"], "ok", o["ok"], "reached", o["reached"], "/", o["len"], "cost", o["cost"], "fw", r["time"])
                print("   moves:", mv[max(0, o["reached"] - 12):o["reached"]], "|", mv[o["reached"]:o["reached"] + 12])
                print("   ", o["info"][:300])
    print("なぞれて同じタイム:", nok, "/", n)
