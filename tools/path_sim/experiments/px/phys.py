"""実験 2: 走行パラメータの加速度・ターン時間から作ったコスト表(直進 16 段 / 斜め 16 段)"""
import os
import json, subprocess, pickle, math, sys
import numpy as np
import ev, ds, rs, t2
R = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", ".."))
def params(ex):
    m = ev.MAZES[0]
    inp = {"files": ev._files, "map": ds.fw_map(m), "goals": m["goals"], "exec": ex, "direction": "left"}
    return json.loads(subprocess.run([f"{R}/tools/path_sim/build/path_sim"], input=json.dumps(inp).encode(), capture_output=True).stdout)
def straight_time(v1, vmax, v2, ac, diac, dist):
    # PathCreator::go_straight_dummy と同じ 1 ms 刻み
    if dist <= 0: return 0.0
    dt = 0.001; acc = ac; distance = 0.0; t = 0; V = v1; seq = 1
    d_need = abs((v1 * v1 - v2 * v2) / (2 * diac))
    if d_need > dist: acc = abs((v1 * v1 - v2 * v2) / (2 * dist)) + 1000
    while distance <= dist:
        t += 1
        d2 = abs((V + v2) * (V - v2) / (2.0 * diac))
        diac2 = -abs((V + v2) * (V - v2)) / (2 * abs(dist - distance)) if dist != distance else -1e30
        tmpv = V + acc * dt
        if seq == 1:
            seq = 2 if (dist - distance) < d2 else 1
            if tmpv >= vmax: acc = 0; V = vmax
            else: acc = ac
        if seq == 2:
            if tmpv <= v2: acc = 0; V = v2
            else: acc = min(diac, diac2) if diac2 < diac else diac
        V += acc * dt; distance += V * dt
        if V <= 0: return 10.0
    return t / 1000
def table(ex):
    p = params(ex)
    tp = {x["type"]: x for x in p["turn_params"]["normal"] if "v" in x}
    def tt(name):
        x = tp[name]; return x["time"] * 2 + max(x["front_l"], 0) / x["v"] + max(x["back_l"], 0) / x["v"]
    cell = p["cell_size"]; hd = cell * math.sqrt(2) / 2
    sS = p["straight_params"]["fast"]; sD = p["straight_params"]["dia"]
    names = {k.lower(): k for k in tp}
    L = tp[names["large"]]; D45 = tp[names["dia45"]]; D45_2 = tp[names["dia45_2"]]
    TS = lambda n: straight_time(L["v"], sS["v_max"], L["v"], sS["accl"], sS["decel"], (n - 1) * cell)
    TD = lambda n: straight_time(D45["v"], sD["v_max"], D45_2["v"], sD["accl"], sD["decel"], (n - 2) * hd)
    S = [max(TS(k + 1) - TS(k), 1e-3) if k >= 1 else 1e-3 for k in range(16)]
    tL = tt(names["large"]); t45 = tt(names["dia45"]) + tt(names["dia45_2"])
    D = [tL, max(t45 - tL, 1e-3)] + [max(TD(k + 1) - TD(k), 1e-3) for k in range(2, 16)]
    return S + D, dict(tL=tL, t45=t45, tO=tt(names["orval"]), t135=tt(names["dia135"]), t90=tt(names["dia90"]), vL=L["v"], accl=sS["accl"], vmax=sS["v_max"])
if __name__ == "__main__":
    d = pickle.load(open("opt.pkl", "rb")); ideal_fw = d["opt_fw"]
    r = pickle.load(open("rs_a.pkl", "rb")); fw5 = r["t"][:5].min(0); fw1 = r["t"][0]; fw1b = r["base"][0]
    tot = {k: 0 for k in ("t", "b", "t_rel", "b_rel", "ms")}; n = 0
    allt = np.zeros(len(ev.CASES)); allb = np.zeros(len(ev.CASES)); ms = 0
    for ex in ev.EXECS:
        tab, info = table(ex)
        if ex in (1, 16):
            print(f"exec {ex}: {info}")
            print("   直進の段 [ms]:", " ".join(f"{v * 1000:.0f}" for v in tab[:16]))
            print("   斜めの段 [ms]:", " ".join(f"{v * 1000:.0f}" for v in tab[16:]))
        cases = [ci for ci, (mi, e) in enumerate(ev.CASES) if e == ex]
        for relax in (0, 1):
            res = ev.evaluate([tab], exe=t2.PX2, extra={"relax": relax} if relax else None, cases=cases)
            for ci in cases:
                q = res[ci][0]
                if relax == 0: allt[ci] = ev.T(q); allb[ci] = q["base_time"]; ms += q["ms"]
                else:
                    if ci == cases[0]: pass
            if relax == 1:
                t1 = np.array([ev.T(res[ci][0]) for ci in cases]); b1 = np.array([res[ci][0]["base_time"] for ci in cases])
                print(f"exec {ex:2d} 直す版(relax): 分岐あり {(t1 / ideal_fw[cases]).mean() * 100 - 100:+.3f} %  分岐なし {(b1 / ideal_fw[cases]).mean() * 100 - 100:+.3f} %")
    print(f"加速度から作った表(1 パターン): 分岐あり {(allt / ideal_fw).mean() * 100 - 100:+.3f} %  分岐なし {(allb / ideal_fw).mean() * 100 - 100:+.3f} %  計算 {ms:.0f} ms")
    print(f"現行パターン 1(1 パターン):     分岐あり {(fw1 / ideal_fw).mean() * 100 - 100:+.3f} %  分岐なし {(fw1b / ideal_fw).mean() * 100 - 100:+.3f} %  計算 {r['ms'][0].sum():.0f} ms")
    print(f"現行 5 パターン:                分岐あり {(fw5 / ideal_fw).mean() * 100 - 100:+.3f} %                         計算 {r['ms'][:5].sum():.0f} ms")
