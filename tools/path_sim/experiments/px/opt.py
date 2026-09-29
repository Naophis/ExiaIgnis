"""実験 3: タイム最小の経路(px_opt)と、ファームの経路・既知の最良との比較"""
import json, subprocess, pickle, sys, os
import numpy as np
from multiprocessing import Pool
import ev, ds, rs, t2
PX2 = os.path.dirname(t2.PX2)
def run_opt(ci, raw_paths=None, opt=True, files=None, mp=None, goals=None, count_final=True):
    mi, ex = ev.CASES[ci]; m = ev.MAZES[mi]
    inp = {"files": files or ev._files, "map": mp or ds.fw_map(m), "goals": goals or m["goals"], "exec": ex, "opt": opt, "count_final": count_final}
    if raw_paths is not None: inp["raw_paths"] = raw_paths
    r = subprocess.run([f"{PX2}/px_opt"], input=json.dumps(inp).encode(), capture_output=True, timeout=600)
    try: return json.loads(r.stdout)
    except Exception: return {"ok": False, "error": f"rc={r.returncode} {r.stderr[-300:]!r}"}
def job_fw(ci): return run_opt(ci, count_final=False)
def job(ci): return run_opt(ci)
if __name__ == "__main__":
    d = pickle.load(open("rs_a.pkl", "rb")); best = d["t"].min(0); fw = d["t"][:5].min(0)
    # 1) ファームが出した経路(素)を評価に通して、報告されたタイムと同じか。最後の直線を足した本当のタイムも出す
    res = ev.evaluate([t2.tab6(rs.FWV[i]) for i in ev.FW], exe=t2.PX2, want_path=True)
    nbad = 0; n = 0; fin = []
    fw_true = np.zeros(len(ev.CASES))
    for ci in range(len(ev.CASES)):
        raws = [{"s": r["raw_s"], "t": r["raw_t"]} for r in res[ci]]
        o = run_opt(ci, raws, opt=False)
        k = int(np.argmin([r["time"] for r in res[ci]]))
        for r, e in zip(res[ci], o["evals"]):
            n += 1
            if abs(e["fw_time"] - r["time"]) > 1e-4: nbad += 1; print("  評価の食い違い", ev.CASES[ci], r["id"], r["time"], e["fw_time"])
            fin.append(e["model_time"] - e["fw_time"])
        fw_true[ci] = o["evals"][k]["model_time"]
    print(f"ファームの経路 {n} 本: 評価の食い違い {nbad}。最後の直線(ファームが数えていない時間): 平均 {np.mean(fin) * 1000:.0f} ms 最小 {np.min(fin) * 1000:.0f} 最大 {np.max(fin) * 1000:.0f} ms")
    # 2) ファームと同じ数え方(最後の直線なし)での最小
    with Pool(22) as p: outs = p.map(job_fw, range(len(ev.CASES)))
    opt = np.array([o["opt"]["time"] if o.get("ok") and o["opt"]["found"] else np.inf for o in outs])
    fwt = np.array([o["opt"].get("fw_time", np.inf) if o.get("ok") else np.inf for o in outs])
    bad = [(ev.MAZES[ev.CASES[i][0]]["name"], ev.CASES[i][1], opt[i], fwt[i]) for i in range(len(outs)) if not abs(opt[i] - fwt[i]) < 1e-4]
    print("[ファームと同じ数え方] 探索のタイムとファームの計算が食い違う:", len(bad)); [print("  ", b) for b in bad[:10]]
    print(f"  合計: 現行 {fw.sum():.3f}  既知の最良 {best.sum():.3f}  タイム最小 {fwt.sum():.3f}")
    print(f"  現行はタイム最小より {(fw / fwt).mean() * 100 - 100:+.3f} % 遅い(最大 {(fw / fwt).max() * 100 - 100:+.2f} %)  既知の最良は {(best / fwt).mean() * 100 - 100:+.3f} %  探索のほうが既知の最良より遅いケース {int((fwt > best + 1e-4).sum())}")
    print("  計算 ms:", round(sum(o["opt"]["ms"] for o in outs)), " 節点", sum(o["opt"]["nodes"] for o in outs), " 辺", sum(o["opt"]["edges"] for o in outs))
    for i in np.argsort(-(fw - fwt))[:12]:
        print(f"    {ev.MAZES[ev.CASES[i][0]]['name']:18s} exec {ev.CASES[i][1]:2d}  現行 {fw[i]:.3f}  既知最良 {best[i]:.3f}  タイム最小 {fwt[i]:.3f}  差 {fw[i] - fwt[i]:+.3f} s ({(fw[i] / fwt[i] - 1) * 100:+.2f} %)")
    # 3) 最後の直線も数えた本当のタイムでの最小
    with Pool(22) as p: outs2 = p.map(job, range(len(ev.CASES)))
    opt2 = np.array([o["opt"]["time"] if o.get("ok") and o["opt"]["found"] else np.inf for o in outs2])
    mt2 = np.array([o["opt"].get("model_time", np.inf) if o.get("ok") else np.inf for o in outs2])
    bad = [(ev.MAZES[ev.CASES[i][0]]["name"], ev.CASES[i][1], opt2[i], mt2[i]) for i in range(len(outs2)) if not abs(opt2[i] - mt2[i]) < 1e-4]
    print("[最後の直線も数える] 探索のタイムと経路の再計算が食い違う:", len(bad)); [print("  ", b) for b in bad[:10]]
    print(f"  合計: 現行の経路 {fw_true.sum():.3f}  タイム最小 {mt2.sum():.3f}  差 {fw_true.sum() - mt2.sum():+.3f} s")
    print(f"  現行はタイム最小より {(fw_true / mt2).mean() * 100 - 100:+.3f} % 遅い(最大 {(fw_true / mt2).max() * 100 - 100:+.2f} %)  現行が最小と同じ {int((fw_true < mt2 + 1e-4).sum())}/{len(mt2)}")
    for i in np.argsort(-(fw_true - mt2))[:15]:
        print(f"    {ev.MAZES[ev.CASES[i][0]]['name']:18s} exec {ev.CASES[i][1]:2d}  現行 {fw_true[i]:.3f}  タイム最小 {mt2[i]:.3f}  差 {fw_true[i] - mt2[i]:+.3f} s ({(fw_true[i] / mt2[i] - 1) * 100:+.2f} %)")
    pickle.dump({"opt_fw": fwt, "outs_fw": outs, "opt_true": mt2, "outs_true": outs2, "fw_true": fw_true}, open("opt.pkl", "wb"))
