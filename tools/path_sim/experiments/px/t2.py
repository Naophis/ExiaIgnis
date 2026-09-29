"""実験 2 の確認: 表にしたパターン 1〜5 がファームと同じ結果か。緩和(relax)・分岐幅の効果。"""
import os
import sys, pickle, numpy as np
import ev, rs
PX2 = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "px2", "px_path")
def tab6(v):
    s = [v[0], v[1], v[1]] + [v[2]] * 13
    d = [v[3], v[4], v[4]] + [v[5]] * 13
    return s + d
def run(vps, **extra):
    res = ev.evaluate(vps, exe=PX2, extra=extra or None)
    C = len(ev.CASES)
    t = np.array([[ev.T(r) for r in res[ci]] for ci in range(C)]).T
    b = np.array([[r.get("base_time", np.inf) if r.get("ok") else np.inf for r in res[ci]] for ci in range(C)]).T
    ms = np.array([[r.get("ms", 0) for r in res[ci]] for ci in range(C)]).T
    return t, b, ms
if __name__ == "__main__":
    d = pickle.load(open("rs_a.pkl", "rb"))
    fwt = [tab6(rs.FWV[i]) for i in ev.FW]
    t, b, ms = run(fwt)
    print("表にした 1〜5 がファームと一致:", bool(np.allclose(t, d["t"][:5], atol=1e-6)), "合計", round(t.min(0).sum(), 3))
    best = d["t"].min(0)
    pickle.dump({"t": t, "b": b, "ms": ms}, open("t2_base.pkl", "wb"))
    for name, kw in [("relax=1", dict(relax=1)), ("margin 1.0", dict(margin=1.0)), ("margin 2.0", dict(margin=2.0)), ("margin 2.0 iters 10", dict(margin=2.0, iters=10)), ("margin 4.0 iters 10", dict(margin=4.0, iters=10))]:
        t2, b2, ms2 = run(fwt, **kw)
        v = t2.min(0); vb = b2.min(0)
        print(f"{name:22s} 5 パターン: 合計 {v.sum():.3f} s  既知最良比 {(v / best).mean() * 100 - 100:+.3f} %  最良より速い {int((v < best - 1e-6).sum())} 件  分岐なし {(vb / best).mean() * 100 - 100:+.3f} %  計算 {ms2.sum():.0f} ms")
        print("     パターン別(分岐あり / なし):", " ".join(f"{(t2[k] / best).mean() * 100 - 100:+.2f}/{(b2[k] / best).mean() * 100 - 100:+.2f}" for k in range(5)))
        pickle.dump({"t": t2, "b": b2, "ms": ms2}, open(f"t2_{name.replace(' ', '_')}.pkl", "wb"))
