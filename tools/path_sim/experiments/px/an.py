import sys, pickle, itertools, random
import numpy as np
np.set_printoptions(linewidth=200, precision=3, suppress=True)
def load(tags):
    ds = [pickle.load(open(f"rs_{t}.pkl", "rb")) for t in tags]
    d = {k: np.concatenate([x[k] for x in ds]) for k in ("t", "ms", "base", "sig", "other")}
    d["vps"] = sum((x["vps"] for x in ds), []); d["cases"] = ds[0]["cases"]
    return d
def norm(v): return [round(x / v[0], 4) for x in v]
def greedy(t, cand, K, cols, start=()):
    """cols の合計(相対タイム)を最小にする K 個を貪欲に選ぶ"""
    sel = list(start); cur = np.full(len(cols), np.inf) if not sel else t[sel][:, cols].min(0)
    while len(sel) < K:
        tot = np.minimum(t[cand][:, cols], cur).sum(1)
        b = cand[int(np.argmin(tot))]; sel.append(b); cur = np.minimum(cur, t[b, cols])
    return sel
def swap_refine(t, cand, sel, cols, rounds=3):
    sel = list(sel)
    for _ in range(rounds):
        changed = False
        for i in range(len(sel)):
            rest = [s for j, s in enumerate(sel) if j != i]
            cur = t[rest][:, cols].min(0) if rest else np.full(len(cols), np.inf)
            tot = np.minimum(t[cand][:, cols], cur).sum(1)
            b = cand[int(np.argmin(tot))]
            if tot.min() < np.minimum(t[sel[i], cols], cur).sum() - 1e-9: sel[i] = b; changed = True
        if not changed: break
    return sel
if __name__ == "__main__":
    d = load(sys.argv[1].split(","))
    t = d["t"]; P, C = t.shape; cases = d["cases"]
    best = t.min(0)
    rel = t / best  # 相対タイム(1 = 既知の最良)
    fw = [0, 1, 2, 3, 4]
    print(f"patterns {P} cases {C}  既知の最良の合計 {best.sum():.3f} s")
    fwmin = t[fw].min(0)
    print(f"現行 5 パターン: 合計 {fwmin.sum():.3f} s  既知の最良との差 {fwmin.sum() - best.sum():+.3f} s ({(fwmin / best).mean() * 100 - 100:+.3f} %)  最良と一致 {int((fwmin <= best + 1e-6).sum())}/{C}  計算 {d['ms'][fw].sum():.0f} ms")
    for i in fw:
        others = [j for j in fw if j != i]
        uniq = int((t[i] < t[others].min(0) - 1e-6).sum())
        print(f"  fw{i + 1}: 単独 {t[i].sum():.3f} s ({rel[i].mean() * 100 - 100:+.2f} %)  5 個の中で単独最速 {uniq} 件  これを外すと {t[others].min(0).sum() - fwmin.sum():+.3f} s  計算 {d['ms'][i].sum():.0f} ms  値 {norm(d['vps'][i])}")
    allc = list(range(C)); cand = list(range(P))
    print("--- 全データで選んだ最良の組(相対タイムの和を最小化)")
    for K in (1, 2, 3, 4, 5):
        sel = swap_refine(rel, cand, greedy(rel, cand, K, allc), allc)
        m = t[sel].min(0)
        print(f"K={K}: 合計 {m.sum():.3f} s ({(m / best).mean() * 100 - 100:+.3f} %)  現行比 {m.sum() - fwmin.sum():+.3f} s  現行より速い {int((m < fwmin - 1e-6).sum())} / 遅い {int((m > fwmin + 1e-6).sum())}  計算 {d['ms'][sel].sum():.0f} ms")
        for s in sel: print("     ", "fw%d" % (s + 1) if s < 5 else "r%d" % s, norm(d["vps"][s]))
    # 迷路ごとの交差検証
    mazes = sorted(set(c[0] for c in cases)); rng = random.Random(0); rng.shuffle(mazes)
    F = 5; folds = [mazes[i::F] for i in range(F)]
    print("--- 交差検証(迷路で 5 分割: 4 つで選び、残りで測る)")
    for K in (1, 2, 3, 4, 5):
        tot = 0; totfw = 0; totbest = 0; relsum = 0; n = 0
        for f in folds:
            te = [i for i, c in enumerate(cases) if c[0] in f]; tr = [i for i, c in enumerate(cases) if c[0] not in f]
            sel = swap_refine(rel, cand, greedy(rel, cand, K, tr), tr)
            m = t[sel][:, te].min(0); tot += m.sum(); totfw += fwmin[te].sum(); totbest += best[te].sum(); relsum += (m / best[te]).sum(); n += len(te)
        print(f"K={K}: テスト合計 {tot:.3f} s ({relsum / n * 100 - 100:+.3f} %)  現行 {totfw:.3f} s  差 {tot - totfw:+.3f} s")
