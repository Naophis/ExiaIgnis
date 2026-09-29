import sys, pickle, random
import numpy as np
from an import load, norm, greedy, swap_refine
d = load(sys.argv[1].split(","))
t = d["t"]; b = d["base"]; P, C = t.shape; cases = d["cases"]; ms = d["ms"]
best = t.min(0)
fw = [0, 1, 2, 3, 4]
def show(name, sel, m=t):
    v = m[sel].min(0)
    print(f"{name:28s} 合計 {v.sum():8.3f} s  最良比 {(v / best).mean() * 100 - 100:+.3f} %  最大 {(v / best).max() * 100 - 100:+.2f} %  計算 {ms[sel].sum():6.0f} ms")
print("=== 分岐の比較あり(ファームの「右」と同じ)")
show("現行 1-5", fw); show("1 と 4", [0, 3]); show("1 だけ", [0]); show("4 だけ", [3]); show("1,3,4", [0, 2, 3])
print("=== 分岐の比較なし(近似コストどおりの経路そのまま)")
show("現行 1-5", fw, b); show("1 と 4", [0, 3], b); show("1 だけ", [0], b); show("4 だけ", [3], b)
allc = list(range(C)); cand = list(range(P))
relb = b / best
for K in (1, 2, 3):
    sel = swap_refine(relb, cand, greedy(relb, cand, K, allc), allc)
    show(f"探索した最良 K={K}(分岐なし)", sel, b)
    for s in sel: print("      ", norm(d["vps"][s]))
mazes = sorted(set(c[0] for c in cases)); rng = random.Random(0); rng.shuffle(mazes)
folds = [mazes[i::5] for i in range(5)]
for K in (1, 2, 3):
    tot = 0; rs = 0
    for f in folds:
        te = [i for i, c in enumerate(cases) if c[0] in f]; tr = [i for i, c in enumerate(cases) if c[0] not in f]
        sel = swap_refine(relb, cand, greedy(relb, cand, K, tr), tr)
        m = b[sel][:, te].min(0); tot += m.sum(); rs += (m / best[te]).sum()
    print(f"  交差検証 K={K}(分岐なし): 合計 {tot:.3f} s  最良比 {rs / C * 100 - 100:+.3f} %")
# exec ごとの内訳
print("=== exec ごと(現行 5 パターン、最良比)")
for ex in sorted(set(c[1] for c in cases)):
    idx = [i for i, c in enumerate(cases) if c[1] == ex]
    v = t[fw][:, idx].min(0)
    print(f"  exec {ex:2d}: 合計 {v.sum():.3f} s  最良比 {(v / best[idx]).mean() * 100 - 100:+.3f} %  fw1 単独 {(t[0, idx] / best[idx]).mean() * 100 - 100:+.3f} %")
# 既知の最良と現行が違うケース
v = t[fw].min(0)
print("=== 現行が既知の最良より遅いケース")
for i in np.argsort(-(v - best)):
    if v[i] - best[i] < 1e-6: break
    print(f"  {cases[i][0]:18s} exec {cases[i][1]:2d}  現行 {v[i]:.3f}  最良 {best[i]:.3f}  差 {v[i] - best[i]:+.3f} s  ({(v[i] / best[i] - 1) * 100:+.2f} %)  最良を出したパターン数 {int((t[:, i] <= best[i] + 1e-6).sum())}/{P}")
