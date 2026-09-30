#!/usr/bin/env python3
"""ログから DiaPostEdgeDetector の入力を作り、C++ 版と Python 版(tools/param_tuner/dia_post_edge.py)の δ を比べる。

  mode diag: dia_post_edge.py と同じ斜めの区間(旋回直後の 1/3/13/14 の連続、dist をつなぐ)だけを流す
  mode full: ログ全体を流す(旋回・停止中は再アーム、x は区間の dist をつないだもの)。
             斜めの区間の外でできた組を数える(直交の直線で余計な組ができないか)
使い方: python3 dump.py diag|full logs/*.csv
"""
import os
import subprocess
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "../../tools/param_tuner"))
import dia_post_edge as T  # noqa: E402

REARM = {0, 2, 4, 7, 8, 9, 10, 11, 12}  # NONE PIVOT SLALOM READY PIVOT_* FRONT_CTRL


def diag_runs(d):
    ms = d["motion_state"].values
    segs = T.segments(ms)
    out = []
    diag = False
    for si, (st, i0, i1) in enumerate(segs):
        if st != T.MS_SLALOM:
            continue
        a = abs(float(d["ideal_ang"].iloc[i1]))
        if 40 < a < 50 or 130 < a < 140:
            diag = not diag
        elif not (diag and 80 < a < 100):  # dia90(斜め → 斜め)の後も斜め
            continue
        if not diag:
            continue
        j = si + 1
        idx = []
        while j < len(segs) and segs[j][0] in T.MS_DIA_RUN:
            idx.append(segs[j])
            j += 1
        if idx:
            out.append(idx)
    return out


def main():
    mode, files = sys.argv[1], sys.argv[2:]
    lines = []
    ref = []
    for f in files:
        name = os.path.basename(f)[:-4]
        d = pd.read_csv(f)
        if mode == "diag":
            for ri, idx in enumerate(diag_runs(d)):
                cid = f"{name}#{ri}"
                base = 0.0
                for st, i0, i1 in idx:
                    q = d.iloc[i0:i1 + 1]
                    for x, yl, yr in zip(q["dist"].values + base, q["left45"].values, q["right45"].values):
                        lines.append(f"{cid} 0 {x:.4f} {yl} {yr}")
                    base += float(q["dist"].iloc[-1])
                # Python 版の δ
                xs = np.concatenate([d.iloc[i0:i1 + 1]["dist"].values for _, i0, i1 in idx])
                bases = np.cumsum([0.0] + [float(d["dist"].iloc[i1]) for _, _, i1 in idx[:-1]])
                x = np.concatenate([d.iloc[i0:i1 + 1]["dist"].values + b for (_, i0, i1), b in zip(idx, bases)])
                L45 = np.concatenate([d.iloc[i0:i1 + 1]["left45"].values for _, i0, i1 in idx]).astype(float)
                R45 = np.concatenate([d.iloc[i0:i1 + 1]["right45"].values for _, i0, i1 in idx]).astype(float)
                R = T.gate_spacing(T.falling_edges(x, R45, 250))
                L = T.gate_spacing(T.falling_edges(x, L45, 250))
                pairs, _ = T.pair_deltas(R, L)
                ref += [(cid, dd, pp) for dd, pp in pairs]
        else:
            ms = d["motion_state"].values
            base = 0.0
            prev = None
            in_diag = np.zeros(len(d), bool)
            for idx in diag_runs(d):
                in_diag[idx[0][1]:idx[-1][2] + 1] = True
            for i in range(len(d)):
                if prev is not None and ms[i] != ms[i - 1]:
                    base += float(d["dist"].iloc[i - 1])
                prev = ms[i]
                x = base + float(d["dist"].iloc[i])
                lines.append(f"{name} {1 if ms[i] in REARM else 0} {x:.4f} {d['left45'].iloc[i]} "
                             f"{d['right45'].iloc[i]} {int(in_diag[i])}")
    exe = "/tmp/dia_post_edge_host_test"
    out = subprocess.run([exe], input="\n".join(" ".join(l.split()[:5]) for l in lines) + "\n",
                         capture_output=True, text=True, check=True).stdout.split("\n")
    got = [o.split() for o in out if o.strip()]
    if mode == "diag":
        by = {}
        for g in got:
            by.setdefault(g[0], []).append((float(g[2]), float(g[3])))
        rb = {}
        for cid, dd, pp in ref:
            rb.setdefault(cid, []).append((dd, pp))
        n_ok = n_all = 0
        worst = 0.0
        for cid in sorted(set(by) | set(rb)):
            a, b = by.get(cid, []), rb.get(cid, [])
            same = len(a) == len(b) and all(abs(x[0] - y[0]) < 0.01 and abs(x[1] - y[1]) < 0.02 for x, y in zip(a, b))
            n_all += 1
            n_ok += same
            if len(a) == len(b):
                worst = max([worst] + [abs(x[0] - y[0]) for x, y in zip(a, b)])
            if not same:
                print(f"DIFF {cid}: C++ {[(round(x, 2), round(y, 1)) for x, y in a]}  py {[(round(x, 2), round(y, 1)) for x, y in b]}")
        print(f"diag runs {n_all}: 一致 {n_ok}  (δ の差の最大 {worst:.4f} mm)")
    else:
        # 組ができた tick が斜めの区間の中かを数える(組の位置 pos の行で判定)
        rows = [l.split() for l in lines]
        xs = {}
        for r in rows:
            xs.setdefault(r[0], []).append((float(r[2]), int(r[5])))
        n_in = n_out = 0
        for g in got:
            arr = xs[g[0]]
            pos = float(g[3])
            k = min(range(len(arr)), key=lambda i: abs(arr[i][0] - pos))
            if arr[k][1]:
                n_in += 1
            else:
                n_out += 1
                print(f"outside diag: {g[0]} pos {pos:.1f} δ {float(g[2]):+.2f}")
        print(f"pairs: 斜めの中 {n_in} / 外 {n_out}")


if __name__ == "__main__":
    main()
