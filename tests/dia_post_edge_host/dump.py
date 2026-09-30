#!/usr/bin/env python3
"""ログから DiaPostEdgeDetector の入力を作り、C++ 版と Python 版(tools/param_tuner/dia_post_edge.py)の δ を比べる。

  mode diag: dia_post_edge.py と同じ斜めの区間(旋回直後の 1/3/13/14 の連続、dist をつなぐ)だけを流す。
             ジャイロの向きの積分 c(−ang を dist で台形積分)も流し、ψ0 と今の横位置の推測を
             Python の同じ計算と比べる
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


KAPPA = float(os.environ.get("KAPPA", "0"))


def main():
    mode, files = sys.argv[1], sys.argv[2:]
    lines = []
    ref = []
    for f in files:
        name = os.path.basename(f)[:-4]
        d = pd.read_csv(f)
        if mode in ("diag", "pred"):
            for ri, idx in enumerate(diag_runs(d)):
                cid = f"{name}#{ri}"
                bases = np.cumsum([0.0] + [float(d["dist"].iloc[i1]) for _, _, i1 in idx[:-1]])
                x = np.concatenate([d.iloc[i0:i1 + 1]["dist"].values + b for (_, i0, i1), b in zip(idx, bases)])
                L45 = np.concatenate([d.iloc[i0:i1 + 1]["left45"].values for _, i0, i1 in idx]).astype(float)
                R45 = np.concatenate([d.iloc[i0:i1 + 1]["right45"].values for _, i0, i1 in idx]).astype(float)
                psi = -np.radians(np.concatenate([d.iloc[i0:i1 + 1]["ang"].values for _, i0, i1 in idx]))
                acl = np.concatenate([d.iloc[i0:i1 + 1]["accl"].values for _, i0, i1 in idx])
                c = np.concatenate([[0.0], np.cumsum(0.5 * (psi[1:] + psi[:-1]) * np.diff(x))])
                for xi, yl, yr, ci, pi, ai in zip(x, L45, R45, c, psi, acl):
                    lines.append(f"{cid} 0 {xi:.4f} {int(yl)} {int(yr)} {ci:.6f} {pi:.7f} {ai:.1f}")
                # Python 版の δ と ψ0(組の中点でのジャイロの向きの積分から)
                R = T.gate_spacing(T.falling_edges(x, R45, 250))
                L = T.gate_spacing(T.falling_edges(x, L45, 250))
                ev = sorted([(p, "R") for p, _ in R] + [(p, "L") for p, _ in L])
                # ファームは縁を採った tick(縁の直後のサンプル)の加速度を添える
                acc_ev = [float(acl[np.searchsorted(x, p)]) if np.searchsorted(x, p) < len(acl) else float(acl[-1]) for p, _ in ev]
                psi0 = 0.0
                n = 0
                prev = None
                lsx, lsy = [], []
                for i in range(1, len(ev)):
                    (pa, sa), (pb, sb) = ev[i - 1], ev[i]
                    if sa == sb or abs(pb - pa - T.PITCH) > 15:
                        continue
                    sc, used = T.local_scale_used(ev, i)
                    if not T.confident(acc_ev, i, used):
                        continue
                    gg = (pb - pa) / sc
                    dd = (T.PITCH - gg) / 2 if sa == "R" else (gg - T.PITCH) / 2
                    pp = (pa + pb) / 2
                    cp = 0.5 * (np.interp(pa, x, c) + np.interp(pb, x, c))
                    da = dd - KAPPA * 0.5 * (np.interp(pa, x, psi) + np.interp(pb, x, psi))
                    lsx.append(pp)
                    lsy.append(da - cp)
                    if len(lsx) >= 2:
                        psi0 = np.polyfit(np.array(lsx) - lsx[0], np.array(lsy), 1)[0]
                        n = len(lsx) - 1
                    prev = (da, pp, cp)
                    ref.append((cid, dd, pp, np.degrees(psi0), n))
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
                             f"{d['right45'].iloc[i]} 0 0 {d['accl'].iloc[i]:.1f} {int(in_diag[i])}")
    exe = "/tmp/dia_post_edge_host_test"
    args = [exe, str(KAPPA)] + (["pred"] if mode == "pred" else [])
    if mode == "pred":
        mode_run = "diag"
    out = subprocess.run(args, input="\n".join(" ".join(l.split()[:8]) for l in lines) + "\n",
                         capture_output=True, text=True, check=True).stdout.split("\n")
    got = [o.split() for o in out if o.strip()]
    if mode == "pred":
        # 次の組が来た tick の直前の推測と、来た組の車軸の横位置の差
        res = []
        pend = {}
        for g in got:
            if g[0] == "PRED":
                pend[g[1]] = float(g[3])
            elif g[0] in pend:
                res.append(float(g[9]) - pend.pop(g[0]))
        res = np.array(res)
        print(f"κ={KAPPA:.0f}: 推測の誤差(次の組 − 直前の推測) n={len(res)} 平均 {res.mean():+.2f} rms {np.sqrt((res**2).mean()):.2f} mm")
        psi = {}
        for g in got:
            if g[0] != "PRED":
                psi[g[0]] = float(g[6])
        print("   最後の ψ0 [deg]: " + " ".join(f"{k.split('#')[0][-6:]} {v:+.2f}" for k, v in psi.items()))
        return
    if mode == "diag":
        by = {}
        for g in got:
            by.setdefault(g[0], []).append((float(g[2]), float(g[3]), float(g[6]), int(g[7])))
        rb = {}
        for cid, dd, pp, ps, n in ref:
            rb.setdefault(cid, []).append((dd, pp, ps, n))
        n_ok = n_all = 0
        worst = worst_psi = 0.0
        for cid in sorted(set(by) | set(rb)):
            a, b = by.get(cid, []), rb.get(cid, [])
            same = len(a) == len(b) and all(
                abs(x[0] - y[0]) < 0.01 and abs(x[1] - y[1]) < 0.02 and abs(x[2] - y[2]) < 0.005 and x[3] == y[3]
                for x, y in zip(a, b))
            n_all += 1
            n_ok += same
            if len(a) == len(b):
                worst = max([worst] + [abs(x[0] - y[0]) for x, y in zip(a, b)])
                worst_psi = max([worst_psi] + [abs(x[2] - y[2]) for x, y in zip(a, b)])
            if not same:
                print(f"DIFF {cid}: C++ {[(round(x[0], 2), round(x[1], 1), round(x[2], 3)) for x in a]}  "
                      f"py {[(round(y[0], 2), round(y[1], 1), round(y[2], 3)) for y in b]}")
        print(f"diag runs {n_all}: 一致 {n_ok}  (δ の差の最大 {worst:.4f} mm、ψ0 の差の最大 {worst_psi:.4f} deg)")
    else:
        # 組ができた tick が斜めの区間の中かを数える(組の位置 pos の行で判定)
        rows = [l.split() for l in lines]
        xs = {}
        for r in rows:
            xs.setdefault(r[0], []).append((float(r[2]), int(r[8])))
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
