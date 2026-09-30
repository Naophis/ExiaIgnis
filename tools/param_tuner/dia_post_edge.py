#!/usr/bin/env python3
"""斜め走行の横位置と向きを、45°センサーが柱を過ぎて落ちる位置からログごとに出すツール。

斜めでは柱が左右交互に 63.64mm(= 90/√2)ごとに来る。45°センサー(LED1 の生値
left45 / right45)は柱を通り過ぎた瞬間に落ちるので、その位置(生値が thr を
切った点、tick 間は直線補間)を左右で並べる:

    右へ δ ずれると R45 が落ちる位置は +δ、L45 は −δ
    → 隣り合う L/R の間隔 g から  R→L: δ = (63.64 − g)/2   L→R: δ = (g − 63.64)/2

前後のずれ(旋回出口の位置・読んだ時刻・速度の遅れ)は左右に同じだけ入るので消える。
柱の反射の強さ(壁の有無で 5〜35mm 動く)も使わない。壁が柱に付いていると落ちる
位置は左右とも 1〜2mm 遅れるが、差では 0.2mm(2026-09-30、壁あり/なし 11 本)。

柱ごとの δ を並べて直線を引く:
    δ(p) = δ0 + tan(ε)·(p − p0)
  δ0  直線の始まり p0 での横位置 [mm]。+ は右
  ε   迷路の対角線に対する向き [deg]。+ は右向き。ジャイロには見えない量で、
      t_400.yaml の dia45 ang 44.75 のときに +0.24±0.07°(45−44.75)が出た
  res 直線からの残差 σ [mm](柱の置き位置・しきい値のノイズ。0.1〜0.3 が普通)

出す列:
  kind   斜めへ入った旋回: dia45 / dia135 / dia90(斜め → 斜め)、dir = L/R(ideal_w の符号)
  first  斜めで最初に見えた柱の側
  n      使った L/R の組の数
  d0 eps res   上のとおり
  dend   直線の終わりでの δ(直線の当てはめ)。旋回後どこまで戻ったか
  sp_l sp_r    同じ側の柱の間隔の平均 [mm](127.28 になるはず。走行距離の検算)
  wall_l wall_r  落ちた後 5〜15mm の生値の平均。柱に壁が付いていると高い
         (2026-09-30 の実測: 壁あり L 100〜130 / R 21〜24、なし L 57〜77 / R 10〜15)
  yawslp 直線中のジャイロ向きの変化 [deg/m](ε が直線の途中で変わっていないか)
  start  斜めへ抜ける旋回の開始位置(スタートからの走行距離)。δ0 はこれと一緒に見る:
         旋回開始が Δ 早いと出口は旋回の内側へ 0.71·Δ 寄る(45° / 135° とも sin = 0.71)

合わせ方(k0 は左右センサーの取り付け差。--k0 で引く):
  1. 同じ旋回を左右各 n>=4 本走らせ --summary で平均を見る(壁あり/なしは不要。
     2026-09-30 の dia45 左 8 本で壁あり −1.87 / なし −1.72)
  2. k0 = (δ0_左 + δ0_右)/2、旋回の内外のずれ = (δ0_左 − δ0_右)/2(内側が −)
  3. ang を ε=0 に(dia45 44.75→45.0 で ε +0.19→−0.05)、front(と壁切れ側の *_str)を
     内外のずれ 0 に合わせる。別々の量。45°/135° とも front 1mm → 横 0.71mm

限界:
  - 横位置は L と R の組から出すので柱 2 本以上の斜めだけ。ε は 3 本以上。実走の
    1 区画の斜め(柱 1 本)では経路方向の位置しか取れない
  - 走行距離はエンコーダー(dist)。4000mm/s 超の加減速では空転・ロックで同じ側の
    間隔が 116〜132mm になり(20260924_010340)、δ が ±3mm 狂う。sp_l / sp_r が
    127.3 から 2mm 以上外れた走行は使わない
  - 縁は tick ごとの S0 の生値(2200mm/s で 2.2mm 刻み)を補間。hf_mode の S1〜S3 を
    使えば 3 倍細かくなる(未対応)

使い方:
    python3 tools/param_tuner/dia_post_edge.py logs/20260930_22*.csv
    python3 tools/param_tuner/dia_post_edge.py --summary --k0 -1.2 logs/*.csv
    python3 tools/param_tuner/dia_post_edge.py -v logs/20260930_225043.csv   # 縁と組を全部出す
"""
import argparse
import math
import sys

import numpy as np
import pandas as pd

PITCH = 90.0 / math.sqrt(2)  # 柱の間隔(経路方向)[mm]
MS_STRAIGHT = 1
MS_SLALOM = 4
MS_WALL_OFF_DIA = 13
MS_DIA_RUN = (MS_STRAIGHT, 3, MS_WALL_OFF_DIA, 14)  # 3 = SLA_FRONT_STR, 14 = SLA_BACK_STR


def segments(ms):
    """motion_state の連続区間 [(state, i0, i1)] を返す(i1 は含む)。"""
    out = []
    s = 0
    for i in range(1, len(ms) + 1):
        if i == len(ms) or ms[i] != ms[s]:
            out.append((int(ms[s]), s, i - 1))
            s = i
    return out


def falling_edges(x, y, thr, n_above=3, peak_ratio=1.5):
    """生値 y が thr を上から下へ切った位置(x を直線補間)。
    手前 n_above サンプルが thr 以上で、直前 30mm の山が thr の peak_ratio 倍以上の
    ものだけ(停止前の上昇中の 1 サンプルの落ち込みや弱い反射を弾く)。"""
    out = []
    for i in range(n_above, len(y)):
        if y[i - 1] >= thr > y[i] and (y[i - n_above:i] >= thr).all():
            back = (x >= x[i] - 30) & (x < x[i])
            if y[back].max() < thr * peak_ratio:
                continue
            p = x[i - 1] + (x[i] - x[i - 1]) * (y[i - 1] - thr) / (y[i - 1] - y[i])
            # 落ちた後 5〜15mm の平均(壁が付いていると高いまま)
            j = (x > p + 5) & (x < p + 15)
            tail = float(y[j].mean()) if j.any() else float("nan")
            out.append((p, tail))
    return out


def gate_spacing(edges, tol=15.0):
    """同じ側の縁は PITCH*2 の整数倍離れているはず。外れたものを捨てる。"""
    kept = []
    for p, tail in edges:
        if kept:
            g = p - kept[-1][0]
            k = round(g / (2 * PITCH))
            if k < 1 or abs(g - k * 2 * PITCH) > tol:
                continue
        kept.append((p, tail))
    return kept


def pair_deltas(R, L, tol=15.0):
    """隣り合う L/R の縁から δ(+右)と組の位置を出す。"""
    ev = sorted([(p, "R") for p, _ in R] + [(p, "L") for p, _ in L])
    out = []
    for (pa, sa), (pb, sb) in zip(ev, ev[1:]):
        if sa == sb:
            continue
        g = pb - pa
        if abs(g - PITCH) > tol:
            continue
        d = (PITCH - g) / 2 if sa == "R" else (g - PITCH) / 2
        out.append((d, (pa + pb) / 2))
    return out, (ev[0][1] if ev else "-")


def analyze(path, thr, k0, verbose):
    d = pd.read_csv(path)
    need = ["motion_state", "dist", "left45", "right45", "ideal_ang", "ideal_w", "ang"]
    if any(c not in d for c in need):
        print(f"skip {path}: 必要な列がない")
        return []
    ms = d["motion_state"].values
    segs = segments(ms)
    rows = []
    diag = False
    run_dist = 0.0  # スタートからの走行距離(区間ごとの dist を積む)
    for si, (st, i0, i1) in enumerate(segs):
        seg_len = float(d["dist"].iloc[i1]) if st != 0 else 0.0
        if st == MS_SLALOM:
            a = abs(float(d["ideal_ang"].iloc[i1]))
            is_dia_turn = 40 < a < 50 or 130 < a < 140
            is_dia90 = diag and 80 < a < 100  # 斜め → 斜め
            if is_dia_turn or is_dia90:
                if is_dia_turn:
                    kind = "dia45" if a < 90 else "dia135"
                    diag = not diag
                else:
                    kind = "dia90"
                if diag:
                    turn_start = run_dist
                    turn_dir = "L" if d["ideal_w"].iloc[i0:i1 + 1].mean() > 0 else "R"
                    # 旋回の直後から、斜めの走行区間(1 / 13 / 14 の連続)を 1 本にまとめる
                    j = si + 1
                    idx = []
                    while j < len(segs) and segs[j][0] in MS_DIA_RUN:
                        idx.append(segs[j])
                        j += 1
                    if not idx:
                        run_dist += seg_len
                        continue
                    rows.append(_analyze_run(path, d, idx, kind, turn_dir, turn_start, thr, k0, verbose))
            run_dist += seg_len
        else:
            run_dist += seg_len
    return [r for r in rows if r]


def _analyze_run(path, d, idx, kind, turn_dir, turn_start, thr, k0, verbose):
    # 区間をつないで経路方向の位置 x を作る(各区間の dist は 0 から)
    xs, cols = [], {c: [] for c in ("left45", "right45", "ang", "ideal_v")}
    base = 0.0
    for st, i0, i1 in idx:
        q = d.iloc[i0:i1 + 1]
        xs.append(q["dist"].values + base)
        for c in cols:
            cols[c].append(q[c].values.astype(float))
        base += float(q["dist"].iloc[-1])
    x = np.concatenate(xs)
    L45 = np.concatenate(cols["left45"])
    R45 = np.concatenate(cols["right45"])
    ang = np.concatenate(cols["ang"])
    v = np.concatenate(cols["ideal_v"])
    R = gate_spacing(falling_edges(x, R45, thr))
    L = gate_spacing(falling_edges(x, L45, thr))
    pairs, first = pair_deltas(R, L)
    if len(pairs) < 2:
        print(f"{path}: {kind} {turn_dir} 組が {len(pairs)} 個しかない(R {len(R)} L {len(L)})")
        return None
    D = np.array([p[0] for p in pairs]) - k0
    P = np.array([p[1] for p in pairs])
    slope, d0 = np.polyfit(P, D, 1)
    res = float(np.std(D - (d0 + slope * P), ddof=2)) if len(D) > 2 else float("nan")
    eps = math.degrees(math.atan(slope))
    dend = d0 + slope * x[-1]
    sp = lambda E: float(np.mean(np.diff([p for p, _ in E]))) if len(E) > 1 else float("nan")
    tail = lambda E: float(np.nanmean([t for _, t in E])) if E else float("nan")
    m = (x > 20) & (x < x[-1] - 20)
    yawslp = float(np.polyfit(x[m], ang[m], 1)[0] * 1000) if m.sum() > 10 else float("nan")
    row = dict(file=path.split("/")[-1], kind=kind, dir=turn_dir, first=first, n=len(pairs),
               d0=d0, eps=eps, res=res, dend=dend, sp_l=sp(L), sp_r=sp(R),
               wall_l=tail(L), wall_r=tail(R), yawslp=yawslp, start=turn_start,
               v=float(v.max()))
    if verbose:
        print(f"--- {row['file']} {kind} {turn_dir}  v {row['v']:.0f}")
        print("  R edges: " + "  ".join(f"{p:6.1f}(tail {t:3.0f})" for p, t in R))
        print("  L edges: " + "  ".join(f"{p:6.1f}(tail {t:3.0f})" for p, t in L))
        print("  pairs (pos, δ): " + "  ".join(f"{p:6.1f}:{dd:+5.2f}" for dd, p in zip(D, P)))
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("files", nargs="+")
    ap.add_argument("--thr", type=float, default=250, help="落ちたと見る生値のしきい値(既定 250)")
    ap.add_argument("--k0", type=float, default=0.0, help="左右センサーの取り付け差 [mm]。δ から引く")
    ap.add_argument("--summary", action="store_true", help="kind/dir ごとの平均と σ")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    rows = []
    for f in a.files:
        rows += analyze(f, a.thr, a.k0, a.verbose)
    if not rows:
        return
    df = pd.DataFrame(rows)
    pd.set_option("display.width", 250)
    fmt = {c: "{:+.2f}".format for c in ("d0", "eps", "dend", "yawslp")}
    fmt.update({c: "{:.2f}".format for c in ("res", "sp_l", "sp_r")})
    fmt.update({c: "{:.0f}".format for c in ("wall_l", "wall_r", "start", "v")})
    print(df.to_string(index=False, formatters=fmt))
    if a.summary:
        print()
        g = df.groupby(["kind", "dir"])
        s = g.agg(n=("d0", "size"), d0=("d0", "mean"), d0_sd=("d0", "std"), eps=("eps", "mean"),
                  eps_sd=("eps", "std"), res=("res", "mean"), dend=("dend", "mean"))
        print(s.round(2).to_string())
        if len(s) >= 2:
            print(f"\nk0 の目安(全体の d0 の平均): {df.d0.mean():+.2f} mm  "
                  f"(左右同数・同じ旋回のときだけ意味がある)")


if __name__ == "__main__":
    main()
