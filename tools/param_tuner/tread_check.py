#!/usr/bin/env python3
"""実効トレッドと左右タイヤ径差をログから同定するツール。

角速度はジャイロ(w_lp)、左右輪速はエンコーダ(v_l / v_r)から独立に取れるので、

    v_r - v_l = tread * w + (e_r - e_l) * v_c

を最小二乗で解けば tread [mm] と 左右タイヤ径の相対差 (e_r - e_l) が同時に出る。
(e_r - e_l) を入れないと、直進中でも出る左右差が旋回方向で逆符号の
オフセットになり、左ターンと右ターンで見かけのトレッドが食い違う。

使うサンプル:
  - 直進巡航   : ideal_w == 0 かつ accl == 0 (タイヤ径差を決める)
  - 旋回プラトー: alpha == 0 かつ |ideal_w| > w_min (トレッドを決める)
過渡(角加速中)は w_lp と輪速のフィルタ遅れが違うので使わない。

スリップが小さい低速(v=400 前後)のターンを左右両方向、各 n>=4 本まとめて渡すこと。
高速ターンは横滑りで実効トレッドが変わるので別物として見る。

使い方:
    python3 tools/param_tuner/tread_check.py logs/20260920_1[67]*.csv
"""
import sys

import numpy as np
import pandas as pd

W_MIN = 5.0  # [rad/s] これ未満のプラトーは分母が小さくノイズに埋もれる
V_MIN = 300.0


def main(files):
    frames = []
    for f in files:
        d = pd.read_csv(f)
        if "w_lp" not in d or "v_l" not in d:
            print(f"skip {f}: 必要な列がない")
            continue
        d["f"] = f
        frames.append(d)
    if not frames:
        return
    d = pd.concat(frames, ignore_index=True)
    d["dv"] = d["v_r"] - d["v_l"]
    d["vc"] = (d["v_r"] + d["v_l"]) / 2

    run = d["ideal_v"] > V_MIN
    straight = (run & (d["ideal_w"].abs() < 1e-6) & (d["accl"].abs() < 1)
                & (d["w_lp"].abs() < 0.5))
    plateau = run & (d["alpha"].abs() < 1) & (d["ideal_w"].abs() > W_MIN)
    left = plateau & (d["ideal_w"] > 0)
    right = plateau & (d["ideal_w"] < 0)
    print(f"samples: straight={straight.sum()} left={left.sum()} "
          f"right={right.sum()}")
    if left.sum() == 0 or right.sum() == 0:
        print("warning: 片方向のターンしかない。タイヤ径差とトレッドが分離できない")

    m = straight | plateau
    A = np.c_[d["w_lp"][m], d["vc"][m]]
    (tread, e_diff), *_ = np.linalg.lstsq(A, d["dv"][m], rcond=None)
    print(f"\npooled: tread = {tread:.2f} mm   (e_r - e_l) = {100 * e_diff:+.3f} %")
    print("  (e_r - e_l) < 0 : 共通の tire 値に対して右輪が遅く読めている"
          " = 右タイヤ実径が左より大きい")

    for name, mm in (("left ", left), ("right", right)):
        if mm.sum() == 0:
            continue
        w = d["w_lp"][mm]
        raw = (d["dv"][mm] @ w) / (w @ w)
        corr = ((d["dv"] - e_diff * d["vc"])[mm] @ w) / (w @ w)
        print(f"  {name} turn: tread = {corr:.2f} mm (径差補正なし {raw:.2f})")

    print("\nper file (プラトー平均):")
    per_file = []
    for f, g in d[plateau].groupby("f"):
        w = g["w_lp"].mean()
        t = (g["dv"] - e_diff * g["vc"]).mean() / w
        per_file.append(t)
        print(f"  {f}  w={w:+6.2f}  v_l={g['v_l'].mean():6.1f} "
              f"v_r={g['v_r'].mean():6.1f}  tread={t:.2f}")
    if len(per_file) > 1:
        print(f"  -> mean {np.mean(per_file):.2f}  std {np.std(per_file):.2f} "
              f"(n={len(per_file)})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1:])
