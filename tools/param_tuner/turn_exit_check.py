#!/usr/bin/env python3
"""旋回の出口残差(横ずれ・ヨー)と旋回中の追従状態をログから旋回ごとに出すツール。

「ターン直後に duty_sen(壁PD)が大きい」を旋回調整の指標にすると、値が
実際のずれを 3〜4 倍に誇張する:
  - 非探索の壁誤差は片壁だと ×2 (exist 44.75 ≒ ref 45 なのでほぼ常に片壁扱い)。
    str_ang_pid_fast.p=0.0375rad/mm なら真の横ずれ 1mm ≒ 4.3°。
  - ヨー 1° は 45°センサーで約 LAT_K=0.96mm の見かけ横ずれになる。
  - 出口直後の数 tick は柱・壁端を見ていて読み値が安定しない。
そこで duty_sen ではなく、旋回後の壁読み値から出口残差を直接出す。

旋回ごとに出す列:
  kind     旋回種別。角度と「斜め区間にいるか」から決める。斜めかどうかは
           走行開始(直線)から 45°/135° の旋回を数えてトグルで追う。
           dia45 / dia135 = 直線→斜め、dia45_2 / dia135_2 = 斜め→直線、
           dia90 = 斜め→斜め、large90 / orval180 = 直線→直線(v<600 は normal)。
  wmax     |ideal_w| 最大 [rad/s]、latg = v*wmax/g [G]
  w_over / w_under  プラトー域(|ideal_w| > 最大の半分)での w_lp−ideal_w の最大/最小
  vc_min   旋回中の v_c 最小 / ideal_v。接触・スリップで大きく落ちる
  v_in     旋回中の内輪速度最小 / ideal_v。r=38mm 級でも 0.4 程度は残る。
           0.1 未満は内輪が止まっている(壁・柱への接触)
  sat      旋回中に |duty| > 99 だった tick 数(どちらかの輪)
  lag      旋回最終 tick の ideal_ang − ang [deg]
  yaw0     旋回後 2tick 目の kim_theta [deg](出口向き基準にリセット後の値)
  off0     旋回後、最初に両側 45° が 1〜90mm に入った tick の (l45−r45)/2 [mm]。+は右。
           斜めへ抜ける旋回(dia45/dia135/dia90)は柱を見るので off 系は空欄
  off/yaw  旋回後 25〜50mm 走った区間の (l45−r45)/2 の平均と kim_theta 平均
  off_c    off − LAT_K*yaw (ヨーの見かけ分を引いた横ずれ)
  wide     off_c を旋回の外側正に直したもの [mm]。+なら外側(大回り)、−なら内側
  dsen40   旋回後 40tick の |duty_sen| 最大 [deg]、ey40 = 同 |s_pid_p| 最大 [mm]

幾何感度(出口横ずれ、外側正、旋回角 θ):
    Δlat ≈ Δrad*(1−cosθ) + Δfront*sinθ
    180°: rad 1mm → 2.0mm、front → 0
    135°: rad 1mm → 1.7mm、front 1mm → 0.71mm
     90°: rad 1mm → 1.0mm、front 1mm → 1.0mm
     45°: rad 1mm → 0.29mm、front 1mm → 0.71mm
back は出口位置を横に動かさない(直線側で吸収される)。

同じ旋回種別を n>=4 本そろえて --summary で平均と σ を見ること。vc_min / v_in /
sat が悪い旋回はオフセットより先に接触・飽和を解消する(出口が再現しない)。

使い方:
    python3 tools/param_tuner/turn_exit_check.py logs/20260923_01*.csv
    python3 tools/param_tuner/turn_exit_check.py logs/*.csv --summary --kind dia135_2
    python3 tools/param_tuner/turn_exit_check.py logs/20260923_012842.csv --dump 532
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

D = 180.0 / np.pi
G = 9806.0  # [mm/s^2]
LAT_K = 0.96  # [mm/deg] hardware.yaml start_align.lat_k の実測値
DT = 0.001

SLALOM, SLA_B = 4, 14
NEED = ["motion_state", "ideal_v", "v_c", "v_l", "v_r", "ideal_w", "w_lp",
        "ideal_ang", "ang", "kim_theta", "left45_d", "right45_d",
        "duty_l", "duty_r", "duty_sen", "s_pid_p"]

SENS = {45: (1 - np.cos(np.pi / 4), np.sin(np.pi / 4)),
        90: (1.0, 1.0),
        135: (1 - np.cos(3 * np.pi / 4), np.sin(3 * np.pi / 4)),
        180: (2.0, 0.0)}


def segments(ms):
    """motion_state の連続区間 [(start, end_exclusive, state), ...]"""
    out = []
    s = 0
    n = len(ms)
    for i in range(1, n + 1):
        if i == n or ms[i] != ms[s]:
            out.append((s, i, ms[s]))
            s = i
    return out


def classify(ang_deg, diag, v):
    a = int(round(abs(ang_deg)))
    if a == 45:
        return "dia45_2" if diag else "dia45", not diag
    if a == 135:
        return "dia135_2" if diag else "dia135", not diag
    if a == 90:
        if diag:
            return "dia90", diag
        return ("normal90" if v < 600 else "large90"), diag
    if a == 180:
        return "orval180", diag
    return f"turn{a}", diag


def analyze(df, name, post_ticks, lat_k):
    ms = df.motion_state.values
    n = len(ms)
    rows = []
    diag = False
    for s, e, st in segments(ms):
        if st != SLALOM or e - s < 5:
            continue
        seg = df.iloc[s:e]
        w = seg.ideal_w.values
        ang = w.sum() * DT * D
        v = float(seg.ideal_v.iloc[0])
        kind, diag = classify(ang, diag, v)
        exit_diag = diag  # 斜めへ抜ける旋回は出口で 45° が柱を見るので横ずれは出さない
        left = ang > 0
        wmax = float(np.abs(w).max())
        plateau = seg[np.abs(seg.ideal_w) > 0.5 * wmax]
        dw = np.abs(plateau.w_lp) - np.abs(plateau.ideal_w)
        inner = seg.v_l if left else seg.v_r
        sat = int(((seg.duty_l.abs() > 99) | (seg.duty_r.abs() > 99)).sum())
        lag = float(df.ideal_ang.values[e - 1] - df.ang.values[e - 1])

        post = df.iloc[e:min(e + max(post_ticks, 60), n)]
        post = post[post.motion_state != SLALOM]  # 次の旋回に入ったら打ち切り
        yaw0 = float(post.kim_theta.values[1]) if len(post) > 1 else np.nan
        both = ((post.left45_d > 1) & (post.left45_d < 90)
                & (post.right45_d > 1) & (post.right45_d < 90))
        fb = post[both]
        off0 = float((fb.left45_d.values[0] - fb.right45_d.values[0]) / 2) if len(fb) else np.nan
        travel = np.cumsum(post.ideal_v.values * DT)
        win = post[(travel >= 25) & (travel <= 50) & both.values
                   & (post.left45_d < 60).values & (post.right45_d < 60).values]
        if exit_diag:
            off0 = np.nan
        if len(win) >= 3 and not exit_diag:
            off = float(((win.left45_d - win.right45_d) / 2).mean())
            yaw = float(win.kim_theta.mean())
            off_c = off - lat_k * yaw
            wide = off_c if left else -off_c
        else:
            off = yaw = off_c = wide = np.nan
        p40 = post.iloc[:post_ticks]
        rows.append(dict(
            log=name, idx=s, kind=kind, dir="L" if left else "R", v=int(v),
            wmax=round(wmax, 1), latg=round(v * wmax / G, 1),
            w_over=round(float(dw.max()), 1), w_under=round(float(dw.min()), 1),
            vc_min=round(float(seg.v_c.min() / v), 2),
            v_in=round(float(inner.min() / v), 2), sat=sat,
            lag=round(lag, 1), yaw0=round(yaw0, 1), off0=round(off0, 1),
            off=round(off, 1), yaw=round(yaw, 1), off_c=round(off_c, 1),
            wide=round(wide, 1),
            dsen40=round(float(p40.duty_sen.abs().max() * D), 0),
            ey40=round(float(p40.s_pid_p.abs().max()), 1)))
    return rows


def dump(df, idx, post_ticks):
    ms = df.motion_state.values
    if ms[idx] != SLALOM:
        print(f"idx {idx} は SLALOM ではない (motion_state={ms[idx]})")
        return
    e = idx
    while e < len(ms) and ms[e] == SLALOM:
        e += 1
    cols = ["motion_state", "ideal_v", "v_c", "v_l", "v_r", "ideal_w", "w_lp",
            "ideal_ang", "ang", "kim_theta", "left45_d", "right45_d",
            "s_pid_p", "duty_sen", "duty_l", "duty_r", "dist"]
    d = df.iloc[max(idx, e - 8):min(e + post_ticks, len(df))][cols].copy()
    d["duty_sen"] = d["duty_sen"] * D
    d = d.rename(columns={"motion_state": "ms", "s_pid_p": "ey",
                          "duty_sen": "dsen_deg"})
    pd.set_option("display.width", 300)
    pd.set_option("display.max_rows", 500)
    print(f"turn idx {idx}-{e - 1}, exit at idx {e}")
    print(d.round(2).to_string())


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--summary", action="store_true",
                    help="(kind, dir, v) ごとに n / 平均 / σ を出す")
    ap.add_argument("--kind", help="旋回種別で絞る (例: dia135_2)")
    ap.add_argument("--dump", type=int, metavar="IDX",
                    help="この idx から始まる SLALOM の出口を tick 単位で表示 (ログ1本)")
    ap.add_argument("--post-ticks", type=int, default=40,
                    help="dsen40/ey40 と --dump の旋回後 tick 数 (default 40)")
    ap.add_argument("--lat-k", type=float, default=LAT_K,
                    help=f"ヨー→見かけ横ずれ [mm/deg] (default {LAT_K})")
    a = ap.parse_args()

    if a.dump is not None:
        if len(a.logs) != 1:
            print("--dump はログ1本を指定する")
            return 1
        dump(pd.read_csv(a.logs[0]), a.dump, a.post_ticks)
        return 0

    rows = []
    for f in a.logs:
        df = pd.read_csv(f)
        miss = [c for c in NEED if c not in df]
        if miss:
            print(f"skip {f}: 列がない {miss}")
            continue
        name = os.path.basename(f).replace(".csv", "")
        rows += analyze(df, name, a.post_ticks, a.lat_k)
    if not rows:
        return 1
    t = pd.DataFrame(rows)
    if a.kind:
        t = t[t.kind == a.kind]
        if t.empty:
            print(f"kind={a.kind} の旋回がない。あるのは: {sorted(set(r['kind'] for r in rows))}")
            return 1
    t = t.sort_values(["kind", "dir", "v", "log", "idx"])
    pd.set_option("display.width", 300)
    pd.set_option("display.max_rows", 500)
    print(t.to_string(index=False))

    if a.summary:
        print()
        print("summary (mean ± std, wide>0 は外側/大回り):")
        g = t.groupby(["kind", "dir", "v"])
        for (kind, dr, v), x in g:
            def ms(c, nd=1):
                s = x[c].dropna()
                if s.empty:
                    return "   n/a     "
                return f"{s.mean():6.{nd}f}±{s.std(ddof=0):4.{nd}f}"
            print(f"  {kind:9s} {dr} {v:5d} n={len(x):2d} "
                  f"wide{ms('wide')} off0{ms('off0')} yaw0{ms('yaw0')} "
                  f"lag{ms('lag')} sat{ms('sat')} vc_min{ms('vc_min', 2)} "
                  f"dsen40{ms('dsen40')}")
        print()
        print("感度 (外側正): 180°: rad 1mm→2.0mm  135°: rad→1.7, front→0.71  "
              "90°: rad→1.0, front→1.0  45°: rad→0.29, front→0.71")
    return 0


if __name__ == "__main__":
    sys.exit(main())
