#!/usr/bin/env python3
"""直進の横方向(壁追従 × 姿勢保持)ループの符号チェックツール。

ヨーレートループの目標値 w_cmd に加算される offset は
control_law.cpp calc_pid_val_ang_vel() より

    offset = duty_roll_ang        (= angle_pid.p*(img_ang+duty_sen-ang_kf) + i,d)
           + sen_kanayama_dw      (= kanayama_straight.ky*ey + ki*sen.error_i
                                     + k_theta*sin(e_theta))
           + turn_angle_fb.w_gain*ang.i_bias

で、CSVにはそれぞれ dbg_off_ang / dbg_off_kny / dbg_off_wgain として出ている。

問題になるのは、この2つの主要項をどちらも「姿勢 theta の関数」として見たときの
符号:

    dbg_off_ang  = -angle_pid.p * theta                      … 負帰還(姿勢を戻す)
    dbg_off_kny ≈  ki * (d sen.error_i / d theta) * theta     … 符号は環境依存

sen.error_i は ey(=45°左右差)のリークなし積分なので、ey に「ヨーレート w と
同符号の成分」が乗っていると sen.error_i が theta(=∫w)を追いかけてしまい、
dbg_off_kny が正帰還になる。そうなると合計が正になった時点で直進は発散する。

このスクリプトは
  1. ey が本当に 45°左右差か
  2. sen.error_i が素の積分か
  3. sen.error_i vs theta の傾き S [1/rad]
  4. 正帰還ゲイン ki*S と 負帰還ゲイン angle_pid.p の大小
を実測から出す。ki と angle_pid.p はログから逆算するので yaml と同期不要。

    ki < angle_pid.p / S  が安定条件。

使い方:
    python3 tools/param_tuner/lat_check.py logs/2026*.csv
"""
import sys

import numpy as np
import pandas as pd

# sen_ref_p.normal.exist.left45/right45 (壁ありと判定する距離のしきい値[mm])
EXIST_45 = 44.75
# sen_ref_p.normal.exist.left90/right90
EXIST_90 = 60.0

REQ = ['motion_state', 'ideal_v', 'ang_kf', 'w_lp', 'y',
       's_pid_p', 's_pid_i', 's_pid_i_v', 'ang_pid_p',
       'dbg_off_ang', 'dbg_off_kny', 'duty_sen',
       'left45_d', 'right45_d', 'left90_d', 'right90_d']


def reg(y, x):
    """slope, intercept, r を返す。"""
    A = np.c_[x, np.ones(len(x))]
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    r = np.corrcoef(x, y)[0, 1] if x.std() > 0 and y.std() > 0 else float('nan')
    return c[0], c[1], r


def analyze(path):
    d = pd.read_csv(path, low_memory=False)
    if not set(REQ) <= set(d.columns):
        print(f"{path}: 必要な列が足りません (古いログ?)")
        return
    d = d[d.motion_state != 0].reset_index(drop=True)
    if len(d) < 300:
        print(f"{path}: 短すぎます (n={len(d)})")
        return

    name = path.split('/')[-1]
    print(f"\n{'='*78}\n### {name}  n={len(d)}  v_max={d.ideal_v.max():.0f}mm/s")

    # --- 走行結果 -----------------------------------------------------------
    th_deg = d.ang_kf
    print(f"  横ずれ y  終端 {d.y.iloc[-1]:+7.2f} mm   |最大| {d.y.abs().max():6.2f} mm")
    print(f"  姿勢 ang  平均 {th_deg.mean():+6.2f}°  範囲 {th_deg.min():+6.2f}〜{th_deg.max():+6.2f}°")
    print(f"  参考: 平均姿勢が一定だとすると横ずれは "
          f"{d.ideal_dist.max()*np.sin(np.radians(th_deg.mean())):+.1f} mm "
          f"(走行 {d.ideal_dist.max():.0f}mm)")

    # --- 壁判定 -------------------------------------------------------------
    w45 = ((d.left45_d < EXIST_45) | (d.right45_d < EXIST_45)).mean() * 100
    w90 = (((d.left90_d > 1) & (d.left90_d < EXIST_90)) |
           ((d.right90_d > 1) & (d.right90_d < EXIST_90))).mean() * 100
    print(f"  壁判定   45°が壁ありと言うtick {w45:5.1f}%   "
          f"90°が壁を見ているtick {w90:5.1f}%   duty_sen!=0 {100*(d.duty_sen!=0).mean():5.1f}%")
    if w45 - w90 > 30:
        print(f"           ※ 45°だけが壁ありと言っている。exist({EXIST_45}mm)が"
              f"45°の飽和帯に入っていないか要確認")

    # --- 解析区間: 定速かつ壁制御が動いているところ ---------------------------
    s = d[(d.ideal_v > 0.85 * d.ideal_v.max()) & (d.s_pid_i_v != 0)].reset_index(drop=True)
    if len(s) < 200:
        print("  解析区間が短く、ループゲインの同定はできません")
        return
    theta = np.radians(s.ang_kf.values)

    # --- 1. ey の正体 --------------------------------------------------------
    diff45 = (s.left45_d - s.right45_d).values
    a, _, r = reg(s.s_pid_p.values, diff45)
    print(f"\n  [1] ey = {a:+.3f} × (left45_d - right45_d)   r={r:+.3f}")

    # --- 2. sen.error_i は素の積分か ------------------------------------------
    di = np.diff(s.s_pid_i.values)
    eydt = s.s_pid_p.values[:-1] * 0.001
    clamp = np.abs(s.s_pid_i.values[:-1]) < np.abs(s.s_pid_i).max() * 0.98
    if clamp.sum() > 50:
        a2, _, r2 = reg(di[clamp], eydt[clamp])
        print(f"  [2] Δsen.error_i = {a2:+.2f} × ey·dt   r={r2:+.3f}  "
              f"(1.0/1.000ならリークなしの素の積分)")
    print(f"      クランプ張り付き {100*(~clamp).mean():.1f}% の tick")

    # --- 3. ey / sen.error_i と姿勢・ヨーレートの関係 --------------------------
    _, _, r_ey_th = reg(s.s_pid_p.values, theta)
    a_w, _, r_ey_w = reg(s.s_pid_p.values, s.w_lp.values)
    S, _, r_i_th = reg(s.s_pid_i.values, theta)
    print(f"\n  [3] ey          vs theta : r={r_ey_th:+.3f}          (姿勢と無相関なら ~0)")
    print(f"      ey          vs w_lp  : r={r_ey_w:+.3f}  slope {a_w:+.2f}  "
          f"(w_lpのノイズで傾きは減衰側にバイアス)")
    print(f"      sen.error_i vs theta : r={r_i_th:+.3f}  slope S={S:+.3f} [1/rad]")

    # --- 4. ループゲインの符号 -------------------------------------------------
    nz = s.s_pid_i.values != 0
    ki = np.median(s.s_pid_i_v.values[nz] / s.s_pid_i.values[nz])
    nzp = s.ang_pid_p.values != 0
    ap = np.median(s.dbg_off_ang.values[nzp] / s.ang_pid_p.values[nzp])
    g_kny = ki * S
    print(f"\n  [4] ログ逆算 kanayama_straight.ki = {ki:.3f}   angle_pid.p = {ap:.3f}")
    print(f"      正帰還  ki × S      = {g_kny:+7.2f}  [rad/s per rad]")
    print(f"      負帰還  -angle_pid.p= {-ap:+7.2f}")
    print(f"      合計                = {g_kny - ap:+7.2f}   "
          f"→ {'発散側(正帰還が勝っている)' if g_kny - ap > 0 else '収束側(OK)'}")
    if S > 0:
        print(f"      安定条件: ki < angle_pid.p / S = {ap/S:.2f}   (現在 {ki:.2f})")

    # --- 参考: w目標の内訳 ----------------------------------------------------
    print(f"\n  [参考] w目標の内訳 mean   ang {s.dbg_off_ang.mean():+.4f}  "
          f"kny {s.dbg_off_kny.mean():+.4f}  合計 {s.duty_roll_before.mean():+.4f}"
          f"   実測 w_lp {s.w_lp.mean():+.4f} rad/s")


def main():
    files = [a for a in sys.argv[1:] if not a.startswith('-')]
    if not files:
        print(__doc__)
        return 1
    for f in files:
        analyze(f)
    return 0


if __name__ == '__main__':
    sys.exit(main())
