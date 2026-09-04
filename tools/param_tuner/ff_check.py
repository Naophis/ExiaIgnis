#!/usr/bin/env python3
"""直進FF(並進)の同定チェックツール。

torque_mode==2 の出力は control_law.cpp summation_duty() より

    req_v = (ff_front_torque + ff_friction_torque + duty_c) * Resist/km_gear
            + ff_duty_rpm                              [V]
    duty  = req_v / battery_lp * 100                   [%]

で、duty_roll は duty_r に +、duty_l に - で乗るため (duty_l+duty_r)/2 を取ると
消える。よって

    V_applied = (duty_l+duty_r)/2 * battery / 100      … 実際に必要だった電圧
    V_ff      = FF項だけから作った電圧                  … モデルの予測
    V_applied - V_ff = duty_c * Resist/km_gear          … FBが埋めた分

が CSV から厳密に再構成できる。V_applied を (accl, v, 1) で回帰した係数と
V_ff を同じく回帰した係数の比が 1.0 なら FF は正しい。1.0 未満なら FF 過大で、
その差を motor_pid2 の I 項が毎 tick 打ち消していることを意味する。

使い方:
    python3 tools/param_tuner/ff_check.py logs/2026*.csv
    python3 tools/param_tuner/ff_check.py --pool logs/A.csv logs/B.csv   # 複数本まとめて1回帰
"""
import sys
import numpy as np
import pandas as pd

# hardware.yaml と合わせること
RESIST = 4.4
KM = 0.00180
GEAR = 63.0 / 13.0
TIRE = 13.9750          # 直径[mm] (半径ではない)
KE = 0.000975

G = RESIST / (KM * GEAR)        # トルク[Nm] -> 電圧[V]
R_M = TIRE / 2000.0             # 半径[m]
# mpc_tgt_calc.cpp の back-EMF 項は 2*Ke*omega 相当の係数で入る (Gain定数由来)。
# 実測: ff_duty_rpm / v[m/s] = 1.3324 @ Ke=0.000975 → 単位Keあたり 1366.6
KBEMF_PER_KE = 1366.6

REQ = ['motion_state', 'duty_l', 'duty_r', 'battery', 'accl', 'ideal_v',
       'ff_front_torque', 'ff_friction_torque_r', 'ff_friction_torque_l',
       'ff_duty_rpm_r', 'ff_duty_rpm_l',
       'm_pid_p_v', 'm_pid_i_v', 'm_pid_i2_v', 'm_pid_d_v']


def prep(path):
    d = pd.read_csv(path, low_memory=False)
    if not set(REQ) <= set(d.columns):
        return None
    d = d[d.motion_state != 0].reset_index(drop=True)
    d['V'] = (d.duty_l + d.duty_r) / 2 * d.battery / 100.0
    d['duty_c'] = d.m_pid_p_v + d.m_pid_i_v + d.m_pid_i2_v + d.m_pid_d_v
    fric = (d.ff_friction_torque_r + d.ff_friction_torque_l) / 2
    d['Vff'] = (d.ff_front_torque + fric) * G + (d.ff_duty_rpm_r + d.ff_duty_rpm_l) / 2
    d['a'] = d.accl / 1000.0        # m/s^2 (FFが使うのと同じ指令値)
    d['v'] = d.ideal_v / 1000.0     # m/s   (FFが使うのと同じ目標値)
    sat = (d.duty_l.abs() > 99) | (d.duty_r.abs() > 99)
    # 加減速中のみ。定速だと a と v が分離できない
    return d[(d.a.abs() > 1) & (d.v > 0.2) & (~sat)]


def fit(d, label):
    A = np.c_[d.a, d.v, np.sign(d.v)]
    if len(d) < 60:
        print(f"{label}: サンプル不足 (n={len(d)})")
        return
    c, *_ = np.linalg.lstsq(A, d.V, rcond=None)
    cm, *_ = np.linalg.lstsq(A, d.Vff, rcond=None)
    res = d.V - A @ c
    se = np.sqrt(np.diag(res.var(ddof=3) * np.linalg.inv(A.T @ A)))
    print(f"\n### {label}  n={len(d)}  Vbat={d.battery.mean():.2f}  "
          f"v<={d.v.max():.2f}m/s |a|<={d.a.abs().max():.0f}m/s^2  cond={np.linalg.cond(A):.0f}")
    print(f"  実測必要   V = {c[0]:.4f}(±{se[0]:.4f})*a + {c[1]:.4f}(±{se[1]:.4f})*v "
          f"{c[2]:+.4f}(±{se[2]:.4f})")
    print(f"  FFモデル   V = {cm[0]:.4f}*a + {cm[1]:.4f}*v {cm[2]:+.4f}")
    print(f"  ratio      a={c[0]/cm[0]:.3f}  v={c[1]/cm[1]:.3f}   (1.00ならFF一致)")
    print(f"  I項が毎tick埋めている電圧: mean {(d.duty_c*G).mean():+.2f} V "
          f"({(d.duty_c*G).mean()/d.battery.mean()*100:+.1f}%duty)")
    print(f"  -> Mass            = {c[0]/(R_M*G):.5f} kg")
    print(f"  -> 速度比例分       = {c[1]:.4f} V/(m/s)  "
          f"[Ke据置なら viscous_friction = {(c[1]-KBEMF_PER_KE*KE)/G:.6f}]")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    pool = '--pool' in sys.argv
    ds = [(f, prep(f)) for f in args]
    ds = [(f, d) for f, d in ds if d is not None and len(d) >= 60]
    if not ds:
        print("使えるログがありません")
        return 1
    if pool:
        fit(pd.concat([d for _, d in ds]), f"pooled ({len(ds)} logs)")
    else:
        for f, d in ds:
            fit(d, f.split('/')[-1])
    return 0


if __name__ == '__main__':
    sys.exit(main())
