#!/usr/bin/env python3
"""壁切れ(kireme)検出をグラフ目視でなく数値で評価するツール。

WallOffController (src/action/wall_off_controller.cpp) の現行判定は
「45°センサ値が絶対しきい値(noexist_th≈49mm)を超えたら壁切れ」というレベル
判定 (exist=true, 開始時点で壁が見えているパターン)。これは
  - 壁までの距離(=姿勢/寄り)でベースラインが変わるぶんだけ検出位置がずれる
  - 立ち上がりの傾きが速度で変わるので検出までの走行距離もずれる
という2つの理由で、姿勢が悪いときに検出が遅れる・ばらつく…はずだった。

もう一つ、開始時点で壁がまだ見えていない(exist=false)パターンもある。この
場合センサ値は遠い値から近づいてきて最小値(トラフ)を打ち、そこから離れて
いく形になる。現行コードはこちらでは絶対しきい値ではなく
sen.l45/r45.sensor_dist(最小値保持)からの偏差判定を使っており、実測では
検出遅れが小さい(後述)。

このスクリプトは両パターンを自動判別し(区間内のセンサ値の最小点を
「壁に最も寄った瞬間」とみなしてベースラインをそこから取る。開始時点で
既に最小付近ならパターンA=開始時点で可視、区間の途中に最小があれば
パターンB=不可視から接近、を意味する)、同じログから

  1. 現行方式が実際どこで発火したか (= motion_state セグメント終端の走行距離)
  2. ベースライン+ARM_DELTA を最初に超える「気づいた点」(Layer1、しきい値
     判定のみで曲線の形を仮定しない)
  3. 立ち上がり区間を直線フィットしてベースライン方向に外挿した「エッジ位置」
     (Layer2、検出方式によらない切れ目そのものの推定走行距離)

を出す。

重要な注意(2026-09-05, n=4実測で判明): Layer2(直線外挿)は個々のログでは
r2=0.93〜0.99と当てはまりが良く見えても、複数試行でのばらつきは現行方式
(x_actual)よりむしろ大きくなることがある。理由は x_edge=(baseline-切片)/傾き
の計算が傾きの推定誤差で割るため、傾きが試行間で変動する(=センサー読みの
立ち上がりが真に線形ではなく、姿勢や進入角で形が変わる)とその変動がそのまま
位置誤差に増幅されるため。この設計を採用する前に、必ず summarize() の
std(x_edge) vs std(x_actual) vs std(x_arm) を確認すること。単発のr2の高さは
「試行間の再現性」を保証しない。

フィットは走行距離ではなく「ベースラインからの値の窓」で切る
(fit-lo〜fit-hi mm)。立ち上がりは凸なので走行距離窓だと速度・姿勢で切り取る
部分がずれて傾きがぶれるため。

使い方:
    python3 tools/param_tuner/wall_off_edge_check.py logs/2026*.csv
    python3 tools/param_tuner/wall_off_edge_check.py --state 13 logs/2026*.csv   # 斜め壁切れ
"""
import sys

import numpy as np
import pandas as pd

REQ = ['motion_state', 'dist', 'left45_d', 'right45_d', 'v_c']

BASELINE_N = 5      # ベースライン推定に使う、アンカー(最小点)からのサンプル数
ARM_DELTA = 1.0      # [mm] ベースラインからこれだけ離れたら「気づいた」候補
FIT_LO = 1.0         # [mm] フィット窓の下端 (ベースライン+この値から)
FIT_HI = 5.0         # [mm] フィット窓の上端
MIN_FIT_N = 3        # フィットに必要な最小サンプル数
MEDIAN_WINDOW = 3    # アンカー(最小点)探索用の平滑化窓
VISIBLE_TH = 2.0     # [mm] アンカーの走行距離がこれ以下なら「開始時点で可視」


def reg(x, y):
    """slope, intercept, r2 を返す。"""
    A = np.c_[x, np.ones(len(x))]
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    if x.std() > 0 and y.std() > 0:
        r = np.corrcoef(x, y)[0, 1]
        r2 = r * r
    else:
        r2 = float('nan')
    return c[0], c[1], r2


def median_filter(values, window):
    if window <= 1:
        return values.copy()
    half = window // 2
    n = len(values)
    out = np.empty(n)
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out[i] = np.median(values[lo:hi])
    return out


def find_segments(state_col, target):
    segs = []
    start = None
    for i, v in enumerate(state_col):
        if start is None and v == target:
            start = i
        elif start is not None and v != target:
            segs.append((start, i - 1))
            start = None
    if start is not None:
        segs.append((start, len(state_col) - 1))
    return segs


def find_anchor(y):
    """区間内でセンサ値が最も壁に近づいた(=最小の)点を探す。

    開始時点で既に壁が見えているパターンAでは、この点はほぼ区間先頭に
    一致する(最小値=開始直後の値)。開始時点で見えていないパターンBでは、
    値が遠い所から下がってきてどこかで底を打つので、その底がここで
    検出される。medianフィルタで単発ノイズによる誤検出を避け、同値が
    複数ある場合は最初の(=最も早い)ものを採用する。
    """
    smoothed = median_filter(y, MEDIAN_WINDOW)
    return int(np.argmin(smoothed))


def analyze_side(seg_df, col):
    x = seg_df['dist'].values
    y = seg_df[col].values
    n = len(x)
    anchor_i = find_anchor(y)
    window_end = min(anchor_i + BASELINE_N, n)
    baseline = float(np.median(y[anchor_i:window_end]))
    delta = float(y[-1] - baseline)
    return dict(anchor_i=anchor_i, baseline=baseline, delta=delta, x=x, y=y)


def analyze_segment(seg_df, side):
    col = f'{side}45_d'
    info = analyze_side(seg_df, col)
    x, y = info['x'], info['y']
    n = len(x)
    anchor_i, baseline = info['anchor_i'], info['baseline']

    if n - anchor_i < BASELINE_N + MIN_FIT_N:
        return None

    pattern = 'A(開始時可視)' if x[anchor_i] <= VISIBLE_TH else 'B(接近して不可視から検出)'
    x_actual = float(x[-1])   # 現行方式が実際に発火した走行距離
    y_actual = float(y[-1])   # そのときのセンサ値 (baselineとの差はmm単位で比較可能)
    x_anchor = float(x[anchor_i])

    # Layer1: アンカー以降で baseline+ARM_DELTA を最初に超えた点
    over = np.where(y[anchor_i:] - baseline > ARM_DELTA)[0]
    x_arm = float(x[anchor_i + over[0]]) if len(over) else None

    # Layer2: アンカー以降、baseline+[FIT_LO, FIT_HI] の窓だけで直線フィットし外挿
    tail_y = y[anchor_i:]
    tail_x = x[anchor_i:]
    mask = (tail_y - baseline > FIT_LO) & (tail_y - baseline < FIT_HI)
    if mask.sum() < MIN_FIT_N:
        return dict(pattern=pattern, baseline=baseline, x_anchor=x_anchor,
                    x_actual=x_actual, y_actual=y_actual, x_arm=x_arm,
                    x_edge=None, slope=None, r2=None, n_fit=int(mask.sum()))

    a, c, r2 = reg(tail_x[mask], tail_y[mask])
    x_edge = (baseline - c) / a if a != 0 else None
    return dict(pattern=pattern, baseline=baseline, x_anchor=x_anchor,
                x_actual=x_actual, y_actual=y_actual, x_arm=x_arm,
                x_edge=x_edge, slope=a, r2=r2, n_fit=int(mask.sum()))


def pick_side(seg_df):
    """区間内でより大きく立ち上がった側を壁切れ対象側とみなす。

    左右それぞれ自分のアンカー(最小点)基準でdeltaを取るので、パターンA/B
    どちらでも(=開始時点で近い/遠いのどちらでも)同じロジックで判定できる。
    """
    left = analyze_side(seg_df, 'left45_d')
    right = analyze_side(seg_df, 'right45_d')
    return 'right' if right['delta'] > left['delta'] else 'left'


def analyze(path, state):
    d = pd.read_csv(path, low_memory=False)
    if not set(REQ) <= set(d.columns):
        print(f"{path}: 必要な列が足りません (古いログ?)")
        return []

    segs = find_segments(d['motion_state'].values, state)
    if not segs:
        print(f"{path}: motion_state=={state} の区間が見つかりません")
        return []

    name = path.split('/')[-1]
    results = []
    for s0, s1 in segs:
        seg = d.iloc[s0:s1 + 1]
        if len(seg) < BASELINE_N + MIN_FIT_N:
            continue
        side = pick_side(seg)
        r = analyze_segment(seg, side)
        if r is None:
            continue
        r['file'] = name
        r['side'] = side
        r['v'] = float(seg['v_c'].iloc[:BASELINE_N].mean())
        results.append(r)

        print(f"\n### {name}  [{side}]  {r['pattern']}  v≈{r['v']:.0f}mm/s  "
              f"n_seg={len(seg)}")
        print(f"  アンカー(最小点)   x_anchor = {r['x_anchor']:6.2f} mm   "
              f"baseline = {r['baseline']:6.2f} mm")
        print(f"  現行方式の発火位置  x_actual = {r['x_actual']:6.2f} mm"
              f"  (センサ値 baseline+{r['y_actual'] - r['baseline']:.2f}mm で発火)")
        if r['x_arm'] is not None:
            print(f"  Layer1 気づいた位置 x_arm    = {r['x_arm']:6.2f} mm"
                  f"  (baseline+{ARM_DELTA}mm 到達)")
        if r['x_edge'] is not None:
            print(f"  Layer2 逆算エッジ   x_edge   = {r['x_edge']:6.2f} mm"
                  f"   slope={r['slope']:.3f} mm/mm  r2={r['r2']:.3f}  "
                  f"n_fit={r['n_fit']}")
            print(f"  現行方式との差      x_actual - x_edge = "
                  f"{r['x_actual'] - r['x_edge']:+6.2f} mm  (=検出遅れ)")
        else:
            print(f"  Layer2: フィット点不足 (n_fit={r['n_fit']}) "
                  f"→ FIT_LO/FIT_HI かセグメント長を見直してください")

    return results


def summarize(all_results):
    if len(all_results) < 2:
        return
    print(f"\n{'='*78}\n### 複数試行の比較 (n={len(all_results)})")
    for side in ('left', 'right'):
        for pattern_key, pattern_label in (('A', 'A(開始時可視)'), ('B', 'B(接近して不可視から検出)')):
            rs = [r for r in all_results if r['side'] == side and r['pattern'] == pattern_label]
            if len(rs) < 2:
                continue
            act = np.array([r['x_actual'] for r in rs])
            arm = np.array([r['x_arm'] for r in rs if r['x_arm'] is not None])
            edge = np.array([r['x_edge'] for r in rs if r['x_edge'] is not None])
            print(f"\n  [{side}] {pattern_label}  n={len(rs)}")
            print(f"    現行方式  x_actual : mean={act.mean():6.2f}  "
                  f"std={act.std():5.2f}  range=[{act.min():.2f}, {act.max():.2f}]")
            if len(arm) >= 2:
                print(f"    Layer1    x_arm    : mean={arm.mean():6.2f}  "
                      f"std={arm.std():5.2f}  range=[{arm.min():.2f}, {arm.max():.2f}]"
                      f"   std比={arm.std()/act.std():.2f}" if act.std() > 0 else "")
            if len(edge) >= 2:
                print(f"    Layer2    x_edge   : mean={edge.mean():6.2f}  "
                      f"std={edge.std():5.2f}  range=[{edge.min():.2f}, {edge.max():.2f}]")
                if act.std() > 0:
                    print(f"    ばらつき比 std(x_edge)/std(x_actual) = "
                          f"{edge.std()/act.std():.2f}"
                          f"  ({'改善' if edge.std() < act.std() else '悪化/要確認'})")
            # 速度依存性: slopeが速度で変わっていないか(=Layer2が本当に速度不変か)
            speeds = np.array([r['v'] for r in rs])
            slopes = np.array([r['slope'] for r in rs if r['slope'] is not None])
            if len(slopes) == len(speeds) and speeds.std() > 20:
                a, _, r2 = reg(speeds, slopes)
                print(f"    slope vs v: {a:+.5f} mm/mm per mm/s   r2={r2:.3f}  "
                      f"(0に近いほど速度不変)")


def main():
    args = sys.argv[1:]
    state = 6
    files = []
    i = 0
    while i < len(args):
        if args[i] == '--state':
            state = int(args[i + 1])
            i += 2
        elif not args[i].startswith('-'):
            files.append(args[i])
            i += 1
        else:
            i += 1
    if not files:
        print(__doc__)
        return 1

    all_results = []
    for f in files:
        all_results.extend(analyze(f, state))
    summarize(all_results)
    return 0


if __name__ == '__main__':
    sys.exit(main())
