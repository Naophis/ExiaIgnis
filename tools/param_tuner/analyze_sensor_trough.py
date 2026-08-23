#!/usr/bin/env python3
"""motion_state=4 が終わった直後、motion_state が --states
(デフォルト 14 または 1)である区間の中でだけ、指定センサー列
(left45_d/right45_d/left90_d/right90_d 等)が下降してから
再び上昇に転じるタイミング(極小点)を求める。
区間内に複数回の下降->上昇(オシレーション)があれば、その全てを
順番に検出する。motion_state が --states から外れた時点で区間終了
とみなし、それ以降は探索しない。

ノイズ対策として --eps 未満の変動は下降/上昇とみなさない
(シュミットトリガ的に: 直近の極大/極小から --eps 以上動いたら
反転とみなすジグザグ検出)。センサー値が 0 以下の行は
未確定値とみなして無視する(motion_state 遷移直後に一時的に
0 が入ることがあるため)。また --median-window (デフォルト3)幅の
メディアンフィルタを反転判定前にかけ、1サンプルだけのスパイクを
本物の反転と誤検出しないようにしている。

Usage:
    python3 analyze_sensor_trough.py [csv_path] \
        [--columns left45_d right45_d left90_d right90_d] \
        [--motion-state 4] [--states 14 1] [--eps 3.0] [--median-window 3]
"""
import argparse
import csv


def as_float(row, key):
    try:
        return float(row[key])
    except (KeyError, ValueError, TypeError):
        return None


def as_sensor_float(row, key):
    """センサー距離列の値を取得する。0以下は motion_state 遷移直後の
    未確定値とみなして None を返す。"""
    v = as_float(row, key)
    if v is None or v <= 0:
        return None
    return v


def fmt(row, key):
    v = as_float(row, key)
    return f"{v:.2f}" if v is not None else row.get(key)


def find_motion_state_ends(rows, motion_state):
    """motion_state が motion_state から別の値に変わった直後の index を返す"""
    ends = []
    prev = None
    for i, r in enumerate(rows):
        ms = as_float(r, "motion_state")
        if prev == motion_state and ms is not None and ms != motion_state:
            ends.append(i)
        prev = ms
    return ends


def median_filter(values, window):
    """1サンプルだけの外れ値(スパイク)を除去するための単純なメディアンフィルタ。
    left90_d/right90_d はセンサー値が大きいほど1サンプルだけの読み取り誤差も
    大きくなり(例: 152.6のような単発の落ち込みや、逆に単発の跳ね上がり)、
    それを本物の反転と誤検出してしまうため、ジグザグ判定の前に適用する。"""
    if window <= 1:
        return list(values)
    half = window // 2
    n = len(values)
    out = []
    for k in range(n):
        seg = sorted(values[max(0, k - half):min(n, k + half + 1)])
        out.append(seg[len(seg) // 2])
    return out


def find_troughs(rows, col, start, eps, in_states=None, median_window=3):
    """start以降(in_states区間内)で、値のジグザグ(下降->上昇)を繰り返し検出し、
    見つかった (trough, rise) ペアを全てリストで返す。反転判定はメディアン
    フィルタ後の値で行うが、返す行インデックスは元の行(未フィルタ)のもの。

    区間終了までに上昇へ転じなかった谷が残っていれば、rise_row=None として
    最後に追加する。

    戻り値: [{"trough_row": i, "rise_row": j または None}, ...] (時系列順)
    """
    n = len(rows)

    def in_window(i):
        if in_states is None:
            return True
        return as_float(rows[i], "motion_state") in in_states

    # 区間内の有効なサンプルだけを (元の行index, 値) として集める
    idxs = []
    raws = []
    i = start
    while i < n and in_window(i):
        v = as_sensor_float(rows[i], col)
        if v is not None:
            idxs.append(i)
            raws.append(v)
        i += 1
    if not raws:
        return []

    smoothed = median_filter(raws, median_window)

    results = []
    mode = "trough"  # "trough": 下降中の最小値を追跡 / "peak": 上昇中の最大値を追跡
    cycle_start_v = smoothed[0]  # 現在の追跡が、この水準から eps 以上動いて初めて確定とみなす
    extreme_k, extreme_v = 0, smoothed[0]
    for k in range(1, len(smoothed)):
        v = smoothed[k]
        if mode == "trough":
            if v < extreme_v:
                extreme_k, extreme_v = k, v
            elif v > extreme_v + eps:
                if extreme_v <= cycle_start_v - eps:
                    results.append({"trough_row": idxs[extreme_k], "rise_row": idxs[k]})
                mode = "peak"
                cycle_start_v = extreme_v
                extreme_k, extreme_v = k, v
        else:  # peak
            if v > extreme_v:
                extreme_k, extreme_v = k, v
            elif v < extreme_v - eps:
                mode = "trough"
                cycle_start_v = extreme_v
                extreme_k, extreme_v = k, v

    if mode == "trough" and extreme_v <= cycle_start_v - eps:
        results.append({"trough_row": idxs[extreme_k], "rise_row": None})

    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_path", nargs="?", default="logs/latest.csv")
    parser.add_argument("--columns", nargs="+", default=["left45_d", "right45_d", "left90_d", "right90_d"])
    parser.add_argument("--motion-state", type=float, default=4)
    parser.add_argument("--states", type=float, nargs="+", default=[14, 1],
                         help="この motion_state 区間内でのみ trough を探索する")
    parser.add_argument("--eps", type=float, default=3.0, help="ノイズとみなして無視する変動幅")
    parser.add_argument("--median-window", type=int, default=3,
                         help="反転判定前にかけるメディアンフィルタの窓幅(1で無効化)")
    args = parser.parse_args()
    in_states = set(args.states)

    with open(args.csv_path, newline="") as f:
        rows = list(csv.DictReader(f))

    ends = find_motion_state_ends(rows, args.motion_state)
    print(f"motion_state={args.motion_state} の終了点 (index列): {[rows[i]['index'] for i in ends]}")
    print(f"探索区間: motion_state in {sorted(in_states)}")
    print()

    for col in args.columns:
        print(f"=== {col} ===")
        if not ends:
            print("  該当する motion_state 終了点なし")
            print()
            continue
        for e in ends:
            end_index = rows[e]["index"]
            troughs = find_troughs(rows, col, e, args.eps, in_states, args.median_window)
            if not troughs:
                print(f"  end_index={end_index}: 区間内で下降を検出できず")
                continue

            for n, result in enumerate(troughs, start=1):
                trough_row = result["trough_row"]
                trough_index = rows[trough_row]["index"]
                if result["rise_row"] is None:
                    print(f"  end_index={end_index} [{n}]: 極小 index={trough_index} "
                          f"(val={fmt(rows[trough_row], col)}) ですが、その後上昇に転じないまま区間終了/データ終端")
                    continue

                rise_row = result["rise_row"]
                rise_index = rows[rise_row]["index"]
                diff = int(float(rise_index)) - int(float(trough_index))
                print(f"  end_index={end_index} [{n}] | 極小: index={trough_index} (val={fmt(rows[trough_row], col)}) -> "
                      f"上昇開始: index={rise_index} (val={fmt(rows[rise_row], col)}) | diff={diff}")
        print()


if __name__ == "__main__":
    main()
