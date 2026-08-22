#!/usr/bin/env python3
"""motion_state=4 の開始後、指定センサー列(left45_d/right45_d 等)が
low_th 以下に下がってから high_th 以上に戻るまでの index 差分を求める。

Usage:
    python3 analyze_sensor_drop.py [csv_path] [--low 70] [--high 80] \
        [--columns left45_d right45_d] [--motion-state 4]
"""
import argparse
import csv


def as_float(row, key):
    try:
        return float(row[key])
    except (KeyError, ValueError, TypeError):
        return None


def fmt(row, key):
    v = as_float(row, key)
    return f"{v:.2f}" if v is not None else row.get(key)


def find_motion_state_starts(rows, motion_state):
    starts = []
    prev = None
    for i, r in enumerate(rows):
        ms = as_float(r, "motion_state")
        if ms == motion_state and prev != motion_state:
            starts.append(i)
        prev = ms
    return starts


def analyze(rows, col, start, low_th, high_th):
    n = len(rows)
    drop_i = None
    for i in range(start, n):
        v = as_float(rows[i], col)
        if v is not None and v <= low_th:
            drop_i = i
            break
    if drop_i is None:
        return None

    rise_i = None
    for i in range(drop_i + 1, n):
        v = as_float(rows[i], col)
        if v is not None and v >= high_th:
            rise_i = i
            break
    if rise_i is None:
        return {"drop_row": drop_i, "rise_row": None}

    return {"drop_row": drop_i, "rise_row": rise_i}


def print_motion_state_values(rows, motion_state, columns):
    starts = find_motion_state_starts(rows, motion_state)
    if not starts:
        print(f"motion_state={motion_state} の開始点は見つかりませんでした")
        return

    print(f"--- motion_state={motion_state} になった瞬間 ---")
    for s in starts:
        values = ", ".join(f"{col}={fmt(rows[s], col)}" for col in columns)
        print(f"  index={rows[s]['index']}: {values}")
    print()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", nargs="?", default="logs/latest.csv")
    parser.add_argument("--low", type=float, default=70)
    parser.add_argument("--high", type=float, default=80)
    parser.add_argument("--columns", nargs="+", default=["left45_d", "right45_d"])
    parser.add_argument("--motion-state", type=float, default=4)
    parser.add_argument("--state-values", type=float, default=3,
                         help="この motion_state のときの --state-values-columns を表示する")
    parser.add_argument("--state-values-columns", nargs="+",
                         default=["left90_d", "right90_d", "front_d"])
    parser.add_argument("--no-state-values", action="store_true",
                         help="motion_state 別センサー値の表示を無効化")
    args = parser.parse_args()

    with open(args.csv_path, newline="") as f:
        rows = list(csv.DictReader(f))

    starts = find_motion_state_starts(rows, args.motion_state)
    print(f"motion_state={args.motion_state} の開始点 (index列): "
          f"{[rows[i]['index'] for i in starts]}")
    print()

    for col in args.columns:
        print(f"=== {col} ===")
        for s in starts:
            start_index = rows[s]["index"]
            result = analyze(rows, col, s, args.low, args.high)
            if result is None:
                print(f"  start_index={start_index}: <={args.low} に到達せず")
                continue
            if result["rise_row"] is None:
                drop_row = result["drop_row"]
                print(f"  start_index={start_index}: <={args.low} "
                      f"index={rows[drop_row]['index']} (val={fmt(rows[drop_row], col)}) "
                      f"ですが、その後 >={args.high} に到達せず")
                continue
            drop_row, rise_row = result["drop_row"], result["rise_row"]
            drop_index = int(float(rows[drop_row]["index"]))
            rise_index = int(float(rows[rise_row]["index"]))
            diff = rise_index - drop_index
            print(f"  motion_state={args.motion_state} start_index={start_index} | "
                  f"<={args.low}: index={drop_index} (val={fmt(rows[drop_row], col)}) -> "
                  f">={args.high}: index={rise_index} (val={fmt(rows[rise_row], col)}) | diff={diff}")
        print()

    if not args.no_state_values:
        print_motion_state_values(rows, args.state_values, args.state_values_columns)


if __name__ == "__main__":
    main()
