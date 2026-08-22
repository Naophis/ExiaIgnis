#!/usr/bin/env python3
"""motion_state が指定値(デフォルト 6, 13)になった瞬間と、
そこから抜けた瞬間(デフォルトでは通常4へ遷移した時)の
left45_d/left45_2_d/left45_3_d, right45_d/right45_2_d/right45_3_d を表示する。

Usage:
    python3 analyze_motion_state_transitions.py [csv_path] \
        [--states 6 13] [--exit-state 4] [--columns left45_d left45_2_d left45_3_d right45_d right45_2_d right45_3_d]
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


def find_blocks(rows, states):
    """motion_state が states に含まれる連続区間を [(start_row, end_row, next_ms), ...] で返す"""
    n = len(rows)
    blocks = []
    block_start = None
    prev = None
    for i, r in enumerate(rows):
        ms = as_float(r, "motion_state")
        in_state = ms in states
        was_in_state = prev in states if prev is not None else False
        if in_state and not was_in_state:
            block_start = i
        if was_in_state and not in_state:
            blocks.append((block_start, i - 1, ms))
        prev = ms
    if block_start is not None and prev in states:
        blocks.append((block_start, n - 1, None))
    return blocks


def print_row(rows, i, columns, label):
    values = ", ".join(f"{col}={fmt(rows[i], col)}" for col in columns)
    print(f"  [{label}] index={rows[i]['index']} motion_state={rows[i]['motion_state']}: {values}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", nargs="?", default="logs/latest.csv")
    parser.add_argument("--states", type=float, nargs="+", default=[6, 13])
    parser.add_argument("--exit-state", type=float, default=4,
                         help="この状態への遷移で終わった区間のみ「終了」として明示する")
    parser.add_argument("--columns", nargs="+",
                         default=["left45_d", "left45_2_d", "left45_3_d",
                                  "right45_d", "right45_2_d", "right45_3_d"])
    args = parser.parse_args()

    with open(args.csv_path, newline="") as f:
        rows = list(csv.DictReader(f))

    for state in args.states:
        blocks = find_blocks(rows, [state])
        print(f"=== motion_state={state} ===")
        if not blocks:
            print("  該当区間なし")
            print()
            continue
        for b_start, b_end, next_ms in blocks:
            print_row(rows, b_start, args.columns, "開始")
            if next_ms is None:
                print(f"  [終了] データ終端まで motion_state={state} が継続")
            elif next_ms == args.exit_state:
                print_row(rows, b_end + 1, args.columns, f"終了(motion_state={args.exit_state}へ遷移)")
            else:
                print_row(rows, b_end + 1, args.columns, f"終了(motion_state={next_ms}へ遷移)")
            print()
        print()


if __name__ == "__main__":
    main()
