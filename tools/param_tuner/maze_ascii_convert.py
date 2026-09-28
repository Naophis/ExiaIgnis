#!/usr/bin/env python3
"""kerikun11/micromouse-maze-data のテキスト迷路(.maze)を、このプロジェクトの maze_data 形式(yaml)へ変換する。

元の形式(README より): 柱 `+`、横壁 `---`、縦壁 `|`、未知の壁 ` . ` / `.`、スタート ` S `、ゴール ` G `。
上の行が北(y = N-1)、左が西(x = 0)。

出力(maze_data/*.yaml、既存の japan20xx.yaml と同じ形):
    maze_data:
      goal: [[x, y], ...]      # G の区画
      max_step_val: 1023
      maze_size: N
      wall: [...]              # idx = x * N + y、下位 4bit(N=1, E=2, W=4, S=8)

使い方:
    python3 maze_ascii_convert.py <入力.maze ...> -o <出力ディレクトリ>
変換のたびに「壁から同じテキストを描き直して元と 1 文字ずつ一致するか」を確かめ、
食い違えば変換しない。未知の壁(`.`)を含む迷路は表せないので変換しない。
"""
import argparse
import os
import sys

import yaml

N_BIT, E_BIT, W_BIT, S_BIT = 1, 2, 4, 8


def parse(text):
    lines = text.rstrip("\n").split("\n")
    n = (len(lines) - 1) // 2
    if len(lines) != 2 * n + 1 or any(len(l.rstrip()) > 4 * n + 1 for l in lines):
        raise ValueError(f"大きさが合いません(行数 {len(lines)})")
    lines = [l.ljust(4 * n + 1) for l in lines]
    walls = [0] * (n * n)
    goals, start, unknown = [], None, 0

    def hwall(row, x):  # 横壁の行 row、区画 x の上/下
        s = lines[row][4 * x + 1 : 4 * x + 4]
        if s == "---":
            return True
        if s == "   ":
            return False
        if s == " . ":
            return None
        raise ValueError(f"横壁が読めません: 行 {row + 1} 桁 {4 * x + 2} {s!r}")

    def vwall(row, col):
        c = lines[row][col]
        if c == "|":
            return True
        if c == " ":
            return False
        if c == ".":
            return None
        raise ValueError(f"縦壁が読めません: 行 {row + 1} 桁 {col + 1} {c!r}")

    for i in range(n):  # i = 上からの区画の行、y = n-1-i
        y = n - 1 - i
        for x in range(n):
            w = 0
            for present, bit in (
                (hwall(2 * i, x), N_BIT),
                (hwall(2 * i + 2, x), S_BIT),
                (vwall(2 * i + 1, 4 * x), W_BIT),
                (vwall(2 * i + 1, 4 * x + 4), E_BIT),
            ):
                if present is None:
                    unknown += 1
                elif present:
                    w |= bit
            walls[x * n + y] = w
            mark = lines[2 * i + 1][4 * x + 1 : 4 * x + 4]
            if mark == " G ":
                goals.append([x, y])
            elif mark == " S ":
                start = [x, y]
            elif mark != "   ":
                raise ValueError(f"区画の印が読めません: ({x},{y}) {mark!r}")
    for r in range(0, 2 * n + 1, 2):  # 柱
        for x in range(n + 1):
            if lines[r][4 * x] != "+":
                raise ValueError(f"柱がありません: 行 {r + 1} 桁 {4 * x + 1}")
    return n, walls, goals, start, unknown


def render(n, walls, goals, start):
    """parse() の逆。変換が正しいかの確認用。"""
    goal_set = {tuple(g) for g in goals}
    out = []
    for i in range(n):
        y = n - 1 - i
        out.append("+" + "+".join("---" if walls[x * n + y] & N_BIT else "   " for x in range(n)) + "+")
        row = "|" if walls[0 * n + y] & W_BIT else " "
        for x in range(n):
            mark = " G " if (x, y) in goal_set else (" S " if start == [x, y] else "   ")
            row += mark + ("|" if walls[x * n + y] & E_BIT else " ")
        out.append(row)
    out.append("+" + "+".join("---" if walls[x * n + 0] & S_BIT else "   " for x in range(n)) + "+")
    return "\n".join(out)


def check(n, walls, goals, start):
    """外周・スタート・隣どうしの食い違いを確かめる。問題の一覧を返す。"""
    probs = []
    for i in range(n):
        if not walls[0 * n + i] & W_BIT or not walls[(n - 1) * n + i] & E_BIT:
            probs.append(f"外周(西/東)が欠けています y={i}")
        if not walls[i * n + 0] & S_BIT or not walls[i * n + (n - 1)] & N_BIT:
            probs.append(f"外周(南/北)が欠けています x={i}")
    if start != [0, 0]:
        probs.append(f"スタートが (0,0) ではありません: {start}")
    if not goals:
        probs.append("ゴール (G) がありません")
    return probs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+")
    ap.add_argument("-o", "--out", required=True, help="出力ディレクトリ(maze_data など)")
    ap.add_argument("--suffix", default="", help="出力ファイル名の末尾に付ける文字列")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    ok = True
    for path in args.inputs:
        name = os.path.splitext(os.path.basename(path))[0]
        text = open(path, encoding="utf-8").read()
        try:
            n, walls, goals, start, unknown = parse(text)
        except ValueError as e:
            print(f"NG {name}: {e}")
            ok = False
            continue
        if unknown:
            print(f"SKIP {name}: 未知の壁が {unknown} か所あり、このプロジェクトの形式では表せません")
            continue
        # 壁から同じ図を描き直して、元と 1 文字ずつ一致するか(行末の空白の違いだけは許す)
        src = [l.rstrip() for l in text.rstrip("\n").split("\n")]
        dst = [l.rstrip() for l in render(n, walls, goals, start).split("\n")]
        if src != dst:
            bad = next(i for i, (a, b) in enumerate(zip(src, dst)) if a != b)
            print(f"NG {name}: 描き直した図が元と一致しません(行 {bad + 1})")
            ok = False
            continue
        probs = check(n, walls, goals, start)
        for p in probs:
            print(f"  注意 {name}: {p}")
        out = os.path.join(args.out, f"{name}{args.suffix}.yaml")
        data = {"maze_data": {"goal": goals, "max_step_val": n * n - 1, "maze_size": n, "wall": walls}}
        with open(out, "w", encoding="utf-8") as f:
            f.write(f"# {os.path.basename(path)} から変換(kerikun11/micromouse-maze-data、MIT License)\n")
            yaml.safe_dump(data, f, default_flow_style=False)
        print(f"OK {name}: {n}x{n} ゴール {len(goals)} 区画 → {out}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
