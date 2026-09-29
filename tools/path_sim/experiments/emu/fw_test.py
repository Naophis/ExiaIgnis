#!/usr/bin/env python3
"""いまのファームの探索(src/search)が、高速化を入れる前(../frozen2)と同じ結果になるかを、
探索 1 本まるごとで確かめる。あわせて、ゴール後の update() のうち表を作り直さずに済んだ回数を数える。

    python3 fw_test.py

  search_sim_old  ../frozen2 の logic.cpp / adachi.cpp(2026-09-29 の高速化の前)
  search_sim_cnt  いまの src/search/logic.cpp に回数のカウンタだけ足したもの + いまの adachi.cpp
探索ループ(tools/path_sim/search_main.cpp)はどちらも同じ。比べるのは出力全体
(判断ごとの位置・動き・時刻・サブゴールの区画・地図の変化・候補の経路)。
迷路は tools/path_sim/check_search.py の 27 本 + 実験用(../px/ds.py)にだけある迷路。"""
import json, os, shutil, subprocess, sys
from multiprocessing import Pool
H = os.path.dirname(os.path.abspath(__file__))
R = os.path.abspath(os.path.join(H, "..", "..", "..", ".."))
Z = os.path.join(H, "..", "frozen2")
sys.path.insert(0, os.path.join(H, "..", "px"))
sys.path.insert(0, os.path.join(R, "tools", "path_sim"))


def rep(s, old, new):
    assert s.count(old) == 1, (old, s.count(old))
    return s.replace(old, new)


def build():
    # 変更前のヘッダーを、ファームのヘッダーより先に見つかる場所へ置く
    ov = f"{H}/ov/include/search"
    os.makedirs(ov, exist_ok=True)
    for f in ("logic.hpp", "adachi.hpp"):
        shutil.copy(f"{Z}/{f}", f"{ov}/{f}")
    s = open(f"{R}/src/search/logic.cpp").read()
    s = rep(s, '#include "logic.hpp"\n', '#include "logic.hpp"\nint g_n_reuse = 0, g_n_rebuild = 0;\n')
    s = rep(s, "  if (!reuse) {\n", "  if (!reuse) {\n    g_n_rebuild++;\n")
    s = rep(s, "  age_subgoal(subgoal_list);\n  // 表づくりで取り出される区画", "  g_n_reuse++;\n  age_subgoal(subgoal_list);\n  // 表づくりで取り出される区画")
    open(f"{H}/logic_cnt.cpp", "w").write(s)
    m = open(f"{R}/tools/path_sim/search_main.cpp").read()
    m = rep(m, "int main() {", "extern int g_n_reuse, g_n_rebuild;\nint main() {")
    m = rep(m, '  out["search_timer"] = sim.param_->seach_timer;', '  out["search_timer"] = sim.param_->seach_timer;\n  out["n_reuse"] = g_n_reuse;\n  out["n_rebuild"] = g_n_rebuild;')
    open(f"{H}/cnt_main.cpp", "w").write(m)
    P = f"{R}/tools/path_sim"
    common = f"-I{P}/stub -I{P} -I{R} -I{R}/include -I{R}/include/search -I{R}/include/action -I{R}/build/_deps/arduinojson-src/src"
    old = f"-I{H}/ov -I{H}/ov/include -I{H}/ov/include/search {common}"
    fl = "-std=gnu++20 -O2 -w"
    rest = f"{R}/src/action/path_creator.cpp {R}/src/action/time_path_planner.cpp {R}/src/action/trajectory_creator.cpp {P}/host_common.cpp"
    subprocess.check_call(f"g++ {fl} {old} {Z}/logic.cpp {Z}/adachi.cpp {rest} {P}/search_main.cpp -o {H}/search_sim_old", shell=True)
    subprocess.check_call(f"g++ {fl} {common} {H}/logic_cnt.cpp {R}/src/search/adachi.cpp {rest} {H}/cnt_main.cpp -o {H}/search_sim_cnt", shell=True)


def mazes():
    import check_search as cs
    import ev, se
    out = [(n, t, g) for n, t, g in cs.MAZES]
    have = {n for n, _, _ in out}
    out += [(m["name"], se.truth_of(m), m["goals"]) for m in ev.MAZES if m["name"] not in have]
    return cs.FILES, out


def job(a):
    files, (name, truth, goals) = a
    inp = json.dumps({"files": files, "truth": truth, "goals": goals}).encode()
    a = json.loads(subprocess.run([f"{H}/search_sim_old"], input=inp, capture_output=True).stdout)
    b = json.loads(subprocess.run([f"{H}/search_sim_cnt"], input=inp, capture_output=True).stdout)
    reuse, rebuild = b.pop("n_reuse"), b.pop("n_rebuild")
    first = None
    if a != b:
        first = next((i for i, (x, y) in enumerate(zip(a["steps"], b["steps"])) if x != y), -1)
    return name, a == b, first, reuse, rebuild, len(a["steps"]), a.get("total_time")


if __name__ == "__main__":
    build()
    files, ms = mazes()
    with Pool() as p:
        outs = p.map(job, [(files, m) for m in ms])
    bad = [o for o in outs if not o[1]]
    for o in bad:
        print("NG", o)
    ru = sum(o[3] for o in outs)
    rb = sum(o[4] for o in outs)
    print(f"{len(outs)} 迷路: 変更前と同じ {len(outs) - len(bad)} / 違う {len(bad)}  ゴール後の update {ru + rb} 回のうち、表を作り直さずに済んだ {ru} 回({ru / max(1, ru + rb) * 100:.0f} %)")
    for o in outs:
        print(f"   {o[0]:18s} 判断 {o[5]:5d} 回  探索 {o[6]:7.1f} s  作り直し {o[4]:4d} / 作り直さない {o[3]:4d}({o[3] / max(1, o[3] + o[4]) * 100:.0f} %)")
    sys.exit(1 if bad else 0)
