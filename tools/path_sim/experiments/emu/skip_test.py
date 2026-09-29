#!/usr/bin/env python3
"""試作の高速化を全部入れた探索シミュレータが、元と同じ結果になるかを探索 1 本まるごとで確かめる。
     表づくり = opt2.cpp の opt_search_goal_position()
     地図が前回の表づくりから変わっていなければ fast_search_goal_position()(表は作り直さない)
     歩数マップ = opt_update_dist_map()
   比べるのは search_sim の出力(判断ごとの位置・動き・時刻・サブゴールの数と区画・地図の変化・候補の経路)。"""
import json, os, subprocess, sys
from multiprocessing import Pool
H = os.path.dirname(os.path.abspath(__file__))
R = os.path.abspath(os.path.join(H, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(H, "..", "px"))

def build():
    s = open(f"{R}/src/search/adachi.cpp").read()
    old = """      lgc->set_param_num(1);
      lgc->set_param();
      lgc->searchGoalPosition(true, subgoal_list);
      cost_mode = 3;"""
    assert s.count(old) == 1
    s = s.replace(old, """      lgc->set_param_num(1);
      lgc->set_param();
      if (g_skip_valid && lgc->map == g_last_map) {
        fast_search_goal_position(*lgc, subgoal_list);
        g_n_skip++;
      } else {
        opt_search_goal_position(*lgc, subgoal_list);
        g_last_map = lgc->map;
        g_skip_valid = true;
        g_n_rebuild++;
      }
      cost_mode = 3;""")
    s = s.replace('#include "adachi.hpp"', '''#include "adachi.hpp"
void opt_search_goal_position(MazeSolverBaseLgc &l, std::unordered_map<unsigned int, unsigned char> &subgoal_list);
void fast_search_goal_position(MazeSolverBaseLgc &l, std::unordered_map<unsigned int, unsigned char> &subgoal_list);
void opt_update_dist_map(MazeSolverBaseLgc &l, int mode, bool search_mode);
static std::vector<unsigned char> g_last_map;
static bool g_skip_valid = false;
int g_n_skip = 0, g_n_rebuild = 0;
// 歩数マップは試作のほうを呼ぶ(元の本体は logic のビルドで update_dist_map_orig に名前を変えてある)
void MazeSolverBaseLgc::update_dist_map(const int mode, const bool search_mode) { opt_update_dist_map(*this, mode, search_mode); }
''', 1)
    open(f"{H}/adachi_skip.cpp", "w").write(s)
    m = open(f"{R}/tools/path_sim/search_main.cpp").read()
    m = m.replace("int main() {", "extern int g_n_skip, g_n_rebuild;\nint main() {", 1)
    old = '  out["search_timer"] = sim.param_->seach_timer;'
    assert m.count(old) == 1
    m = m.replace(old, old + '\n  out["n_skip"] = g_n_skip;\n  out["n_rebuild"] = g_n_rebuild;')
    open(f"{H}/skip_main.cpp", "w").write(m)
    inc = f"-I{H} -I{R}/tools/path_sim/stub -I{R}/tools/path_sim -I{R} -I{R}/include -I{R}/include/search -I{R}/include/action -I{R}/build/_deps/arduinojson-src/src"
    fl = "-std=gnu++20 -O2 -w"
    cmds = [
        f"g++ {fl} {inc} -Dupdate_dist_map=update_dist_map_orig -c {R}/src/search/logic.cpp -o {H}/obj_logic_ren.o",
        f"g++ {fl} {inc} -c {H}/adachi_skip.cpp -o {H}/obj_adachi_skip.o",
        f"g++ {fl} {inc} -c {H}/opt2.cpp -o {H}/obj_opt2.o",
        f"g++ {fl} {inc} -c {H}/skip_main.cpp -o {H}/obj_skip_main.o",
    ]
    for c in cmds: subprocess.check_call(c, shell=True)
    subprocess.check_call(f"g++ {fl} {inc} {H}/obj_logic_ren.o {H}/obj_adachi_skip.o {H}/obj_opt2.o {H}/obj_skip_main.o {R}/src/action/path_creator.cpp {R}/src/action/time_path_planner.cpp {R}/src/action/trajectory_creator.cpp {R}/tools/path_sim/host_common.cpp -o {H}/search_sim_fast", shell=True)
    subprocess.check_call(["make", "-s"], cwd=f"{R}/tools/path_sim")

def job(mi):
    import ev, se
    m = ev.MAZES[mi]
    inp = json.dumps({"files": ev._files, "truth": se.truth_of(m), "goals": m["goals"]}).encode()
    a = json.loads(subprocess.run([f"{R}/tools/path_sim/build/search_sim"], input=inp, capture_output=True).stdout)
    b = json.loads(subprocess.run([f"{H}/search_sim_fast"], input=inp, capture_output=True).stdout)
    skip, rebuild = b.pop("n_skip"), b.pop("n_rebuild")
    same = a == b
    first = None
    if not same:
        for i, (x, y) in enumerate(zip(a["steps"], b["steps"])):
            if x != y: first = i; break
    return m["name"], same, first, skip, rebuild, len(a["steps"])

if __name__ == "__main__":
    build()
    import ev
    with Pool() as p: outs = p.map(job, range(len(ev.MAZES)))
    bad = [o for o in outs if not o[1]]
    for o in bad: print("NG", o)
    sk = sum(o[3] for o in outs); rb = sum(o[4] for o in outs)
    print(f"{len(outs)} 迷路: 元と同じ {len(outs) - len(bad)} / 違う {len(bad)}  ゴール後の update {sk + rb} 回のうち、表を作り直さずに済んだ {sk} 回({sk / (sk + rb) * 100:.0f} %)")
    for o in outs: print(f"   {o[0]:18s} 作り直し {o[4]:4d} / 近道 {o[3]:4d}({o[3] / max(1, o[3] + o[4]) * 100:.0f} %)")
