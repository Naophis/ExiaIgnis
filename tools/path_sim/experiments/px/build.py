#!/usr/bin/env python3
"""実験用ビルド。ファームのソースは読むだけで、書き換えた写しはここ(scratchpad)に置く。
   pattern id: 1..5 = ファームの重みパターン、100+k = g_vp_table[k](外から入れる 6 個)"""
import os, subprocess, sys
R = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", ".."))
H = os.path.dirname(os.path.abspath(__file__))
os.makedirs(f"{H}/inc", exist_ok=True)
os.makedirs(f"{H}/obj", exist_ok=True)
def rep(s, old, new, cnt=1):
    assert s.count(old) >= 1, old
    return s.replace(old, new, cnt)
# --- logic.hpp: 外から値を入れるパターン
s = open(f"{R}/include/search/logic.hpp").read()
s = rep(s, "  void set_param() {\n", """  void set_param() {
    if (param_num >= 100) { // 実験用
      const float *v = g_vp_table[param_num - 100];
      cell_size = v[0];
      St1 = v[0]; St2 = v[1]; St3 = v[2];
      Dia = v[3]; Dia2 = v[4]; Dia3 = v[5];
      return;
    }
""")
s = rep(s, "class MazeSolverBaseLgc", "extern float g_vp_table[512][6];\nclass MazeSolverBaseLgc")
open(f"{H}/inc/logic.hpp", "w").write(s)
# --- adachi.cpp: サブゴールの重みパターンを外から
s = open(f"{R}/src/search/adachi.cpp").read()
s = rep(s, '#include "adachi.hpp"', '#include "adachi.hpp"\n#include <vector>\nint g_sub_primary = 1;\nstd::vector<int> g_sub_fallbacks = {4, 3, 2};\nint g_sub_fallback_used = 0;\n')
s = rep(s, """      lgc->set_param_num(1);
      lgc->set_param();
      lgc->searchGoalPosition(true, subgoal_list);
      cost_mode = 3;""", """      lgc->set_param_num(g_sub_primary);
      lgc->set_param();
      lgc->searchGoalPosition(true, subgoal_list);
      cost_mode = 3;""")
s = rep(s, """        const std::pair<int, int> fallbacks[] = {{4, 4}, {3, 1}, {2, 2}}; // {param_num, cost_mode}
        for (const auto &[pn, mode] : fallbacks) {""", """        for (const int pn : g_sub_fallbacks) {
          const int mode = pn;
          g_sub_fallback_used++;""")
s = rep(s, """        lgc->set_param_num(1);
        lgc->set_param();
      }
    } else {""", """        lgc->set_param_num(g_sub_primary);
        lgc->set_param();
      }
    } else {""")
open(f"{H}/adachi_px.cpp", "w").write(s)
# --- search_main.cpp
s = open(f"{R}/tools/path_sim/search_main.cpp").read()
s = rep(s, "int main() {", """float g_vp_table[512][6];
extern int g_sub_primary;
extern std::vector<int> g_sub_fallbacks;
extern int g_sub_fallback_used;
int main() {""")
s = rep(s, "  SearchSim sim;\n", """  if (in["vp_table"].is<JsonArrayConst>()) {
    int k = 0;
    for (JsonVariantConst row : in["vp_table"].as<JsonArrayConst>()) {
      int j = 0;
      for (JsonVariantConst v : row.as<JsonArrayConst>())
        g_vp_table[k][j++] = v.as<float>();
      k++;
    }
  }
  g_sub_primary = in["sub_primary"] | 1;
  if (in["sub_fallbacks"].is<JsonArrayConst>()) {
    g_sub_fallbacks.clear();
    for (JsonVariantConst v : in["sub_fallbacks"].as<JsonArrayConst>())
      g_sub_fallbacks.push_back(v.as<int>());
  }
  SearchSim sim;
""")
# steps は要らない。最後の地図だけ出す
i = s.index('  JsonArray steps = out["steps"].to<JsonArray>();')
j = s.index("  return jo.finish(0);", i)
s = s[:i] + """  out["n_steps"] = (int)sim.steps.size();
  out["fallback_used"] = g_sub_fallback_used;
  JsonArray fm = out["final_map"].to<JsonArray>();
  for (const auto v : sim.lgc->map)
    fm.add((int)v);
""" + s[j:]
open(f"{H}/px_search.cpp", "w").write(s)
# --- ビルド
AJ = f"{R}/build/_deps/arduinojson-src/src"
PS = f"{R}/tools/path_sim"
INC = f"-I{H}/inc -I{PS}/stub -I{PS} -I{R} -I{R}/include -I{R}/include/search -I{R}/include/action -I{AJ}"
FLAGS = f"-std=gnu++20 -O2 -w {INC}"
units = {
    "logic": f"{R}/src/search/logic.cpp", "adachi": f"{H}/adachi_px.cpp",
    "path_creator": f"{R}/src/action/path_creator.cpp", "trajectory_creator": f"{R}/src/action/trajectory_creator.cpp",
    "host_common": f"{PS}/host_common.cpp", "px_path": f"{H}/px_path.cpp", "px_search": f"{H}/px_search.cpp",
}
procs = [(n, subprocess.Popen(f"g++ {FLAGS} -c {src} -o {H}/obj/{n}.o", shell=True)) for n, src in units.items()]
for n, p in procs:
    if p.wait() != 0: sys.exit(f"compile failed: {n}")
common = " ".join(f"{H}/obj/{n}.o" for n in ["logic", "adachi", "path_creator", "trajectory_creator", "host_common"])
for exe in ["px_path", "px_search"]:
    subprocess.check_call(f"g++ {common} {H}/obj/{exe}.o -o {H}/{exe}", shell=True)
print("built")
