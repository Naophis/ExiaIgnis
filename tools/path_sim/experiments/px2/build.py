#!/usr/bin/env python3
"""実験 2: コストを「続けて進んだ回数 → 値」の表にする(直進 16 段 / 斜め 16 段)。
   ファームのソースは読むだけ。g_mode: 0 = ファームのまま(3 段)、1 = 表を引く。
   g_relax: 0 = 最初に見つけた値で確定(ファームのまま)、1 = あとから安い値が来たら直す"""
import os, subprocess, sys
R = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", ".."))
H = os.path.dirname(os.path.abspath(__file__))
os.makedirs(f"{H}/inc", exist_ok=True)
os.makedirs(f"{H}/obj", exist_ok=True)
PX = os.path.join(os.path.dirname(H), "px")
def rep(s, old, new, cnt=1, n_expect=None):
    c = s.count(old)
    assert c >= 1, old
    if n_expect is not None: assert c == n_expect, (c, old)
    return s.replace(old, new, cnt if n_expect is None else c)
s = open(f"{R}/include/search/logic.hpp").read()
s = rep(s, "  void set_param() {\n", """  void set_param() {
    if (param_num >= 100) { // 実験用
      const float *v = g_vp_table[param_num - 100];
      for (int k = 0; k < 16; k++) { tabS[k] = v[k]; tabD[k] = v[16 + k]; }
      cell_size = v[0];
      St1 = v[0]; St2 = v[1]; St3 = v[3];
      Dia = v[16]; Dia2 = v[17]; Dia3 = v[19];
      use_tab = true;
      return;
    }
    use_tab = false;
""")
s = rep(s, "class MazeSolverBaseLgc", "extern float g_vp_table[512][32];\nextern int g_relax;\nclass MazeSolverBaseLgc")
s = rep(s, "  int param_num = 1;", "  int param_num = 1;\n  float tabS[16];\n  float tabD[16];\n  bool use_tab = false;")
open(f"{H}/inc/logic.hpp", "w").write(s)
# adachi.hpp は隣の logic.hpp を読むので、写しを inc に置いて書き換えた logic.hpp を読ませる
# (元のままだと、この翻訳単位だけ元のクラス配置で set_param() を展開し、重みが切り替わらない)
open(f"{H}/inc/adachi.hpp", "w").write(open(f"{R}/include/search/adachi.hpp").read())
s = open(f"{R}/src/search/logic.cpp").read()
old_s = """          if (v >= 2) {
            tmp += St3;
          } else if (v == 1) {
            tmp += St2;
          } else {
            tmp += St1;
          }
          if (tmp <= getDistV(X + i, Y + j, d2[k])) {
            if (!isUpdated(X + i, Y + j, d2[k])) {
              setDistV(X + i, Y + j, d2[k], tmp);
              vq_list.push(dir_pt_t{.x = (unsigned char)(X + i), .y = (unsigned char)(Y + j), .dir = d2[k], .dist2 = tmp});
              simplesort(tail);
              tail++;
              updateMapCheck(X + i, Y + j, d2[k]);
            }
          }
          addVector(X + i, Y + j, d[k], getVector(X, Y, d[k]));"""
new_s = """          if (use_tab) {
            tmp += tabS[getVector(X, Y, d[k])];
          } else if (v >= 2) {
            tmp += St3;
          } else if (v == 1) {
            tmp += St2;
          } else {
            tmp += St1;
          }
          if (g_relax) {
            if (tmp < getDistV(X + i, Y + j, d2[k])) {
              setDistV(X + i, Y + j, d2[k], tmp);
              vq_list.push(dir_pt_t{.x = (unsigned char)(X + i), .y = (unsigned char)(Y + j), .dir = d2[k], .dist2 = tmp});
              addVector(X + i, Y + j, d[k], getVector(X, Y, d[k]));
            }
          } else {
          if (tmp <= getDistV(X + i, Y + j, d2[k])) {
            if (!isUpdated(X + i, Y + j, d2[k])) {
              setDistV(X + i, Y + j, d2[k], tmp);
              vq_list.push(dir_pt_t{.x = (unsigned char)(X + i), .y = (unsigned char)(Y + j), .dir = d2[k], .dist2 = tmp});
              simplesort(tail);
              tail++;
              updateMapCheck(X + i, Y + j, d2[k]);
            }
          }
          addVector(X + i, Y + j, d[k], getVector(X, Y, d[k]));
          }"""
s = rep(s, old_s, new_s, n_expect=2)
old_d = old_s.replace("""          if (v >= 2) {
            tmp += St3;
          } else if (v == 1) {
            tmp += St2;
          } else {
            tmp += St1;
          }""", """          if (v == 2) {
            tmp += Dia3;
          } else if (v == 1) {
            tmp += Dia2;
          } else {
            tmp += Dia;
          }""")
new_d = new_s.replace("""          if (use_tab) {
            tmp += tabS[getVector(X, Y, d[k])];
          } else if (v >= 2) {
            tmp += St3;
          } else if (v == 1) {
            tmp += St2;
          } else {
            tmp += St1;
          }""", """          if (use_tab) {
            tmp += tabD[getVector(X, Y, d[k])];
          } else if (v == 2) {
            tmp += Dia3;
          } else if (v == 1) {
            tmp += Dia2;
          } else {
            tmp += Dia;
          }""")
s = rep(s, old_d, new_d, n_expect=2)
# 直す版: 取り出した値が古ければ捨てる
pop_old = """    Direction dir = now_pos.dir;
"""
pop_new = """    Direction dir = now_pos.dir;
    if (g_relax && now_pos.dist2 > getDistV(X, Y, dir))
      continue;
"""
s = rep(s, pop_old, pop_new, n_expect=2)
open(f"{H}/logic_px2.cpp", "w").write(s)
# adachi / search / path は px のものを 32 列の表に合わせて使う
s = open(f"{PX}/adachi_px.cpp").read()
# サブゴールを「タイム最小の経路(未知は壁なし扱い)の上の未知区画」から取る版
s = rep(s, "int g_sub_fallback_used = 0;", "int g_sub_fallback_used = 0;\nint g_sub_dp = 0; // 0 = 重みパターンだけ, 1 = 予備として, 2 = 毎回\nvoid (*g_dp_subgoals)(std::unordered_map<unsigned int, unsigned char> &) = nullptr;\nint g_dp_calls = 0;")
s = rep(s, """    {
      lgc->set_param_num(g_sub_primary);
      lgc->set_param();
      lgc->searchGoalPosition(true, subgoal_list);
      cost_mode = 3;
    }""", """    if (g_sub_dp == 2) {
      subgoal_list.clear();
      g_dp_subgoals(subgoal_list);
      g_dp_calls++;
      return;
    }
    {
      lgc->set_param_num(g_sub_primary);
      lgc->set_param();
      lgc->searchGoalPosition(true, subgoal_list);
      cost_mode = 3;
    }""")
s = rep(s, """        subgoal_fallback_done = true;
""", """        subgoal_fallback_done = true;
        if (g_sub_dp == 1) {
          g_dp_subgoals(subgoal_list);
          g_dp_calls++;
          g_sub_fallback_used++;
          return;
        }
""")
s = rep(s, """  if (goaled) {
    if (subgoal_list.size() == 0) {""", """  if (goaled && g_sub_dp == 3 && sm == SearchMode::ALL) {
    if (subgoal_list.empty()) {
      // 前に確かめてから地図が変わっていたら(帰り道で新しい壁を見た等)、もう一度確かめる
      int known = 0;
      for (const auto v : lgc->map)
        known += __builtin_popcount(v & 0xf0);
      static int known_at_check = -1;
      if (!subgoal_fallback_done || known != known_at_check) {
        subgoal_fallback_done = true;
        known_at_check = known;
        g_dp_subgoals(subgoal_list);
        g_dp_calls++;
        g_sub_fallback_used++;
      }
    } else {
      subgoal_fallback_done = false;
    }
  }
  if (goaled) {
    if (subgoal_list.size() == 0) {""")
s = rep(s, "        if (g_sub_dp == 1) {", "        if (g_sub_dp == 1 || g_sub_dp == 3) {")
# update() にかかった時間(PC 上)
s = rep(s, "int g_dp_calls = 0;", "int g_dp_calls = 0;\n#include <chrono>\ndouble g_update_ms = 0;\nint g_update_calls = 0;\nstruct UpdTimer { std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now(); ~UpdTimer() { g_update_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(); g_update_calls++; } };")
s = rep(s, "  if (goal_step && sm == SearchMode::ALL) {\n    if (subgoal_list.contains", "  if (goal_step && sm == SearchMode::ALL) {\n    UpdTimer upd_timer;\n    if (subgoal_list.contains")
open(f"{H}/adachi_px.cpp", "w").write(s)
for f in ["px_path.cpp", "px_search.cpp"]:
    s = open(f"{PX}/{f}").read()
    s = rep(s, "float g_vp_table[512][6];", "float g_vp_table[512][32];\nint g_relax = 0;")
    s = rep(s, '  if (in["vp_table"].is<JsonArrayConst>()) {', '  g_relax = in["relax"] | 0;\n  if (in["vp_table"].is<JsonArrayConst>()) {')
    if f == "px_search.cpp":
        s = rep(s, '#include "include/search/adachi.hpp"', '#include "adachi.hpp"')
        s = rep(s, '#include "include/action/path_creator.hpp"', '#define private public\n#include "include/action/path_creator.hpp"\n#undef private\n#include <chrono>\n#include <map>\n#include <queue>\n#include <unordered_map>')
        s = rep(s, "extern int g_sub_fallback_used;", """extern int g_sub_fallback_used;
extern int g_sub_dp;
extern void (*g_dp_subgoals)(std::unordered_map<unsigned int, unsigned char> &);
extern int g_dp_calls;
#include "opt_core.hpp"
#include "opt_core2.hpp"
static std::vector<param_set_t> g_psets;
static std::vector<point_t> g_goals0;
static MazeSolverBaseLgc *g_lgc0;
static PathCreator *g_pc0;
static int g_N0;
static double g_dp_ms = 0;
static void dp_subgoals(std::unordered_map<unsigned int, unsigned char> &list) {
  const auto t0 = std::chrono::steady_clock::now();
  for (auto &ps : g_psets) {
    tp::C = tp::Ctx{g_lgc0, g_pc0, &ps, true, true, g_dp_astar != 0, g_dp_weight};
    const auto t1 = std::chrono::steady_clock::now();
    const auto r = tp::solve(g_goals0, g_N0);
    g_dp_log.push_back({std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count(), (double)r.n_nodes, (double)r.n_edge, (double)r.n_exp, (double)r.heap_max, (double)r.time});
    if (!r.found)
      continue;
    Pos p{0, 0, 0};
    const std::string mv = "F" + r.moves; // (0,0) → (0,1) の 1 歩
    for (char c : mv) {
      const int d = c == 'F' ? p.d : c == 'R' ? (p.d + 1) % 4 : (p.d + 3) % 4;
      const int nx = p.x + DX[d], ny = p.y + DY[d];
      if (g_lgc0->is_unknown(p.x, p.y, DIRS[d]))
        list[nx + ny * g_N0] = 1;
      p = Pos{nx, ny, d};
    }
  }
  g_dp_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}""")
        s = rep(s, "  sim.run(goals);\n", """  g_dp_astar = in["dp_astar"] | 1;
  g_dp_weight = in["dp_weight"] | 1.0f;
  g_sub_dp = in["sub_dp"] | 0;
  g_dp_subgoals = dp_subgoals;
  g_goals0 = goals;
  g_lgc0 = sim.lgc.get();
  g_pc0 = &sim.pc;
  g_N0 = sim.N;
  if (g_sub_dp != 0) {
    MainTaskCopy loader;
    loader.load_params();
    loader.load_turn_param_profiles();
    loader.exec_param_prof();
    for (JsonVariantConst v : in["sub_execs"].as<JsonArrayConst>()) {
      const auto ep = loader.exec_param_list[v.as<int>()];
      loader.load_slalom_param(ep.fast_idx, ep.normal_idx, ep.slow_idx);
      loader.param_set.cell_size = loader.param_->cell;
      loader.param_set.start_offset = loader.param_->offset_start_dist;
      g_psets.push_back(loader.param_set);
    }
  }
  sim.run(goals);
""")
        s = rep(s, "extern int g_dp_calls;", "extern int g_dp_calls;\nextern double g_update_ms;\nextern int g_update_calls;")
        s = rep(s, '  out["n_steps"] = (int)sim.steps.size();', '  out["n_steps"] = (int)sim.steps.size();\n  out["update_ms"] = g_update_ms;\n  out["update_calls"] = g_update_calls;')
        # 止まっている間だけ確かめる版: 超信地(後退の後)と、スタートへ戻ったとき
        s = rep(s, """    adachi->update();
    sleep(5);""", """    adachi->update();
    if (g_sub_dp == 4 && adachi->goal_step && adachi->subgoal_list.empty()) {
      dp_subgoals(adachi->subgoal_list);
      g_dp_calls++;
    }
    sleep(5);""")
        s = rep(s, """      if (adachi->goal_step && ego->x == 0 && ego->y == 0) {
        end_reason = "home";
        break;
      }""", """      if (adachi->goal_step && ego->x == 0 && ego->y == 0) {
        if ((g_sub_dp == 4 || g_sub_dp == 5) && g_resorties < 8) {
          dp_subgoals(adachi->subgoal_list);
          g_dp_calls++;
          if (!adachi->subgoal_list.empty()) {
            g_resorties++;
            g_home_times.push_back(now);
            continue; // スタートから出直す
          }
        }
        end_reason = "home";
        break;
      }""")
        s = rep(s, "bool UserInterface::button_state_hold() { return false; }", "bool UserInterface::button_state_hold() { return false; }\nextern int g_sub_dp;\nextern int g_dp_calls;\nstatic void dp_subgoals(std::unordered_map<unsigned int, unsigned char> &list);\nstatic int g_dp_astar = 1;\nstatic float g_dp_weight = 1.0f;\nstatic std::vector<std::vector<double>> g_dp_log;\nstatic int g_resorties = 0;\nstatic std::vector<double> g_home_times;")
        s = rep(s, '  out["n_steps"] = (int)sim.steps.size();', '  out["n_steps"] = (int)sim.steps.size();\n  out["resorties"] = g_resorties;\n  if (!g_home_times.empty()) out["first_home"] = g_home_times[0];')
        s = rep(s, '  out["n_steps"] = (int)sim.steps.size();', '  out["n_steps"] = (int)sim.steps.size();\n  { JsonArray a = out["dp_log"].to<JsonArray>(); for (const auto &r : g_dp_log) { JsonArray b = a.add<JsonArray>(); for (double v : r) b.add(v); } }')
        s = rep(s, '  out["fallback_used"] = g_sub_fallback_used;', '  out["fallback_used"] = g_sub_fallback_used;\n  out["dp_calls"] = g_dp_calls;\n  out["dp_ms"] = g_dp_ms;')
    open(f"{H}/{f}", "w").write(s)
# path_creator: 分岐候補の幅と繰り返し回数を外から
s = open(f"{R}/src/action/path_creator.cpp").read()
s = rep(s, "static constexpr float OTHER_ROUTE_MARGIN_CELLS = 0.5f;", "#define OTHER_ROUTE_MARGIN_CELLS g_margin")
s = rep(s, '#include "include/action/path_creator.hpp"', '#include "include/action/path_creator.hpp"\nfloat g_margin = 0.5f;\nint g_iters = 5;')
s = rep(s, "  for (int i = 0; i < 5; i++) {\n    const auto before = other_route_map.size();", "  for (int i = 0; i < g_iters; i++) {\n    const auto before = other_route_map.size();")
s = rep(s, '#include "include/action/path_creator.hpp"', '#include "include/action/path_creator.hpp"\nstd::vector<float> g_raw_s, g_best_raw_s;\nstd::vector<unsigned char> g_raw_t, g_best_raw_t;')
s = rep(s, """      add_path_s(idx, 2);
      path_t.emplace_back(255);
      path_size = idx;
      return true;""", """      add_path_s(idx, 2);
      path_t.emplace_back(255);
      path_size = idx;
      g_raw_s = path_s;
      g_raw_t = path_t;
      return true;""")
s = rep(s, """            p.time = route.time;
            p.path_s.clear();""", """            p.time = route.time;
            g_best_raw_s = g_raw_s;
            g_best_raw_t = g_raw_t;
            p.path_s.clear();""")
open(f"{H}/path_creator_px2.cpp", "w").write(s)
s = open(f"{H}/px_path.cpp").read() if os.path.exists(f"{H}/px_path.cpp") else ""
for f in ["px_path.cpp"]:
    s = open(f"{H}/{f}").read()
    s = rep(s, "int g_relax = 0;", "int g_relax = 0;\nextern float g_margin;\nextern int g_iters;\nextern std::vector<float> g_best_raw_s;\nextern std::vector<unsigned char> g_best_raw_t;")
    s = rep(s, "    if (want_path) {", """    if (want_path) {
      JsonArray rs = o["raw_s"].to<JsonArray>();
      JsonArray rt = o["raw_t"].to<JsonArray>();
      for (const auto v : g_best_raw_s) rs.add(v);
      for (const auto v : g_best_raw_t) rt.add((int)v);""")
    s = rep(s, '  g_relax = in["relax"] | 0;', '  g_relax = in["relax"] | 0;\n  g_margin = in["margin"] | 0.5f;\n  g_iters = in["iters"] | 5;')
    open(f"{H}/{f}", "w").write(s)
AJ = f"{R}/build/_deps/arduinojson-src/src"
PS = f"{R}/tools/path_sim"
INC = f"-I{H}/inc -I{PS}/stub -I{PS} -I{R} -I{R}/include -I{R}/include/search -I{R}/include/action -I{AJ}"
FLAGS = f"-std=gnu++20 -O2 -w {INC}"
units = {
    "logic": f"{H}/logic_px2.cpp", "adachi": f"{H}/adachi_px.cpp",
    "path_creator": f"{H}/path_creator_px2.cpp", "trajectory_creator": f"{R}/src/action/trajectory_creator.cpp",
    "host_common": f"{PS}/host_common.cpp", "px_path": f"{H}/px_path.cpp", "px_search": f"{H}/px_search.cpp", "px_opt": f"{H}/px_opt.cpp",
}
procs = [(n, subprocess.Popen(f"g++ {FLAGS} -c {src} -o {H}/obj/{n}.o", shell=True)) for n, src in units.items()]
for n, p in procs:
    if p.wait() != 0: sys.exit(f"compile failed: {n}")
common = " ".join(f"{H}/obj/{n}.o" for n in ["logic", "adachi", "path_creator", "trajectory_creator", "host_common"])
for exe in ["px_path", "px_search", "px_opt"]:
    subprocess.check_call(f"g++ {common} {H}/obj/{exe}.o -o {H}/{exe}", shell=True)
print("built")
