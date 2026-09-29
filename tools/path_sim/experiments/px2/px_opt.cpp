// 実験 3: タイムそのもの(calc_goal_time と同じ計算)を最小にする経路を、区間(直線 + ターン)を
// 辺にした最短経路探索で求める。ファームの経路変換(convert_large_path / diagonalPath)の規則を
// 辺の作り方に写してあり、求めた経路は「素の経路 → ファームの変換 → calc_goal_time」に通して
// 同じタイムになるかを確かめる(check)。
// 入力: files, map, goals, exec, [raw_paths: [{s:[], t:[]}...]](ファームの変換とタイム計算だけ通す)
// 出力: opt {time, raw_s, raw_t, fw_time, fw_s, fw_t, nodes, edges, ms}, evals [...]
#include <chrono>
#include <map>
#include <queue>
#include <unordered_map>

#include "host_common.hpp"
#define private public
#include "include/action/path_creator.hpp"
#undef private
#include "include/ui.hpp"
float g_vp_table[512][32];
int g_relax = 0;

bool UserInterface::button_state() { return false; }
bool UserInterface::button_state_hold() { return false; }

class PathSim : public MainTaskCopy {
public:
  std::shared_ptr<MazeSolverBaseLgc> lgc = std::make_shared<MazeSolverBaseLgc>();
  std::shared_ptr<PathCreator> pc = std::make_shared<PathCreator>();
  std::shared_ptr<UserInterface> ui_ = std::make_shared<UserInterface>();
  void setup_maze(const std::vector<uint8_t> &map, const std::vector<point_t> &goals) {
    lgc->init(sys_.maze_size, sys_.maze_size * sys_.maze_size - 1);
    lgc->set_goal_pos(goals);
    pc->set_logic(lgc);
    pc->set_userinterface(ui_);
    for (int i = 0; i < (int)map.size(); i++)
      lgc->set_native_wall_data(i, map[i]);
  }
};

#include "opt_core.hpp"
#include "opt_core2.hpp"
int main() {
  host::JsonOut jo;
  JsonDocument &out = jo.doc;
  JsonDocument in;
  if (!host::read_input(in, out))
    return jo.finish(1);
  PathSim sim;
  sim.load_params();
  sim.load_turn_param_profiles();
  sim.exec_param_prof();
  const int exec = in["exec"] | 0;
  const auto ep = sim.exec_param_list[exec];
  std::vector<uint8_t> src;
  for (JsonVariantConst v : in["map"].as<JsonArrayConst>())
    src.push_back(v.as<int>() & 0xff);
  const std::vector<uint8_t> map = host::embed_maze(src, sim.sys_.maze_size, 0xff);
  std::vector<point_t> goals;
  for (JsonVariantConst g : in["goals"].as<JsonArrayConst>()) {
    point_t p{};
    convertFromJson(g, p);
    goals.push_back(p);
  }
  sim.setup_maze(map, goals);
  sim.load_slalom_param(ep.fast_idx, ep.normal_idx, ep.slow_idx);
  sim.param_set.cell_size = sim.param_->cell;
  sim.param_set.start_offset = sim.param_->offset_start_dist;
  N = sim.sys_.maze_size;
  is_goal.assign(N * N, 0);
  for (const auto &g : goals)
    is_goal[g.x + g.y * N] = 1;
  CTX = OptCtx{sim.lgc.get(), sim.pc.get(), &sim.param_set, (bool)(in["search"] | false)};
  out["ok"] = true;

  // ファームの経路(素)を通すだけ
  if (in["raw_paths"].is<JsonArrayConst>()) {
    JsonArray ev = out["evals"].to<JsonArray>();
    for (JsonVariantConst rp : in["raw_paths"].as<JsonArrayConst>()) {
      std::vector<float> rs;
      std::vector<int> rt;
      for (JsonVariantConst v : rp["s"].as<JsonArrayConst>())
        rs.push_back(v.as<float>());
      for (JsonVariantConst v : rp["t"].as<JsonArrayConst>())
        rt.push_back(v.as<int>());
      const auto e = fw_eval(rs, rt);
      JsonObject o = ev.add<JsonObject>();
      o["fw_time"] = e.time;
      o["model_time"] = model_time(e.s, e.t);
      if (in["debug"] | false) {
        JsonArray a = o["fw_seg"].to<JsonArray>();
        for (size_t i = 0; i < CTX.pc->path_time_s.size(); i++) {
          JsonArray b = a.add<JsonArray>();
          b.add(e.s[i]); b.add(e.t[i]); b.add(CTX.pc->path_time_s[i]); b.add(CTX.pc->path_time_t[i]); b.add(CTX.pc->path_time_total[i].v_start); b.add(CTX.pc->path_time_total[i].v_end); b.add(CTX.pc->path_time_total[i].total_time);
        }
        JsonArray m = o["model_seg"].to<JsonArray>();
        float v = 0, time = 0; bool dia = false;
        for (size_t i = 0; i < e.t.size(); i++) {
          bool FN = false; int nt = 255;
          if (i + 1 < e.t.size()) { nt = e.t[i + 1]; const auto ntt = TC.get_turn_type(nt); FN = (0.5 * e.s[i + 1] - 1) > 0 && (ntt == TurnType::Orval || ntt == TurnType::Large); }
          const auto so = seg_cost(i == 0, dia, e.s[i], e.t[i], v, FN, nt);
          time += so.time;
          JsonArray b = m.add<JsonArray>();
          b.add(e.s[i]); b.add(e.t[i]); b.add(so.time); b.add(v); b.add(so.v_now); b.add(time); b.add(FN);
          v = so.v_now; dia = so.dia;
          if (e.t[i] == 255 || e.t[i] == 0) break;
        }
      }
    }
  }
  g_count_final = in["count_final"] | true;
  if (in["trace"].is<const char *>()) {
    const std::string mv = in["trace"].as<std::string>();
    const int st = get_node(Pos{0, 1, 0}, 3, 0.0f, 0, 0);
    const bool ok = trace(st, 0, mv, 0, 0);
    JsonObject o = out["trace"].to<JsonObject>();
    o["ok"] = ok;
    o["reached"] = (int)g_trace_best;
    o["len"] = (int)mv.size();
    o["info"] = g_trace_info;
    o["cost"] = g_trace_cost;
    return jo.finish(0);
  }
  if (in["opt"] | true) {
    const int impl = in["impl"] | 1;
    const int reps = in["reps"] | 1;
    tp::C = tp::Ctx{sim.lgc.get(), sim.pc.get(), &sim.param_set, (bool)(in["search"] | false), g_count_final, (bool)(in["astar"] | false), in["weight"] | 1.0f};
    double cold_ms = 0;
    if (impl == 2) {
      const auto tc0 = std::chrono::steady_clock::now();
      tp::solve(goals, sim.sys_.maze_size); // 1 回目は区間のタイムの表を埋める
      cold_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tc0).count();
    }
    const auto t0 = std::chrono::steady_clock::now();
    OptResult r;
    long exp2 = 0;
    int heap2 = 0;
    if (impl == 2) {
      tp::Result r2;
      for (int k = 0; k < reps; k++)
        r2 = tp::solve(goals, sim.sys_.maze_size);
      r.found = r2.found; r.time = r2.time; r.moves = r2.moves; r.n_nodes = r2.n_nodes; r.n_edge = r2.n_edge;
      exp2 = r2.n_exp;
      out["seg_filled"] = r2.seg_filled;
      out["n_speeds"] = r2.n_speeds;
      heap2 = r2.heap_max;
    } else {
      r = opt_solve(goals, sim.sys_.maze_size);
    }
    JsonObject o = out["opt"].to<JsonObject>();
    o["ms"] = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / (impl == 2 ? reps : 1);
    o["cold_ms"] = cold_ms;
    o["expanded"] = exp2;
    o["heap_max"] = heap2;
    o["nodes"] = r.n_nodes;
    o["edges"] = r.n_edge;
    if (!r.found) {
      o["found"] = false;
    } else {
      o["found"] = true;
      o["time"] = r.time;
      const std::string mv = r.moves;
      o["moves"] = mv;
      std::vector<float> rs;
      std::vector<int> rt;
      raw_from_moves(mv, rs, rt);
      const auto e = fw_eval(rs, rt);
      o["fw_time"] = e.time;
      o["model_time"] = model_time(e.s, e.t);
      o["n_turns"] = (int)e.t.size() - 1;
      JsonArray s = o["path_s"].to<JsonArray>();
      JsonArray t = o["path_t"].to<JsonArray>();
      for (size_t i = 0; i < e.t.size(); i++)
        s.add(e.s[i]), t.add(e.t[i]);
    }
  }
  return jo.finish(0);
}
