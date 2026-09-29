// 実験用: 重みパターンをたくさんまとめて評価する。path_run() の「右」の 1 パターンぶん
// (path_create → convert_large_path → diagonalPath → timebase_path_create)を patterns の数だけ回す。
// 入力: files, map, goals, exec, patterns [id...], vp_table [[6]...]
// 出力: results [{id, ok, time, base_time, ms, other, sig}]
#include <chrono>
#include <map>

#include "host_common.hpp"
#include "include/action/path_creator.hpp"
#include "include/ui.hpp"
float g_vp_table[512][6];


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

static size_t path_len(const std::vector<unsigned char> &t) {
  for (size_t i = 0; i < t.size(); i++)
    if (t[i] == 255 || t[i] == 0)
      return i + 1;
  return t.size();
}
template <class S, class T> static std::string sig_of(const S &ps, const T &pt) {
  std::vector<unsigned char> tv(pt.begin(), pt.end());
  const size_t n = std::min((size_t)ps.size(), path_len(tv));
  std::string s;
  for (size_t i = 0; i < n; i++) {
    s += std::to_string((int)std::lround(ps[i])) + ":" + std::to_string((int)tv[i]) + ",";
  }
  return s;
}

int main() {
  host::JsonOut jo;
  JsonDocument &out = jo.doc;
  JsonDocument in;
  if (!host::read_input(in, out))
    return jo.finish(1);
  if (in["vp_table"].is<JsonArrayConst>()) {
    int k = 0;
    for (JsonVariantConst row : in["vp_table"].as<JsonArrayConst>()) {
      int j = 0;
      for (JsonVariantConst v : row.as<JsonArrayConst>())
        g_vp_table[k][j++] = v.as<float>();
      k++;
    }
  }
  std::vector<int> patterns;
  for (JsonVariantConst v : in["patterns"].as<JsonArrayConst>())
    patterns.push_back(v.as<int>());
  const bool want_path = in["want_path"] | false;

  PathSim sim;
  sim.load_params();
  sim.load_turn_param_profiles();
  sim.exec_param_prof();
  const int exec = in["exec"] | 0;
  if (exec < 0 || exec >= (int)sim.exec_param_list.size()) {
    out["ok"] = false;
    out["error"] = "exec out of range";
    return jo.finish(1);
  }
  const auto ep = sim.exec_param_list[exec];
  std::vector<uint8_t> src;
  for (JsonVariantConst v : in["map"].as<JsonArrayConst>())
    src.push_back(v.as<int>() & 0xff);
  const std::vector<uint8_t> map = host::embed_maze(src, sim.sys_.maze_size, 0xff);
  if (map.empty()) {
    out["ok"] = false;
    out["error"] = "maze size";
    return jo.finish(1);
  }
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

  out["ok"] = true;
  JsonArray res = out["results"].to<JsonArray>();
  auto &pc = sim.pc;
  for (const int id : patterns) {
    JsonObject o = res.add<JsonObject>();
    o["id"] = id;
    const auto t0 = std::chrono::steady_clock::now();
    sim.lgc->set_param_num(id);
    pc->other_route_map.clear();
    const bool r = pc->path_create(false);
    if (!r) {
      o["ok"] = false;
      continue;
    }
    pc->convert_large_path(true);
    pc->diagonalPath(true, true);
    const int other = (int)pc->other_route_map.size();
    // 分岐を試す前の素の経路のタイム
    const float base_time = pc->calc_goal_time(sim.param_set, false);
    path_set_t p;
    p.type = id;
    p.time = 10000;
    pc->timebase_path_create(false, sim.param_set, p);
    o["ok"] = (bool)p.result;
    o["time"] = p.time;
    o["base_time"] = base_time;
    o["other"] = other;
    o["ms"] = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    o["sig"] = sig_of(p.path_s, p.path_t);
    if (want_path) {
      JsonArray s = o["path_s"].to<JsonArray>();
      JsonArray t = o["path_t"].to<JsonArray>();
      std::vector<unsigned char> tv(p.path_t.begin(), p.path_t.end());
      const size_t n = std::min((size_t)p.path_s.size(), path_len(tv));
      for (size_t i = 0; i < n; i++)
        s.add(p.path_s[i]), t.add((int)tv[i]);
    }
  }
  return jo.finish(0);
}
