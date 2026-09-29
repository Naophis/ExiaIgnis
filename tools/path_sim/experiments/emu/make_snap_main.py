#!/usr/bin/env python3
"""tools/path_sim/search_main.cpp に「update() の直前の状態を残す」処理を足した snap_main.cpp を作る"""
import os
H = os.path.dirname(os.path.abspath(__file__))
R = os.path.abspath(os.path.join(H, "..", "..", "..", ".."))
s = open(f"{R}/tools/path_sim/search_main.cpp").read()
def rep(s, old, new, n=1):
    assert s.count(old) == n, (old, s.count(old))
    return s.replace(old, new)
s = rep(s, "class SearchSim : public MainTaskCopy {\npublic:", """#include <cstdio>
static std::vector<uint8_t> g_snap;
static int g_snap_n = 0;
static void put32(int v) { for (int k = 0; k < 4; k++) g_snap.push_back((v >> (8 * k)) & 255); }
class SearchSim : public MainTaskCopy {
public:
  // update() を呼ぶ直前の状態(ゴール後だけ)を残す
  void dump_snapshot(int stationary) {
    if (!(adachi->goal_step && adachi->sm == SearchMode::ALL))
      return;
    g_snap_n++;
    for (const auto v : lgc->map) g_snap.push_back(v);
    put32(ego->x); put32(ego->y); put32(static_cast<int>(ego->dir)); put32(stationary);
    put32((int)adachi->subgoal_list.size());
    for (const auto &kv : adachi->subgoal_list) { put32((int)kv.first); put32((int)kv.second); }
    put32((int)adachi->pt_list.size());
    for (const auto &p : adachi->pt_list) { put32(p.x); put32(p.y); }
  }
""")
s = rep(s, "    adachi->update();\n    capture_route();\n    straight(", "    dump_snapshot(0);\n    adachi->update();\n    capture_route();\n    straight(")
s = rep(s, "    adachi->update();\n    capture_route();\n    sleep(5);", "    dump_snapshot(1);\n    adachi->update();\n    capture_route();\n    sleep(5);")
s = rep(s, "  sim.run(goals);\n", """  sim.run(goals);
  {
    std::vector<uint8_t> head;
    auto p32 = [&](int v) { for (int k = 0; k < 4; k++) head.push_back((v >> (8 * k)) & 255); };
    p32(sim.N); p32((int)goals.size());
    for (const auto &g : goals) { p32(g.x); p32(g.y); }
    p32(g_snap_n);
    FILE *f = fopen((in["snap_out"] | "snap.bin"), "wb");
    fwrite(head.data(), 1, head.size(), f);
    fwrite(g_snap.data(), 1, g_snap.size(), f);
    fclose(f);
  }
""")
open(f"{H}/snap_main.cpp", "w").write(s)
