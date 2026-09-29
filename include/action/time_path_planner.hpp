#pragma once
#ifndef TIME_PATH_PLANNER_H
#define TIME_PATH_PLANNER_H

#include <cstdint>
#include <vector>

#include "action/path_creator.hpp"

// タイム最小の経路探索(2026-09-29)。
//
// 経路を「直線 + ターン」の区間の列として、区間を辺にした最短経路問題を解く。辺の重みは
// PathCreator::calc_segment_time()(calc_goal_time() の 1 区間ぶん)そのものなので、求めた
// 経路の calc_goal_time() が、この迷路・この走行パラメータで作れる経路の中で最小になる。
// 重みパターン(set_param_num 1〜5)も、分岐を 1 つずつ試す timebase_path_create() も使わない。
//
// 節点 = ターンを終えた位置・向き(入った区画と進行方向)・直進中か斜め中か・そのときの速度・
//        次の区間への約束
// 辺   = 直進を f 区画(斜めなら a 歩)+ ターン 1 個
//
// 「約束」は calc_segment_time() の先読みのため: ターンの前後に直線があり、次のターンが
// Large / Orval なら、そのターンは速いパラメータ(map_fast)で計算される。辺を出す時点では
// 次が分からないので「速いターンとして計算した / していない」を節点に持ち、次の辺がそれに
// 合うときだけつなぐ。
//
// 辺の作り方は convert_large_path() / diagonalPath() の規則の写し:
//   ターン 1 個 = Large、同じ向き 2 個 = Orval
//   向きが交互に続く組 = 斜め。入口は Dia45(最初が同じ向き 2 個なら Dia135)、出口も同じ。
//   途中の同じ向き 2 個 = Dia90
//   同じ向き 3 個(その場で 270° 回る形)は Normal が残るので扱わない
// 変換の規則を変えたら、ここも合わせること(tools/path_sim/check_time_path.py が、求めた
// 経路を変換と calc_goal_time() に通して同じタイムになるかを確かめる)。
//
// 使うメモリは solve() の間だけ確保する(節点 1 個 20 バイト前後)。空きが足りない・上限を
// 超えたときは失敗を返すので、呼んだ側は従来の経路生成へ戻る。
class TimePathPlanner {
public:
  enum class Result : unsigned char {
    Ok = 0,
    NoPath,   // ゴールへ行ける経路が無い
    NoMemory, // 空きメモリが足りず始められない
    Overflow, // 節点・ヒープ・速度の種類が上限を超えた
    Aborted,  // ボタンで中断
  };
  struct Stats {
    float time = 0;     // 求めた経路のタイム(calc_goal_time() と同じ値)
    int nodes = 0;      // 使った節点
    int node_cap = 0;   // 節点の上限
    int heap_max = 0;   // ヒープの最大
    int edges = 0;      // 試した辺
    int seg_cached = 0; // 覚えた区間のタイム
    int mem_bytes = 0;  // 確保したメモリ
    int free_bytes = 0; // 始めるときの空きメモリ(機体だけ。ホストでは 0)
  };

  // 成功したら pc.path_s / pc.path_t / pc.path_size に素の経路(path_create() と同じ形)を
  // 入れる。呼んだ側が convert_large_path() / diagonalPath() を続けること。
  // 通るのは既知の区画だけ(path_create(false) と同じ)。ゴールは pc.lgc の goal_list。
  Result solve(PathCreator &pc, param_set_t &p_set);

  Stats stats;
  // 節点の上限。0 なら空きメモリから決める(機体)/ 既定値(ホスト)
  int node_cap_request = 0;

  static const char *result_str(Result r);

private:
  struct Node {
    float cost;
    uint32_t key : 24;
    uint32_t run : 7;  // 前の節点からの直進 / 斜めの数
    uint32_t done : 1;
    uint16_t prev;
    uint8_t prim; // 前の節点からの区間の終わりのターン(path_t の番号)
    uint8_t pad;
  };
  struct HeapE {
    float cost;
    uint16_t id;
  };
  struct SegE {
    uint32_t key; // 0 = 空き。下位 4bit は区間を終えたときの速度の番号
    float time;
  };
  struct Pos {
    int8_t x, y, d;
  };
  struct Seg {
    float time;
    int v;
    bool ok;
  };

  static constexpr int NONE = 0xffff;
  static constexpr int V_MAX = 16;        // 速度の種類の上限
  static constexpr int SEG_CAP = 2048;    // 区間のタイムを覚える表の大きさ(2 の累乗)。実績は最大 960
  static constexpr int ABORT_CHECK = 256; // 節点をこの数だけ広げるごとにボタンを見る

  PathCreator *pc_ = nullptr;
  MazeSolverBaseLgc *lgc_ = nullptr;
  param_set_t *ps_ = nullptr;
  int n_ = 0;

  std::vector<Node> nodes_;
  std::vector<uint16_t> table_; // 節点のハッシュ表(開番地)
  uint32_t table_mask_ = 0;
  std::vector<HeapE> heap_;
  int heap_cap_ = 0;
  std::vector<SegE> seg_;
  std::vector<uint8_t> open_; // 区画ごとの「通れる向き」(bit0..3 = N E S W)と bit7 = ゴール
  float vlist_[V_MAX];
  int n_v_ = 0;
  bool overflow_ = false;

  float best_goal_ = 0;
  int best_prev_ = NONE;
  int best_run_ = 0;
  int best_prim_ = 0;

  void release();
  int v_index(float v);
  Seg seg_raw(bool first, bool dia, int s, int tcode, int vin, bool fast_next,
              int next_tcode);
  Seg seg(bool first, bool dia, int s, int tcode, int vin, bool fast_next,
          int next_tcode);

  int find_node(uint32_t key);
  void heap_push(float cost, int id);
  HeapE heap_pop();

  bool can_move(const Pos &p, int m, Pos &q) const;
  bool goal_at(const Pos &p) const;
  bool meets(uint32_t key, int s, int tcode) const;
  void relax(int from, float cost, int run, int prim, const Pos &p, int kind,
             int v, int b, int nt);
  void finish(int from, float cost, int run, int prim);
  void emit(int from, bool first, bool dia, int s, int tcode, int run,
            const Pos &land, int land_kind);
  void emit_final_turn(int from, bool first, bool dia, int s, int tcode,
                       int run);
  void expand(int id);
  void write_path();
};

#endif
