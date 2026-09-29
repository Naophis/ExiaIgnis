#pragma once
// 壁の切れ目(壁あり → 急に遠のく)を形で検知する(2026-09-30)
//
// 壁ありで始まる WALL_OFF の従来の判定(wall_off_controller.cpp の detect_wall_off)は
// 「45° の距離が絶対しきい値 noexist_th(49mm)を越え、かつ増えている」で、普段の
// 壁ありの距離(46〜49mm)との余裕が 1〜3mm しかない。壁から少し離れて走ったり
// (WALL_OFF 開始時の壁 PD の段差で毎回 +1〜1.5mm 寄る)、暗い壁で読みが下がったり
// すると、壁が続いているのに発火した(20260930_002609 / 004825)。検知位置も
// 壁との距離で変わる(左 46mm と右 48.5mm で 3.5mm 違う)。
//
// ここでは値ではなく形で判定する(1 tick に 4 回読む wo サンプルを使う):
//   壁の距離(level): 少し前の区間 [x−win_far, x−win_near] のサンプルの中央値。
//                   level_max 以上なら壁なしとみなして判定しない。
//   発火: 最新のサンプルが level+depth 以上で、level+band を越えてからそこまでの
//         上昇が span mm 以内(ゆっくり離れるドリフトは弾く)、その間 min_run
//         サンプル以上続き、noise_back 以上戻っていない。
//   基準位置(edge_x): level+anchor_h を最初に越えた点(サンプル間を直線補間)。
//         発火位置ではなくここで旋回位置を決めるので、発火までの遅れは補正定数に
//         混ざらず、壁との距離にも左右されない。
//
// オフライン評価(scratchpad の edge_proto.py、同じ手順):
//   同じコースの反復走行(09-29、v≈2200): 検知位置の std 左 0.68→0.53mm、
//   右 0.89→0.47mm、32/32 発火、誤検知 2 件(002609 / 004825 の 1 回目)は発火しない。
//   09-1x〜09-30 の壁ありで始まる WALL_OFF 709 件: SLA_FRONT_STR の終わりまでに
//   十分遠のいた 689 件で 688 件発火、発火は従来より中央値 +2.3mm・p95 +8.5mm 遅い
//   (基準位置で差し引く)。level_max を 52mm にすると、壁から 51〜53mm 離れて
//   走った 9/14〜15 の左旋回を取りこぼした。
//
// 低速でも 1 tick に 4 回読むとサンプルが詰まる(400mm/s で 0.1mm 間隔)。前に入れた
// サンプルから min_dx 未満しか進んでいないサンプルは捨てて、バッファ(kBuf)が速度に
// よらず 32mm 以上を覚えているようにする。評価に使った走行(v≥1500mm/s で最小 0.35mm
// 間隔、古いログは 1 tick 1 点)では 1 点も捨てない。
//
// 発火したあとは、発火位置から win_far 進むまで(壁の距離を出す区間が発火後の
// サンプルだけになるまで)発火しない。以前は発火で履歴を消して再アームしていたが、
// 上昇の途中の読みだけで壁の距離を出し直し、同じ上昇でもう一度発火した
// (20260930_011727 / 011829 / 011852、4〜5 tick 後)。WALL_OFF をやり直したとき
// (再チェック)に 2 回目を拾う恐れがある。「読みが 壁の距離 + depth より近くへ
// 戻るまで」にすると、発火直後の読みの揺れ(51.3 → 50.6、011727)で解けてしまった。
// 壁の切れ目どうしは柱の間隔(90mm)以上離れているので、16mm 見ない区間は困らない。
//
// Pico SDK に依存しない純粋なクラス。Core1(SensorProcessor::update_wall_edge)が
// 毎 tick 更新し、sensing_result_entity_t::edge_l/r に公開、Core0
// (WallOffController::take_wall_edge)が読む。

#include <algorithm>
#include <cstdint>

struct WallEdgeParams {
  float win_far = 16.0f;    // [mm] 壁の距離を取る区間の遠い端(今の位置から)
  float win_near = 4.0f;    // [mm] 同、近い端(上昇の始まりを含めないため)
  float level_max = 60.0f;  // [mm] これ以上の壁の距離は壁なしとみなす
  int   min_n = 3;          // 壁の距離を出すのに要るサンプル数
  float depth = 4.0f;       // [mm] 壁の距離からこれ以上遠のいたら発火の候補
  float span = 10.0f;       // [mm] level+band を越えてから発火までの走行距離の上限
  float band = 1.0f;        // [mm] 上昇が始まったとみなす壁の距離からの差
  float noise_back = 1.0f;  // [mm] 上昇中にこれ以上戻ったら不採用
  float anchor_h = 3.0f;    // [mm] 基準位置 = level+anchor_h を越えた点
  int   min_run = 2;        // 上昇の連続サンプル数の下限(単発の跳ねを弾く)
  float min_dx = 0.25f;     // [mm] 前に入れたサンプルからこれ未満しか進んでいなければ捨てる
};

class WallEdgeDetector {
public:
  static constexpr int kBuf = 128;

  void arm() {
    n_ = 0;
    hold_ = false;
  }

  // 1 tick 分のサンプル(古い順、x は global_pos.dist 座標 [mm]、d は距離 [mm])を
  // 入れる。発火したら true を返して出力を更新する。
  bool update(const float *x, const float *d, int n, const WallEdgeParams &p) {
    if (n <= 0) return false;
    // 壁の距離はこの tick の最初のサンプルを基準に 1 回だけ求める(過去のサンプルから)
    bool level_ok = false;
    float level = 0.0f;
    {
      float tmp[kBuf];
      int cnt = 0;
      const float x0 = x[0];
      // 新しい方からたどり、区間より古くなったら打ち切る(x は増える一方)
      for (int i = n_ - 1; i >= 0; i--) {
        const float xi = X(i);
        if (xi < x0 - p.win_far) break;
        if (xi <= x0 - p.win_near) tmp[cnt++] = D(i);
      }
      if (cnt > 0 && cnt >= p.min_n) {
        std::nth_element(tmp, tmp + cnt / 2, tmp + cnt);
        level = tmp[cnt / 2];
        if (cnt % 2 == 0) {
          const float lo = *std::max_element(tmp, tmp + cnt / 2);
          level = 0.5f * (level + lo);
        }
        level_ok = level < p.level_max;
      }
    }
    bool fired = false;
    for (int q = 0; q < n; q++) {
      if (n_ > 0) {
        const float dx = x[q] - X(n_ - 1);
        if (dx < -kResetDx) {
          arm(); // 位置が大きく戻った = global_pos.dist のリセット
        } else if (dx < p.min_dx) {
          continue;
        }
      }
      push(x[q], d[q]);
      if (hold_) {
        if (x[q] < hold_until_x_) continue;
        hold_ = false;
      }
      if (level_ok && check(level, p)) {
        level_ = level;
        seq_++;
        hold_ = true;
        hold_until_x_ = fire_x_ + p.win_far;
        fired = true;
      }
    }
    return fired;
  }

  float edge_x() const { return edge_x_; }
  float fire_x() const { return fire_x_; }
  float level() const { return level_; }
  uint16_t seq() const { return seq_; }

private:
  static constexpr float kResetDx = 5.0f; // [mm]
  float xs_[kBuf] = {};
  float ds_[kBuf] = {};
  int head_ = 0; // 次に書く位置
  int n_ = 0;    // 有効なサンプル数
  float edge_x_ = 0.0f, fire_x_ = 0.0f, level_ = 0.0f;
  bool hold_ = false;       // 発火後、hold_until_x_ まで進むまで発火しない
  float hold_until_x_ = 0.0f;
  uint16_t seq_ = 0;

  // i = 0 が最古、n_-1 が最新
  int idx(int i) const { return (head_ - n_ + i + 2 * kBuf) % kBuf; }
  float X(int i) const { return xs_[idx(i)]; }
  float D(int i) const { return ds_[idx(i)]; }

  void push(float x, float d) {
    xs_[head_] = x;
    ds_[head_] = d;
    head_ = (head_ + 1) % kBuf;
    if (n_ < kBuf) n_++;
  }

  bool check(float level, const WallEdgeParams &p) {
    const int j = n_ - 1;
    if (D(j) < level + p.depth) return false;
    // level+band 以上が続く区間の始まり i を、span 以内で後ろへたどる
    int i = j;
    while (i - 1 >= 0 && D(i - 1) >= level + p.band && X(j) - X(i - 1) <= p.span) i--;
    if (i - 1 < 0) return false;                  // 上昇の始まりが見えていない
    if (D(i - 1) >= level + p.band) return false; // 上昇が span より長い = ゆっくり離れた
    if (j - i + 1 < p.min_run) return false;
    for (int k = i + 1; k <= j; k++) {
      if (D(k) < D(k - 1) - p.noise_back) return false;
    }
    // 基準位置: level+anchor_h を最初に越えた点(i-1 は level+band 未満 = 越える前)
    const float h = level + p.anchor_h;
    int k = i - 1;
    while (k + 1 <= j && D(k + 1) < h) k++;
    if (k + 1 > j) return false;
    const float d0 = D(k), d1 = D(k + 1), x0 = X(k), x1 = X(k + 1);
    edge_x_ = (d1 > d0) ? x0 + (h - d0) / (d1 - d0) * (x1 - x0) : x1;
    fire_x_ = X(j);
    return true;
  }
};
