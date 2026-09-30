#pragma once
// 斜め走行の横位置を、45° センサーが柱を過ぎて落ちる位置から出す(2026-10-01)
//
// 従来の斜め制御(ControlLaw::check_sen_error_dia)は柱までの距離の読みを基準にしているが、
// 同じ柱の読みが壁の有無で変わる(柱の横に壁が付くと光を返す面が増えて近く見える。
// 20260930_222045 / 222320: 45° で +5〜6mm、90° で +14〜35mm、左右とも同時に遠い)。
// ここでは読みの大きさではなく、読みが落ちる「位置」を使う:
//
//   斜めでは柱が左右交互に kPitch(= 90/√2 = 63.64mm)ごとに来る。45° LED1 の生値は
//   柱を通り過ぎた瞬間に落ちる(壁があってもなくても同じ出来事)。右へ δ ずれると
//   右の縁は +δ、左の縁は −δ 動くので、隣り合う縁の間隔 g から
//     右 → 左: δ = (kPitch − g)/2     左 → 右: δ = (g − kPitch)/2
//   旋回の出口の前後のずれ・読んだ時刻の遅れ・速度の遅れは左右に同じだけ入って消える。
//
// 縁 = 生値が thr を上から下へ切った点(サンプル間を直線補間)。手前 n_above サンプル
// 以上が thr 以上で、thr を越えてからの山が thr × peak_ratio 以上のものだけ(停止前の
// 上昇中の 1 サンプルの落ち込みや弱い反射を弾く)。同じ側の縁は柱 2 本分(2·kPitch)の
// 整数倍 ± tol、組にする反対側の縁は直前の縁で kPitch ± tol のときだけ。
// 固定しきい値は、山と谷の間の 50% や傾きが最大の点より壁の有無に強かった(壁ありでは
// 左の縁の後ろに次の壁が約 120 raw 見えていて、相対値や傾きがそれに引っ張られる)。
//
// オフライン(tools/param_tuner/dia_post_edge.py、同じ手順): v400 の斜め 320mm で
// 直線を引いた後の残差 0.1〜0.3mm、壁あり/なし各 7 本(dia135 右)で δ の差 0.35mm
// (ばらつき 0.6〜0.9mm の中)。4000〜5640mm/s でも柱ごとに縁が取れるが、走行距離が
// 空転・ロックで同じ側の間隔 116〜132mm になった走行は δ が ±3mm 狂う。
// 横位置の定義(k0 = 左右センサーの取り付け差)はまだ引いていない。
//
// Pico SDK に依存しない純粋なクラス。Core1(SensorProcessor::update_dia_post_edge)が
// 毎 tick 左右 1 サンプルずつ入れ、sensing_result_entity_t::dia_post に公開する。

#include <cmath>
#include <cstdint>

struct DiaPostEdgeParams {
  float thr = 250.0f;       // [raw] 縁とみなす生値
  int n_above = 3;          // 縁の手前で thr 以上が続いたサンプル数の下限
  float peak_ratio = 1.5f;  // thr を越えてからの山 / thr の下限
  float tol = 15.0f;        // [mm] 柱の間隔からの許容
};

class DiaPostEdgeDetector {
public:
  static constexpr float kPitch = 63.63961f; // 90/√2 [mm]
  enum Side { LEFT = 0, RIGHT = 1 };

  void arm() {
    for (auto &s : s_) s = SideState{};
    has_last_ = false;
    n_pairs_ = 0;
    eps_deg_ = 0.0f;
  }

  // 1 サンプル入れる。x は読んだ時刻の位置 [mm](global_pos.dist 基準)、y は生値。
  // 左右の組ができた(横位置が更新された)ら true。
  bool update(Side side, float x, float y, const DiaPostEdgeParams &p) {
    SideState &s = s_[side];
    bool paired = false;
    if (s.has_prev && s.prev_y >= p.thr && y < p.thr && s.above >= p.n_above &&
        s.peak >= p.thr * p.peak_ratio) {
      const float e = s.prev_x + (x - s.prev_x) * (s.prev_y - p.thr) / (s.prev_y - y);
      if (accept_same_side(s, e, p)) {
        s.has_edge = true;
        s.edge_x = e;
        paired = pair(side, e, p);
        has_last_ = true;
        last_side_ = side;
        last_x_ = e;
      }
    }
    if (y >= p.thr) {
      s.above++;
      if (y > s.peak) s.peak = y;
    } else {
      s.above = 0;
      s.peak = 0;
    }
    s.has_prev = true;
    s.prev_x = x;
    s.prev_y = y;
    return paired;
  }

  uint16_t seq() const { return seq_; }       // 組ができるたびに +1(arm でも戻さない)
  int n_pairs() const { return n_pairs_; }    // arm してからの組の数
  float delta() const { return delta_; }      // 最後の組の横位置 [mm](+ は右)
  float pos() const { return pos_; }          // 最後の組の位置(2 つの縁の中点)[mm]
  // 最後の 2 組から出した向き [deg](+ は右向き)。n_pairs() >= 2 のときだけ有効
  float eps_deg() const { return eps_deg_; }
  float edge_x(Side side) const { return s_[side].edge_x; }

private:
  struct SideState {
    bool has_prev = false;
    float prev_x = 0.0f;
    float prev_y = 0.0f;
    int above = 0;      // 今 thr 以上が続いているサンプル数
    float peak = 0.0f;  // thr を越えてからの最大
    bool has_edge = false;
    float edge_x = 0.0f; // 最後に採った縁
  };

  static bool accept_same_side(const SideState &s, float e, const DiaPostEdgeParams &p) {
    if (!s.has_edge) return true;
    const float g = e - s.edge_x;
    const float k = std::round(g / (2.0f * kPitch));
    return k >= 1.0f && std::fabs(g - k * 2.0f * kPitch) <= p.tol;
  }

  bool pair(Side side, float e, const DiaPostEdgeParams &p) {
    if (!has_last_ || last_side_ == side) return false;
    const float g = e - last_x_;
    if (std::fabs(g - kPitch) > p.tol) return false;
    const float d = (last_side_ == RIGHT) ? (kPitch - g) * 0.5f : (g - kPitch) * 0.5f;
    const float mid = 0.5f * (e + last_x_);
    if (n_pairs_ >= 1 && mid > pos_) {
      eps_deg_ = std::atan2(d - delta_, mid - pos_) * (180.0f / 3.14159265f);
    }
    delta_ = d;
    pos_ = mid;
    n_pairs_++;
    seq_++;
    return true;
  }

  SideState s_[2];
  bool has_last_ = false;
  Side last_side_ = LEFT;
  float last_x_ = 0.0f;
  int n_pairs_ = 0;
  uint16_t seq_ = 0;
  float delta_ = 0.0f;
  float pos_ = 0.0f;
  float eps_deg_ = 0.0f;
};
