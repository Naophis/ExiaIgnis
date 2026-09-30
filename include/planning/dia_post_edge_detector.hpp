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
// 向きのずれ ψ0(2026-10-01): 各サンプルにジャイロの向きの積分 c = ∫ψ_g ds [mm·rad]
// (ψ_g = ジャイロで見た右向きの向き)を添えて入れる。組と組の間で
//   ε = Δδ / Δpos(迷路基準の右向きの向き)、g = Δc / Δpos(同じ区間のジャイロの向きの平均)
// から ψ0 = ε − g(ジャイロの向きの基準と迷路の向きのずれ、+ は右)を出し、組ごとの
// 平均を持つ。今の横位置は、最後の組からジャイロ(と ψ0)で推測する(now_delta)。
// 向きを ±1.5° 動かした走行(09-20 の斜め制御あり 4 本)で、δ の変化とジャイロの
// 向きの積分の係数は +0.68〜+1.51(平均 1.0、期待値 +1)。v400 で向きが ±0.1° しか
// 動かない走行では、組ごとの δ のばらつき(0.22mm)の方が大きい。
//
// 向きによる読みのずれ κ(2026-10-01): 縁から出した δ は、車軸の横位置ではなく、
// センサーが車軸より前にある分と光の向きが回る分だけ、向き ψ(右向き +)に比例して動く:
//   δ_読み = δ_車軸 + κ·ψ
// 斜め制御ありで向きを ±1.5° 動かした 4 本(20261001_004338〜004501)で、直線への当てはめ
// では κ ≈ +100mm/rad(残差 0.30 → 0.15mm、走行ごとの ψ0 が制御なしの走行とそろう)、
// 次の組が来たときの推測の誤差では 75 が最小(斜め制御あり 0.71 → 0.31mm、制御なし
// 0.67 → 0.42mm)。1.4° 向きを切ると読みが 2mm 前後動くので、補正しないと ψ0 と推測が狂う
// (実機で ψ0 が +0.77° まで振れ、横位置が −1.2mm で止まった)。組ごとに縁の時刻の
// ジャイロの向きで直した値(delta_gyro)で ψ0 を出し、今の横位置(now_delta)を出すときに
// だけ κ·ψ0 も引く。κ·ψ0 は全部の組に同じだけ乗るので ψ0 の差分には要らない。ψ0 の
// 推定に κ·ψ0 を入れると、ψ0 の変化を自分で拾って 1 組ごとに κ/63.6 = 1.57 倍で跳ね
// 返り、推測が発散した(ホスト再生で推測の誤差 rms 0.67 → 2.48mm)。
// delta() は向きの補正前の値(オフラインのツールと同じ)、delta_axle() が車軸の横位置。
//
// 走行距離の縮み(2026-10-01): 加減速の空転・ロックで、走行距離は加速中 4〜6.5% 長く、
// 減速中 5〜9% 短く出る(4000mm/s・吸引あり、20261001_010257〜010544。同じ側の柱の
// 間隔が本来の 127.28mm に対して 116〜135mm)。距離が s 倍だと、右 → 左の組と左 → 右の
// 組で δ に ±(1 − s)·kPitch/2 の逆向きの誤差が出て交互に跳ねる(4000mm/s で ±2mm)。
// 組を作るたびに、直近の左右の同じ側の間隔(新しい縁とその 2 つ前、1 つ前の縁とその
// 2 つ前)の平均から s を出して、縁の間隔 g を g/s に直す(左右の平均なので横に動いた
// 分は打ち消し合う)。4000mm/s で直線を引いた後の残差 2.6〜2.9 → 1.2〜1.6mm、721mm の
// 直進 2.0 → 0.9mm、400mm/s でも 0.2 → 0.1mm 前後。組の前後の間隔を使った(後から見て
// 一番良い)s でも 0.9〜1.0mm までなので、残りは縮みの見積もりの遅れではない。
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
  float kappa = 0.0f;       // [mm/rad] 向きによる読みのずれ(δ_読み = δ_車軸 + κ·ψ)
  int scale_fix = 1;        // 1 = 直近の同じ側の間隔から走行距離の縮みを出して直す
};

class DiaPostEdgeDetector {
public:
  static constexpr float kPitch = 63.63961f; // 90/√2 [mm]
  enum Side { LEFT = 0, RIGHT = 1 };

  void arm() {
    for (auto &s : s_) s = SideState{};
    has_last_ = false;
    n_pairs_ = 0;
    n_hist_ = 0;
    scale_ = 1.0f;
    eps_deg_ = 0.0f;
    n_psi_ = 0;
    psi0_ = 0.0f;
    c_pos_ = 0.0f;
  }

  // 1 サンプル入れる。x は読んだ時刻の位置 [mm](global_pos.dist 基準)、y は生値、
  // c はその位置までのジャイロの向きの積分 [mm·rad]、psi はそのときのジャイロの向き
  // [rad](どちらも右向きが +)。左右の組ができた(横位置が更新された)ら true。
  bool update(Side side, float x, float y, float c, float psi, const DiaPostEdgeParams &p) {
    SideState &s = s_[side];
    bool paired = false;
    if (s.has_prev && s.prev_y >= p.thr && y < p.thr && s.above >= p.n_above &&
        s.peak >= p.thr * p.peak_ratio) {
      const float f = (s.prev_y - p.thr) / (s.prev_y - y);
      const float e = s.prev_x + (x - s.prev_x) * f;
      const float ec = s.prev_c + (c - s.prev_c) * f;
      const float epsi = s.prev_psi + (psi - s.prev_psi) * f;
      if (accept_same_side(s, e, p)) {
        s.has_edge = true;
        s.edge_x = e;
        push_hist(side, e);
        paired = pair(side, e, ec, epsi, p);
        has_last_ = true;
        last_side_ = side;
        last_x_ = e;
        last_c_ = ec;
        last_psi_ = epsi;
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
    s.prev_c = c;
    s.prev_psi = psi;
    return paired;
  }

  // 位置 x(ジャイロの向きの積分 c)での横位置の推測 [mm]。最後の組から、ジャイロで
  // 見た向きと ψ0 で進める。n_pairs() >= 1 のときだけ意味がある。
  float now_delta(float x, float c) const {
    float d = delta_gyro_ - kappa_ * psi0_ + (c - c_pos_);
    if (n_psi_ >= 1) d += psi0_ * (x - pos_);
    return d;
  }

  uint16_t seq() const { return seq_; }       // 組ができるたびに +1(arm でも戻さない)
  int n_pairs() const { return n_pairs_; }    // arm してからの組の数
  float delta() const { return delta_; }      // 最後の組の横位置 [mm](+ は右、向きの補正前)
  // 同、車軸の横位置に直したもの(縁の時刻のジャイロの向きと、いまの ψ0 で)
  float delta_axle() const { return delta_gyro_ - kappa_ * psi0_; }
  float pos() const { return pos_; }          // 最後の組の位置(2 つの縁の中点)[mm]
  // 最後の 2 組から出した向き [deg](+ は右向き)。n_pairs() >= 2 のときだけ有効
  float eps_deg() const { return eps_deg_; }
  float edge_x(Side side) const { return s_[side].edge_x; }
  float scale() const { return scale_; }      // 最後の組で使った走行距離の倍率(1 = 直していない)
  float psi0() const { return psi0_; }        // 向きのずれ ψ0 [rad](+ は右、組ごとの平均)
  int n_psi() const { return n_psi_; }        // ψ0 を出した回数(= 組の数 − 1)

private:
  struct SideState {
    bool has_prev = false;
    float prev_x = 0.0f;
    float prev_y = 0.0f;
    float prev_c = 0.0f;
    float prev_psi = 0.0f;
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

  // 採った縁の履歴(新しい順)。走行距離の縮みを出すのに使う。
  void push_hist(Side side, float e) {
    for (int i = 3; i > 0; i--) hist_[i] = hist_[i - 1];
    hist_[0] = {e, side};
    if (n_hist_ < 4) n_hist_++;
  }
  // 新しい縁 hist_[0] と 1 つ前 hist_[1] の組に使う走行距離の倍率。左右交互に並んで
  // いれば、同じ側の間隔 (0 − 2) と (1 − 3) の平均 / 2·kPitch、片方だけなら (0 − 2)。
  float local_scale(const DiaPostEdgeParams &p) const {
    const float two = 2.0f * kPitch;
    float sum = 0.0f;
    int n = 0;
    if (n_hist_ >= 3 && hist_[2].side == hist_[0].side) {
      const float sp = hist_[0].x - hist_[2].x;
      if (std::fabs(sp - two) <= p.tol) { sum += sp; n++; }
    }
    if (n == 1 && n_hist_ >= 4 && hist_[3].side == hist_[1].side) {
      const float sp = hist_[1].x - hist_[3].x;
      if (std::fabs(sp - two) <= p.tol) { sum += sp; n++; }
    }
    return (n > 0) ? sum / (n * two) : 1.0f;
  }

  bool pair(Side side, float e, float ec, float epsi, const DiaPostEdgeParams &p) {
    if (!has_last_ || last_side_ == side) return false;
    const float g_raw = e - last_x_;
    if (std::fabs(g_raw - kPitch) > p.tol) return false;
    scale_ = p.scale_fix ? local_scale(p) : 1.0f;
    const float g = g_raw / scale_;
    const float d = (last_side_ == RIGHT) ? (kPitch - g) * 0.5f : (g - kPitch) * 0.5f;
    const float mid = 0.5f * (e + last_x_);
    const float cmid = 0.5f * (ec + last_c_);
    // 2 つの縁の時刻のジャイロの向きの分だけ直す(κ·ψ0 は今の横位置を出すときに引く)
    const float dg = d - p.kappa * 0.5f * (epsi + last_psi_);
    kappa_ = p.kappa;
    if (n_pairs_ >= 1 && mid > pos_) {
      const float ds = mid - pos_;
      eps_deg_ = std::atan2(d - delta_, ds) * (180.0f / 3.14159265f);
      const float sample = (dg - delta_gyro_) / ds - (cmid - c_pos_) / ds;
      n_psi_++;
      psi0_ += (sample - psi0_) / (float)n_psi_;
    }
    delta_ = d;
    delta_gyro_ = dg;
    pos_ = mid;
    c_pos_ = cmid;
    n_pairs_++;
    seq_++;
    return true;
  }

  struct Hist {
    float x = 0.0f;
    Side side = LEFT;
  };
  SideState s_[2];
  Hist hist_[4];
  int n_hist_ = 0;
  float scale_ = 1.0f;
  bool has_last_ = false;
  Side last_side_ = LEFT;
  float last_x_ = 0.0f;
  float last_c_ = 0.0f;
  float last_psi_ = 0.0f;
  int n_pairs_ = 0;
  uint16_t seq_ = 0;
  float delta_ = 0.0f;
  float delta_gyro_ = 0.0f; // ジャイロの向きの分だけ直した横位置
  float kappa_ = 0.0f;
  float pos_ = 0.0f;
  float eps_deg_ = 0.0f;
  int n_psi_ = 0;
  float psi0_ = 0.0f;
  float c_pos_ = 0.0f;  // 最後の組の位置でのジャイロの向きの積分
};
