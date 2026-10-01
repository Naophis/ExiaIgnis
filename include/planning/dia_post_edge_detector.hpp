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
// 縁 = 生値が thr を上から下へ切った点(サンプル間を直線補間)。手前で thr 以上が
// n_above サンプル以上か、2 サンプル以上で min_run mm 以上続き、thr を越えてからの山が
// thr × peak_ratio 以上のものだけ(停止前の上昇中の 1 サンプルの落ち込みや弱い反射を弾く)。
// 5000mm/s 近くでは壁のない柱の山(幅約 12mm)に 1 tick 5mm で 2 サンプルしか乗らず、
// 頂点も取り逃がす(500〜800 が 314)ので、サンプル数ではなく長さで見る(20261001_010716、
// 431 区間で組 563 → 579、遅い走行の残差は変わらず)。同じ側の縁は柱 2 本分(2·kPitch)の
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
// (ψ_g = ジャイロで見た右向きの向き)を添えて入れる。組ごとの (δ − c) を位置に対して
// 最小二乗で直線に当て、その傾きを ψ0(ジャイロの向きの基準と迷路の向きのずれ、+ は右)
// とする。最初は隣り合う組の傾きの平均にしていたが、平均は両端の組だけで決まり、速い域の
// 組のばらつき(0.5〜1mm)と減速に入った直後の組の +1.5mm で −0.15〜−0.41° に偏り、
// 制御がその分右へ切り続けて 0 を越えて +1mm 行き過ぎた(5600mm/s、20261001_013435〜
// 013543)。最小二乗なら同じ走行で −0.14〜+0.12°(制御なしの 5 本で −0.07〜+0.11°)。
// 今の横位置は、最後の組からジャイロ(と ψ0)で推測する(now_delta)。
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
// 組を作るたびに、直近の左右の同じ側の間隔(新しい縁とその 2 つ前 = s1、1 つ前の縁と
// その 2 つ前 = s2)から s を出して、縁の間隔 g を g/s に直す。両方あるときは
// s = 1.5·s1 − 0.5·s2(組の中点まで直線で先読み。s1 の区間の中心は組より半組前、s2 は
// 1 組半前)。減速中は倍率が組ごとに 0.955 → 0.93 → 0.90 と変わり続け、平均だと半組遅れて
// 減速に入った直後の組が約 +1.5mm ずれ、ψ0 を負へ偏らせていた。先読みで 5600mm/s の
// 制御なしの走行の残差 0.29〜0.76 → 0.17〜0.41mm(加速中・減速中それぞれでは 0.1〜0.15mm)。4000mm/s で直線を引いた後の残差 2.6〜2.9 → 1.2〜1.6mm、721mm の
// 直進 2.0 → 0.9mm、400mm/s でも 0.2 → 0.1mm 前後。組の前後の間隔を使った(後から見て
// 一番良い)s でも 0.9〜1.0mm までなので、残りは縮みの見積もりの遅れではない。
//
// 使わない組(2026-10-01): 倍率が信用できない組は、縁の記録と倍率の計算は続けるが、
// 横位置・ψ0・推測には使わず組として出さない。
//   ・前に同じ側の間隔が無いまま(倍率 1)、|加速度| > conf_accel でできた組(最初の組)
//   ・倍率を出した区間の縁の中で、加速度が +conf_accel 超と −conf_accel 未満の両方が
//     ある組(加速 → 減速の切り替わり。倍率が 1 組の間に 1.04 → 0.95 と変わる)
// 5600mm/s・721mm(加速 +3.8G / 減速 −4.9G)で最初の組と切り替わりの 3 組が外れ、
// 残差 1.0〜1.3 → 0.7〜0.96mm。切り替わりの組を外さないと、どの走行も減速の始めに
// +2〜4mm 跳ね、制御がそれを追った(20261001_012610 / 012628)。
//
// 直進の柱(2026-10-02、rel_thr > 0): 同じクラスを pair_pitch 0 / same_pitch 90 で使う。直進では
// 左右の柱が同じ位置にあるので δ = (x_R − x_L)/2 − k0 で、柱の間隔も走行距離の倍率も要らない
// (scale_fix 0)。縁は固定の生値ではなく「直前の山の rel_thr(50%)」を切る点: 柱の山は左
// 230〜240 / 右 130〜150 raw(距離 62mm 前後)で左右のゲインが違い、柱までの距離でも変わる
// ので、固定値だと切る位置の形が左右・距離で変わる。山に対する割合なら、ビームの形の
// たたみ込みで決まる縁の形が振幅によらず同じになる。直進の柱の後ろは 10〜20 raw まで
// 落ちるので、斜めで相対値を避けた理由(縁の後ろに次の壁が見える)は起きない。
// 壁の終わり(500 raw の段)の 50% を切る点は柱の山の 50% より約 5mm 手前(ビーム幅 σ≈9mm の
// モデルで 5.8mm、20261002_004431 で 左 6.4 / 右 3.9mm)なので別物として扱い、縁にしない:
// 山の手前 rise_max(78mm)の間の最小を下地(base)とし、山がその倍以上(contrast_min)で、頂点の
// 手前に下地近く(base + 30%)の読みがあった山だけ柱とみなす(壁は 78mm 以上続いて下地が無い。
// 柱の間の谷は寄った側で 10〜47 raw まで動くので絶対値ではなく下地からの割合)。
// 壁がある側はその壁の距離で横位置が分かるので壁制御に任せる。
// 20261002_004431(400mm/s、壁なしの柱 6 本、制御なし)で (x_R − x_L)/2 = −0.76〜−1.00、
// σ 0.11mm、直線を引いた後の残差 0.12mm。ツールは tools/param_tuner/str_post_edge.py。
//
// Pico SDK に依存しない純粋なクラス。Core1(SensorProcessor::update_dia_post_edge /
// update_str_post_edge)が毎 tick 左右のサンプルを入れ、sensing_result_entity_t::dia_post /
// str_post に公開する。

#include <cmath>
#include <cstdint>

struct DiaPostEdgeParams {
  float thr = 250.0f;       // [raw] 縁とみなす生値(rel_thr = 0 のとき)
  int n_above = 3;          // 縁の手前で thr 以上が続いたサンプル数(これ以上なら長さを問わない)
  float min_run = 2.0f;     // [mm] 2 サンプル以上のときの thr 以上の区間の長さの下限
  float peak_ratio = 1.2f;  // thr を越えてからの山 / thr の下限
  float tol = 15.0f;        // [mm] 柱の間隔からの許容
  float kappa = 0.0f;       // [mm/rad] 向きによる読みのずれ(δ_読み = δ_車軸 + κ·ψ)
  int scale_fix = 1;        // 1 = 直近の同じ側の間隔から走行距離の縮みを出して直す
  float conf_accel = 4900.0f; // [mm/s^2] これを超える加減速のとき、倍率が信用できない組を使わない
  // 柱の並び(既定は斜め)。直進は pair_pitch 0(左右の柱が同じ位置)・same_pitch 90。
  float pair_pitch = 63.63961f; // [mm] 反対側の縁との間隔(90/√2)
  float same_pitch = 2.0f * 63.63961f; // [mm] 同じ側の縁の間隔(2·pair_pitch)
  int gate_kmax = 0;            // 同じ側の間隔が same_pitch × これを超えたら整数倍の確認をしない(0 = 常に確認)
  // 相対しきい値(直進の柱、2026-10-02)。0 より大きいと、縁 = 生値が「下地 + rel_thr·(山 − 下地)」
  // を上から下へ切った点(update_rel)。n_above / min_run / peak_ratio / thr は使わない。
  float rel_thr = 0.0f;
  float peak_min = 0.0f;   // [raw] 山の下限(側ごとに距離から換算して渡す)
  // 山の手前の下地(base)= 頂点の手前 rise_max の間の最小(1mm ごとの最小の輪)。柱の判定に使う:
  // 山は (山 − base)/山 ≥ contrast_min のときだけ柱(壁は 78mm 以上続いて下地が無い)、
  // x_low = 頂点の手前で base + low_ratio·(山 − base) 以下だった最後の位置(柱の間の谷は寄った側で
  // 10〜47 raw まで動く(013141)ので、絶対値ではなく下地からの割合)。しきい値は rel_thr·山。
  float low_ratio = 0.3f;
  float contrast_min = 0.5f;
  float rise_max = 78.0f;  // [mm] x_low から縁まで(柱 25〜60)。下地を見る窓の長さでもある
  float fall_max = 30.0f;  // [mm] 山の頂点から縁まで
  // ψ0 の事前値(直進)。最小二乗に重み psi0_w [mm^2] で傾き psi0_prior を足す(組が 1 つなら
  // そのまま事前値、組が増えるほど組の傾きへ寄る。90^2 で組 1 つ分の重み)。0 で使わない
  float psi0_w = 0.0f;
  float psi0_prior = 0.0f; // [rad]
};

class DiaPostEdgeDetector {
public:
  static constexpr float kPitch = 63.63961f; // 90/√2 [mm]
  static constexpr int kRing = 96;           // rel_thr > 0 の下地の輪(1mm ごと)。rise_max 以上
  enum Side { LEFT = 0, RIGHT = 1 };

  void arm() {
    for (auto &s : s_) s = SideState{};
    has_last_ = false;
    n_pairs_ = 0;
    n_hist_ = 0;
    scale_ = 1.0f;
    n_skip_ = 0;
    eps_deg_ = 0.0f;
    n_psi_ = 0;
    psi0_ = 0.0f;
    c_pos_ = 0.0f;
    ls_n_ = 0;
    ls_x0_ = ls_sx_ = ls_sy_ = ls_sxx_ = ls_sxy_ = 0.0f;
    prior_w_ = 0.0f;
    prior_b0_ = 0.0f;
  }

  // 壁から横位置と向きを引き継ぐ(直進、2026-10-02)。両壁の区間が終わるときに、壁の距離から
  // 出した横位置(読みの座標 δ_wall + k0)と、壁に沿って走っていたときのジャイロの向きから出した
  // ψ0 を入れる。横位置は組 1 つ分として(n_pairs 1、seq +1)、ψ0 は最小二乗の事前値として
  // (重み w。その後の柱の組で寄っていく)。最初の柱の組まで(90mm 以上)制御が無い区間と、
  // 組 2 つまで ψ0 が無い区間(組が 1 つのときの次の組の推測の誤差 +1.2〜+5.5mm)をなくす。
  void seed(float x, float c, float psi, float delta_read, float psi0, float w, float kappa) {
    kappa_ = kappa;
    delta_ = delta_read;
    delta_gyro_ = delta_read - kappa * psi;
    pos_ = x;
    c_pos_ = c;
    if (n_pairs_ < 1) n_pairs_ = 1;
    seq_++;
    set_psi0_prior(psi0, w);
  }
  // ψ0 の事前値(重み w [mm^2])。組が 1 つのときはそのまま使い、組が増えると組の傾きへ寄る
  void set_psi0_prior(float b0, float w) {
    prior_b0_ = b0;
    prior_w_ = w;
    recompute_psi0();
  }

  // 1 サンプル入れる。x は読んだ時刻の位置 [mm](global_pos.dist 基準)、y は生値、
  // c はその位置までのジャイロの向きの積分 [mm·rad]、psi はそのときのジャイロの向き
  // [rad](どちらも右向きが +)。左右の組ができた(横位置が更新された)ら true。
  bool update(Side side, float x, float y, float c, float psi, const DiaPostEdgeParams &p) {
    if (p.rel_thr > 0.0f) return update_rel(side, x, y, c, psi, p);
    SideState &s = s_[side];
    bool paired = false;
    const bool long_enough =
        s.above >= p.n_above || (s.above >= 2 && s.prev_x - s.first_x >= p.min_run);
    if (s.has_prev && s.prev_y >= p.thr && y < p.thr && long_enough &&
        s.peak >= p.thr * p.peak_ratio) {
      const float f = (s.prev_y - p.thr) / (s.prev_y - y);
      const float e = s.prev_x + (x - s.prev_x) * f;
      const float ec = s.prev_c + (c - s.prev_c) * f;
      const float epsi = s.prev_psi + (psi - s.prev_psi) * f;
      if (accept_same_side(s, e, p)) {
        take_edge(side, e, ec, epsi, p, paired);
      }
    }
    if (y >= p.thr) {
      if (s.above == 0) s.first_x = x;
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

  // 相対しきい値の縁(直進の柱、rel_thr > 0)。山(peak)は最後の縁(か再アーム)からの最大、
  // 下地(base)は頂点の手前 rise_max の間の最小(1mm ごとの最小の輪 ring_ から、頂点が更新された
  // ときに求め直す)。縁 = rel_thr·peak を上から下へ切った点。切ったら(採っても採らなくても)
  // 山を捨てる(壁の終わりもここで捨てる)。採るのは、山が peak_min 以上で下地より十分高く
  // (contrast_min)、頂点から fall_max 以内に切り、頂点の手前 rise_max 以内に下地近く
  // (base + low_ratio·(peak − base) 以下)の読みがあったものだけ。
  bool update_rel(Side side, float x, float y, float c, float psi, const DiaPostEdgeParams &p) {
    SideState &s = s_[side];
    bool paired = false;
    if (s.has_prev && s.peak >= p.peak_min) {
      // しきい値は山の割合そのまま(下地を引いて割合を取る形も試したが、下地が 78mm の間の
      // 1mm ごとの最小=ノイズの下端で、004431 の残差 0.12 → 0.27mm と悪くなった)
      const float thr = p.rel_thr * s.peak;
      if (s.prev_y >= thr && y < thr) {
        const float f = (s.prev_y - thr) / (s.prev_y - y);
        const float e = s.prev_x + (x - s.prev_x) * f;
        const float ec = s.prev_c + (c - s.prev_c) * f;
        const float epsi = s.prev_psi + (psi - s.prev_psi) * f;
        if (s.has_low && s.peak - s.base >= p.contrast_min * s.peak && e - s.peak_x <= p.fall_max &&
            e - s.x_low <= p.rise_max && accept_same_side(s, e, p)) {
          take_edge(side, e, ec, epsi, p, paired);
        }
        s.peak = 0.0f;
      }
    }
    // 1mm ごとの最小の輪
    {
      const int bin = (int)std::floor(x);
      const int i = ((bin % kRing) + kRing) % kRing;
      if (s.ring_bin[i] != bin) {
        s.ring_bin[i] = bin;
        s.ring_y[i] = y;
      } else if (y < s.ring_y[i]) {
        s.ring_y[i] = y;
      }
    }
    if (y > s.peak) {
      s.peak = y;
      s.peak_x = x;
      // 下地と x_low を求め直す(頂点の手前 rise_max の間)
      const int pb = (int)std::floor(x);
      float base = y;
      for (int b = pb - 1; b >= pb - (int)p.rise_max; b--) {
        const int i = ((b % kRing) + kRing) % kRing;
        if (s.ring_bin[i] == b && s.ring_y[i] < base) base = s.ring_y[i];
      }
      s.base = base;
      const float lvl = base + p.low_ratio * (y - base);
      s.has_low = false;
      for (int b = pb - 1; b >= pb - (int)p.rise_max; b--) {
        const int i = ((b % kRing) + kRing) % kRing;
        if (s.ring_bin[i] == b && s.ring_y[i] <= lvl) {
          s.has_low = true;
          s.x_low = (float)b + 1.0f; // その 1mm の終わり
          break;
        }
      }
    }
    s.has_prev = true;
    s.prev_x = x;
    s.prev_y = y;
    s.prev_c = c;
    s.prev_psi = psi;
    return paired;
  }

  // 今の目標の加速度 [mm/s^2]。縁に添えて、倍率が信用できない組を見分けるのに使う。
  void set_accel(float a) { accel_ = a; }

  // その側の縁の途中状態(直前のサンプル・thr 以上の区間)を捨てる。読み方(S0 だけ /
  // S1〜S3)が変わったときに呼ぶ。S0 は同じ位置でも S1〜S3 より低く出るので、混ぜると
  // 境目で thr をまたいだように見えることがある。採った縁の履歴は残す。
  void reset_side(Side side) {
    SideState &s = s_[side];
    s.has_prev = false;
    s.above = 0;
    s.peak = 0.0f;
    // 下地の輪・has_low / x_low は残す(位置は同じ座標)
  }

  // 位置 x(ジャイロの向きの積分 c)での横位置の推測 [mm]。最後の組から、ジャイロで
  // 見た向きと ψ0 で進める。n_pairs() >= 1 のときだけ意味がある。
  float now_delta(float x, float c) const {
    float d = delta_gyro_ - kappa_ * psi0_ + (c - c_pos_);
    if (n_psi_ >= 1) d += psi0_ * (x - pos_);
    return d;
  }
  // 同、センサーの読みの座標で(車軸の横位置 + κ × いまの迷路に対する向き ψ_g + ψ0)。
  // 「いま柱の組ができたらこう読める」値。直進の制御(str_post_ctrl)はこちらを使う: 壁の
  // 45° の読みも向きで κ_w·ψ 動き、既存の壁制御はその分(向きの変化が横位置の変化より
  // 先に見える = 減衰)込みで調整されているので、同じ形で渡す(2026-10-02)。
  float now_delta_read(float x, float c, float psi_now) const {
    return now_delta(x, c) + kappa_ * (psi_now + psi0_);
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
  int n_skip() const { return n_skip_; }      // 使わなかった組の数(arm してから)
  float psi0() const { return psi0_; }        // 向きのずれ ψ0 [rad](+ は右、組ごとの平均)
  int n_psi() const { return n_psi_; }        // ψ0 が使える(≥ 1)。事前値なしなら組の数 − 1、ありなら組の数 + 1

private:
  struct SideState {
    bool has_prev = false;
    float prev_x = 0.0f;
    float prev_y = 0.0f;
    float prev_c = 0.0f;
    float prev_psi = 0.0f;
    int above = 0;      // 今 thr 以上が続いているサンプル数
    float first_x = 0.0f; // その区間の最初のサンプルの位置
    float peak = 0.0f;  // thr を越えてからの最大(rel_thr > 0 では最後の縁からの最大)
    float peak_x = 0.0f; // その位置(rel_thr > 0)
    float base = 0.0f;   // rel_thr > 0: 頂点の手前 rise_max の間の最小(下地)
    bool has_low = false; // rel_thr > 0: 頂点の手前に下地近くの読みがあった
    float x_low = 0.0f;   // その最後の位置
    bool has_edge = false;
    float edge_x = 0.0f; // 最後に採った縁
    int ring_bin[kRing];  // rel_thr > 0: 1mm ごとの最小の輪(位置 [mm] の整数部と、その中の最小)
    float ring_y[kRing];
    SideState() { for (int i = 0; i < kRing; i++) { ring_bin[i] = -1000000; ring_y[i] = 0.0f; } }
  };

  void take_edge(Side side, float e, float ec, float epsi, const DiaPostEdgeParams &p, bool &paired) {
    SideState &s = s_[side];
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

  static bool accept_same_side(const SideState &s, float e, const DiaPostEdgeParams &p) {
    if (!s.has_edge) return true;
    const float g = e - s.edge_x;
    const float k = std::round(g / p.same_pitch);
    if (p.gate_kmax > 0 && k > (float)p.gate_kmax) return true;
    return k >= 1.0f && std::fabs(g - k * p.same_pitch) <= p.tol;
  }

  // 採った縁の履歴(新しい順)。走行距離の縮みを出すのに使う。
  void push_hist(Side side, float e) {
    for (int i = 3; i > 0; i--) hist_[i] = hist_[i - 1];
    hist_[0] = {e, side, accel_};
    if (n_hist_ < 4) n_hist_++;
  }
  // 新しい縁 hist_[0] と 1 つ前 hist_[1] の組に使う走行距離の倍率。左右交互に並んで
  // いれば、同じ側の間隔 (0 − 2) と (1 − 3) の平均 / 2·kPitch、片方だけなら (0 − 2)。
  // used には使った間隔の数(0〜2)を返す。
  float local_scale(const DiaPostEdgeParams &p, int &used) const {
    const float two = p.same_pitch;
    used = 0;
    if (!(n_hist_ >= 3 && hist_[2].side == hist_[0].side)) return 1.0f;
    const float sp1 = hist_[0].x - hist_[2].x;
    if (std::fabs(sp1 - two) > p.tol) return 1.0f;
    used = 1;
    const float s1 = sp1 / two;
    if (n_hist_ >= 4 && hist_[3].side == hist_[1].side) {
      const float sp2 = hist_[1].x - hist_[3].x;
      if (std::fabs(sp2 - two) <= p.tol) {
        used = 2;
        return 1.5f * s1 - 0.5f * (sp2 / two);
      }
    }
    return s1;
  }
  // 倍率が信用できる組か(上の「使わない組」)。used = local_scale が使った間隔の数。
  bool confident(int used, const DiaPostEdgeParams &p) const {
    if (p.conf_accel <= 0.0f) return true;
    if (used == 0) return std::fabs(hist_[0].accel) <= p.conf_accel;
    const int last = (used >= 2) ? 3 : 2; // 倍率を出すのに使った一番古い縁
    bool pos = false, neg = false;
    for (int i = 0; i <= last; i++) {
      pos |= hist_[i].accel > p.conf_accel;
      neg |= hist_[i].accel < -p.conf_accel;
    }
    return !(pos && neg);
  }

  bool pair(Side side, float e, float ec, float epsi, const DiaPostEdgeParams &p) {
    if (!has_last_ || last_side_ == side) return false;
    const float g_raw = e - last_x_;
    if (std::fabs(g_raw - p.pair_pitch) > p.tol) return false;
    int used = 0;
    const float sc = p.scale_fix ? local_scale(p, used) : 1.0f;
    if (p.scale_fix && !confident(used, p)) {
      n_skip_++;
      return false;
    }
    scale_ = sc;
    const float g = g_raw / scale_;
    const float d = (last_side_ == RIGHT) ? (p.pair_pitch - g) * 0.5f : (g - p.pair_pitch) * 0.5f;
    const float mid = 0.5f * (e + last_x_);
    const float cmid = 0.5f * (ec + last_c_);
    // 2 つの縁の時刻のジャイロの向きの分だけ直す(κ·ψ0 は今の横位置を出すときに引く)
    const float dg = d - p.kappa * 0.5f * (epsi + last_psi_);
    kappa_ = p.kappa;
    if (n_pairs_ >= 1 && mid > pos_) {
      eps_deg_ = std::atan2(d - delta_, mid - pos_) * (180.0f / 3.14159265f);
    }
    // ψ0 = (δ − c) の位置に対する最小二乗の傾き(位置は最初の組から測る)
    if (ls_n_ == 0) ls_x0_ = mid;
    {
      const float lx = mid - ls_x0_;
      const float ly = dg - cmid;
      ls_n_++;
      ls_sx_ += lx;
      ls_sy_ += ly;
      ls_sxx_ += lx * lx;
      ls_sxy_ += lx * ly;
      recompute_psi0();
    }
    delta_ = d;
    delta_gyro_ = dg;
    pos_ = mid;
    c_pos_ = cmid;
    n_pairs_++;
    seq_++;
    return true;
  }

  // 最小二乗の傾き。事前値があれば Σ(y − a − b x)^2 + w (b − b0)^2 の最小化
  // (b = (n Sxy − Sx Sy + n w b0) / (n Sxx − Sx^2 + n w))。事前値が無いときは組が 2 つから。
  void recompute_psi0() {
    const float n = (float)ls_n_;
    const float den = n * ls_sxx_ - ls_sx_ * ls_sx_;
    const float num = n * ls_sxy_ - ls_sx_ * ls_sy_;
    if (prior_w_ > 0.0f) {
      if (ls_n_ >= 1) {
        psi0_ = (num + n * prior_w_ * prior_b0_) / (den + n * prior_w_);
      } else {
        psi0_ = prior_b0_;
      }
      n_psi_ = ls_n_ + 1;
    } else if (ls_n_ >= 2 && den > 1e-3f) {
      psi0_ = num / den;
      n_psi_ = ls_n_ - 1;
    }
  }

  struct Hist {
    float x = 0.0f;
    Side side = LEFT;
    float accel = 0.0f; // その縁を採ったときの目標の加速度
  };
  SideState s_[2];
  Hist hist_[4];
  int n_hist_ = 0;
  float scale_ = 1.0f;
  int n_skip_ = 0;
  float accel_ = 0.0f;
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
  // ψ0 の最小二乗の和(x = 組の位置 − 最初の組の位置、y = δ_gyro − c)
  int ls_n_ = 0;
  float ls_x0_ = 0.0f, ls_sx_ = 0.0f, ls_sy_ = 0.0f, ls_sxx_ = 0.0f, ls_sxy_ = 0.0f;
  float prior_w_ = 0.0f;  // ψ0 の事前値の重み [mm^2](0 = 無し)
  float prior_b0_ = 0.0f; // ψ0 の事前値 [rad]
};
