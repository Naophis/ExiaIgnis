#pragma once
// 柱の谷(下に凸)検知器 (2026-09-23)
//
// 壁なし開始の WALL_OFF では、注視側 45° センサー(LED1)が「柱に近づいて読みが
// 下がり(谷)、柱を過ぎて急に遠のく」形を見せる。従来は谷底の絶対値
// (exist_dist_r2=60 等)で柱を判定していたが、谷底は横位置とサンプリング位相で
// 60〜70mm にばらつき(1.8mm/tick で 6mm の柱は 3〜4 サンプルしか無い)、
// 取りこぼすと 25mm 走行の安全網(detect_pass_through_case2)に落ちて旋回が
// 20mm 前後遅れた(20260923_064918.csv idx2203〜2218、同じターンで正常検知した
// 走行と旋回突入時の前壁距離が 16〜25mm 違う)。
//
// ここでは値ではなく形で判定する:
//   (1) 下降: 谷底(追跡中の最小値)が、その手前 stale_dist 以内の最大値(ピーク)より
//       depth_min 以上低い。180(範囲外)からの接近もピークとして数える。谷底は
//       bottom_min〜bottom_max の帯(遠い柱は実測 53〜70mm)。旋回直後の SLA_BACK_STR
//       や直線の出だしで「今曲がった角の柱」が 32〜38mm の近い谷として見え、これも
//       急上昇するので bottom_min=48 で除外する(近い物は exist 経路の担当)。ピークを「直前 stale_dist 以内」に限るのは、壁(180→47→
//       長い平坦)の末尾で古い 180 を引きずって深さが出てしまい、壁切れ直後に
//       始まる WALL_OFF で壁端を柱として誤アンカーするのを防ぐため。壁の平坦は
//       ノイズで谷底が更新され続けるので stale 再アームでは消えない。
//   (2) 離脱(早期): 走行 1mm あたりの上昇率が slope_min 以上を 2tick 連続、
//       かつ谷底から rise_min 以上上がり、両点灯 ch も上昇している。
//   (3) 離脱(確認): far_th 以上へ抜けた(前 tick で既に rise_min 以上上がり、
//       両点灯 ch も上昇)。
//   (4) 谷底から max_lag より進んでも離脱しなければ柱ではない(壁の長い平坦や、
//       谷が古い場合を除外)。谷底が stale_dist より古くなれば再アーム。
// 発火位置ではなく谷底位置(bottom_x)をアンカーにするので、発火までの遅れは
// 補正定数(wall_off_pillar_str_l/r)に混ざらない。
//
// 首振れ(姿勢制御のヨー)耐性: 45° 読みのヨー感度は幾何で ≈ D/57 mm/deg
// (65mm の柱で約 1.1mm/deg)。直線中に観測された最大ヨーは 1.5°/tick なので
// 1tick の変化は ≤1.7mm(壁読みの |Δd| は p99 で 1.8mm/tick)。(2) は 1800mm/s で
// ≥2.7mm/tick を 2 連続、(3) は +30mm 以上を要求するので首振れでは満たせない。
// 首振れ由来の浅い谷(実測 4.7mm、20260923_064338.csv idx410、0.8°/tick で
// 振れている最中)は (1) の depth_min=8 で棄却される。
//
// Pico SDK に依存しない純粋なクラス。tests/pillar_trough_host で実ログを再生
// してホスト検証できる。Core1(SensorProcessor)が毎 tick 更新し、結果を
// sensing_result_entity_t::pillar_l/r に公開、Core0(WallOffController::
// take_pillar_trough)が読む。

struct PillarTroughParams {
  float depth_min = 8.0f;    // [mm] 谷底がその前のピークよりこれ以上低い
  float bottom_min = 48.0f;  // [mm] 谷底の下限(これより近い谷は今曲がった角の柱や壁 = exist 経路の担当)
  float bottom_max = 80.0f;  // [mm] 谷底の上限(これより遠い谷は柱と見なさない)
  float slope_min = 1.5f;    // [mm/mm] 離脱の上昇率(走行 1mm あたり)、2tick 連続
  float rise_min = 4.0f;     // [mm] 谷底からの上昇量(早期条件/確認条件の前 tick)
  float far_th = 100.0f;     // [mm] 確認条件: ここまで抜けたら離脱確定
  float max_lag = 14.0f;     // [mm] 谷底からこれ以内に離脱しなければ柱ではない
  float stale_dist = 30.0f;  // [mm] 谷底がこれより古ければ再アーム(発火済みも消す)。
                             //      ピークを探す窓(谷底の手前この距離以内)にも使う
  float min_travel = 0.5f;   // [mm] 1tick 走行の下限(低速でのノイズ増幅防止)
};

class PillarTroughDetector {
public:
  enum State : int {
    IDLE = 0,
    TRACKING = 1,
    FIRED_EARLY = 2,
    FIRED_CONFIRM = 3,
  };

  // 現在値で再アーム(谷底 = ピーク = d)。SLALOM 中など形が意味を持たない区間では
  // 毎 tick 呼ぶ(既存の sen.*.sensor_dist の SLALOM リセットと同じ考え方)。
  void arm(float d, float x) {
    push(x, d);
    state_ = TRACKING;
    bottom_ = d;
    bottom_x_ = x;
    peak_ = d;
    prev_d_ = d;
    has_prev_ = true;
    slope_prev_ = -1.0e9f;
  }

  // 1tick 更新。
  //   d          注視側 45° LED1 距離 [mm](範囲外は sensor_range_max=180)
  //   d2_diff    両点灯 ch の 1tick 差分 [mm](上昇していることを確認に使う)
  //   x          global 走行距離 [mm](global_pos.dist)
  //   travel     この tick の走行 [mm]
  //   fire_enable 発火を許可するか(低速時は false にして追跡だけ続ける)
  void update(float d, float d2_diff, float x, float travel, bool fire_enable,
              const PillarTroughParams &p) {
    if (state_ == IDLE || !has_prev_) {
      arm(d, x);
      return;
    }
    const float dx = (travel > p.min_travel) ? travel : p.min_travel;
    const float slope = (d - prev_d_) / dx;
    push(x, d);

    if (state_ == TRACKING) {
      if (d < bottom_) {
        bottom_ = d;
        bottom_x_ = x;
        peak_ = window_max(x, p.stale_dist); // 谷底の手前 stale_dist 以内の最大値
      }
    }

    // 谷底が古い: 発火済みでも消して現在値から追跡し直す。下降中は谷底が
    // 毎 tick 更新されるのでここには来ない。
    if ((x - bottom_x_) > p.stale_dist) {
      arm(d, x);
      return;
    }

    if (state_ == TRACKING && fire_enable) {
      const float lag = x - bottom_x_;
      const float rise = d - bottom_;
      const bool shape_ok = (bottom_ >= p.bottom_min) &&
                            (bottom_ <= p.bottom_max) &&
                            ((peak_ - bottom_) >= p.depth_min) &&
                            (lag <= p.max_lag);
      if (shape_ok && d2_diff > 0.0f) {
        if (rise >= p.rise_min && slope >= p.slope_min &&
            slope_prev_ >= p.slope_min) {
          fire(FIRED_EARLY, x);
        } else if (d >= p.far_th && (prev_d_ - bottom_) >= p.rise_min) {
          fire(FIRED_CONFIRM, x);
        }
      }
    }

    prev_d_ = d;
    slope_prev_ = slope;
  }

  int state() const { return state_; }
  bool fired() const { return state_ >= FIRED_EARLY; }
  float bottom() const { return bottom_; }
  float bottom_x() const { return bottom_x_; }
  float peak() const { return peak_; }
  float fire_x() const { return fire_x_; }
  int seq() const { return seq_; }

private:
  void fire(State s, float x) {
    state_ = s;
    fire_x_ = x;
    seq_++;
  }

  int state_ = IDLE;
  float bottom_ = 0.0f;
  float bottom_x_ = 0.0f;
  float peak_ = 0.0f;
  float fire_x_ = 0.0f;
  float prev_d_ = 0.0f;
  float slope_prev_ = -1.0e9f;
  bool has_prev_ = false;
  int seq_ = 0;

  // 直近の (x, d) 履歴。谷底更新時に「手前 stale_dist 以内の最大値」を引く。
  // 64 tick ≈ 1800mm/s で 115mm、500mm/s で 32mm(窓 30mm を満たす下限)。
  static constexpr int kHist = 64;
  float hx_[kHist] = {};
  float hd_[kHist] = {};
  int hn_ = 0;
  int hi_ = 0;
  void push(float x, float d) {
    hx_[hi_] = x;
    hd_[hi_] = d;
    hi_ = (hi_ + 1) % kHist;
    if (hn_ < kHist) hn_++;
  }
  float window_max(float x, float span) const {
    float m = -1.0e9f;
    for (int i = 0; i < hn_; i++) {
      if ((x - hx_[i]) <= span && hd_[i] > m) m = hd_[i];
    }
    return m;
  }
};
