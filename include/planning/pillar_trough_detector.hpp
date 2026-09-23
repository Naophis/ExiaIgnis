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
//   (2) 離脱(主): 走行距離で正規化した 2 階微分(曲率)が curv_th を超える状態が
//       curv_n tick 以上続き、その連続区間の合計が curv_sum 以上で、読みが谷底より上。
//       「連続」と「合計」の両方が要る。連続だけだと谷底の丸い走行で合計は足りるのに
//       1 tick 遅れ(20260923_173424: 谷底の曲率が 0.206 しかなく、しきい値 0.3 では
//       谷底+2 になった)、合計だけだと孤立した 1 発で誤発火する(20260923_013712
//       idx376: 単発 1.151 の直後に -0.262)。
//       2026-09-23 実機 8 本で確認: 首振れや姿勢ドリフトは読みをほぼ直線的に動かす
//       ので、2 階微分では符号が交互に振れるだけになる(20260923_013712 idx376-384、
//       ヨー -1.2→-3.3°: 2階差分 -0.99,+0.37,+0.41,+0.48,-1.61,+2.21,-1.26,+0.56)。
//       柱の切れ目は傾き自体が変わるので正が続く(20260923_160516 idx85-89:
//       -0.07,+1.21,+0.94,+2.13,+1.04)。1 階微分だと首振れの +2.21 と柱の +2.69 が
//       紛らわしく、slope_min や距離キャップで押さえる必要があった。
//       これで 8 本中 7 本が谷底+1 で発火し、平均 lag が 4.53mm → 1.71mm になった。
//       両点灯 ch(LED1 より約 2tick 遅れて谷底を打つ)に依存しないのも効いている。
//   (2') 離脱(保険): 従来の 1 階微分ルール。曲率が確定しない形のときに拾う。
//       上昇率が slope_min 以上、前 tick も上昇、谷底から rise_min 以上、
//       両点灯 ch が明確な下降中でない。曲率ルールのほうが常に先に出るので、
//       これが効くのは曲率が閾値に届かなかった場合だけ。
//   (3) 離脱(確認): far_th 以上へ抜けた(前 tick で既に rise_min 以上上がり、
//       両点灯 ch も上昇)。
//   (4) 谷底から max_lag より進んでも離脱しなければ柱ではない(壁の長い平坦や、
//       谷が古い場合を除外)。谷底が stale_dist より古くなれば再アーム。
// 発火位置ではなく谷底位置(bottom_x)をアンカーにするので、発火までの遅れは
// 補正定数(wall_off_pillar_str_l/r)に混ざらない。
//
// 首振れ(姿勢制御のヨー)耐性: 45° 読みのヨー感度は幾何で ≈ D/57 mm/deg
// (65mm の柱で約 1.1mm/deg)。直線中に観測された最大ヨーは 1.5°/tick なので
// 1tick の変化は ≤1.7mm(壁読みの |Δd| は p99 で 1.8mm/tick)。首振れが作るのは
// ほぼ直線のランプなので (2) の「正の曲率が続く」は成立しない(実測の首振れ区間で
// 正の曲率は単発どまり)。(2') は 1800mm/s で ≥2.7mm/tick の単発上昇+前 tick 上昇+
// 累積 4mm 以上、(3) は +30mm 以上を要求する。
//
// 実測(2026-09-23、実機 9 本 v=1500): 谷底の曲率は 0.206〜0.775 と走行ごとに 4 倍
// 近く変わる(谷底の丸さの差)。一方、谷底をまたぐ連続区間の合計は 0.77〜1.58 で
// ばらつきが小さく、首振れのランプ(同 013712 idx376-384)の最大 0.34 とよく離れる。
// curv_sum=0.7 なら 9 本中 8 本が谷底+1 で発火する(残り 1 本は谷底が同値 2 点の
// 平坦で、読みが谷底より上にならないぶん 1 tick 遅れる)。
//
// 注意: 曲率そのものが首振れを分離しているのではない。実際に誤発火しないのは
// 谷の追跡ゲート(depth_min・bottom_min/max・max_lag・読みが谷底より上)が候補を
// 谷の直後の数 tick に絞っているからで、曲率はその中のどこで出すかを決めている。
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
  float curv_th = 0.0f;      // [mm/mm^2] 離脱(主): 連続とみなす曲率の下限(既定は「正」)
  int   curv_n = 2;          // [tick] curv_th 超えが連続する最小 tick 数
  float curv_sum = 0.7f;     // [mm/mm] その連続区間の曲率の合計(= 傾きの総変化量)
  float slope_min = 1.5f;    // [mm/mm] 離脱(保険): 上昇率(走行 1mm あたり)、その tick の単発
  float both_diff_min = -1.0f; // [mm] 両点灯 ch の 1tick 差分がこれより大きい(明確な下降中は不可)
  float rise_min = 4.0f;     // [mm] 谷底からの上昇量(早期条件/確認条件の前 tick)
  float far_th = 100.0f;     // [mm] 確認条件: ここまで抜けたら離脱確定
  float max_lag = 14.0f;     // [mm] 谷底からこれ以内に離脱しなければ柱ではない
  float stale_dist = 30.0f;  // [mm] 谷底がこれより古ければ再アーム(発火済みも消す)。
                             //      ピークを探す窓(谷底の手前この距離以内)にも使う
  float min_travel = 0.5f;   // [mm] 1tick 走行の下限(低速でのノイズ増幅防止)
  bool  vertex_interp = true; // 谷底位置を放物線の頂点でサブ tick 補正するか
};

class PillarTroughDetector {
public:
  enum State : int {
    IDLE = 0,
    TRACKING = 1,
    FIRED_CURV = 2,    // 曲率ラン(主)
    FIRED_CONFIRM = 3, // far_th まで抜けた(最後の保険)
    FIRED_SLOPE = 4,   // 1 階微分ルール(保険)
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
    prev2_d_ = d;
    has_prev_ = true;
    has_prev2_ = false;
    slope_prev_ = -1.0e9f;
    curv_run_ = 0;
    curv_acc_ = 0.0f;
    // アーム直後の点は「降りてきた谷底」ではないので頂点補間しない。
    bottom_prev_ = -1.0f;
    bottom_h_ = 0.0f;
    vertex_done_ = true;
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
    // 走行距離で正規化した 2 階微分 [mm/mm^2]。等間隔を仮定して dx^2 で割る。
    const float curv = has_prev2_ ? (d - 2.0f * prev_d_ + prev2_d_) / (dx * dx) : 0.0f;
    if (has_prev2_ && curv > p.curv_th) {
      curv_run_++;
      curv_acc_ += curv;
    } else {
      curv_run_ = 0;
      curv_acc_ = 0.0f;
    }
    push(x, d);

    if (state_ == TRACKING) {
      if (d < bottom_) {
        bottom_prev_ = prev_d_; // 谷底の 1 つ前の読み(頂点補間用)
        bottom_ = d;
        bottom_x_ = x;
        bottom_h_ = dx;
        peak_ = window_max(x, p.stale_dist); // 谷底の手前 stale_dist 以内の最大値
        vertex_done_ = false;
      } else if (!vertex_done_ && p.vertex_interp) {
        // 谷底の次の点が来た。3 点(谷底の前・谷底・谷底の次)を通る放物線の頂点で
        // 谷底位置をサブ tick 補正する。1kHz・1.5mm/tick では谷底が 2 tick に
        // またがると最大 0.75mm ずれ、それがそのままアンカー誤差になっていた。
        //
        // 2026-09-23 の検証について(重要): 手持ちのログでは効果を測れない。
        // 機体は補間なしの谷底 tick にアンカーして停止位置を決めており、
        // その同じ点を基準に「停止−谷底」を測ると量子化が相殺されて消える
        // (実測 std 0.213mm は補間の有無ではなく駆動系の再現性を見ているだけ)。
        // 最初に出した「0.34mm → 0.08mm」は n=3 の偶然で、n=8 では逆に
        // 0.213 → 0.243mm と悪化した。どちらも測定の性質上、補間の良し悪しを
        // 示していない。
        // 正しい検証: tick の位相が変わるよう速度を変えた 2 条件(例 1200 と
        // 1800mm/s)で同じ柱を走り、停止位置の条件間差が縮むかを見る。
        // 根拠としては「凸関数を離散サンプルした最小点は必ずサンプル間にある」
        // ことと、同値が 2 点並ぶ平坦な谷底(20260923_064918 idx2207/2208)で
        // +0.5 tick が一意に正しいこと。平均のずれは 8 本で +0.01mm なので
        // pillar_str の付け替えは不要。
        vertex_done_ = true;
        if (bottom_prev_ > 0.0f) {
          const float den = bottom_prev_ - 2.0f * bottom_ + d;
          if (den > 0.0f) {
            float delta = 0.5f * (bottom_prev_ - d) / den;
            // 頂点は谷底 tick の前後半 tick 以内にしかあり得ない。
            if (delta < -0.5f) delta = -0.5f;
            if (delta > 0.5f) delta = 0.5f;
            bottom_x_ += delta * (delta < 0.0f ? bottom_h_ : dx);
          }
        }
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
      if (shape_ok) {
        if (curv_run_ >= p.curv_n && curv_acc_ >= p.curv_sum && rise > 0.0f) {
          fire(FIRED_CURV, x);
        } else if (rise >= p.rise_min && slope >= p.slope_min &&
                   slope_prev_ > 0.0f && d2_diff > p.both_diff_min) {
          fire(FIRED_SLOPE, x);
        } else if (d >= p.far_th && (prev_d_ - bottom_) >= p.rise_min &&
                   d2_diff > 0.0f) {
          fire(FIRED_CONFIRM, x);
        }
      }
    }

    prev2_d_ = prev_d_;
    has_prev2_ = true;
    prev_d_ = d;
    slope_prev_ = slope;
  }

  int state() const { return state_; }
  bool fired() const { return state_ >= FIRED_CURV; }
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
  float prev2_d_ = 0.0f;
  float slope_prev_ = -1.0e9f;
  bool has_prev_ = false;
  bool has_prev2_ = false;
  int curv_run_ = 0;      // curv_th 超えが続いている tick 数
  float curv_acc_ = 0.0f; // その連続区間の曲率の合計
  float bottom_prev_ = -1.0f; // 谷底の 1 つ前の読み(頂点補間用、-1=無効)
  float bottom_h_ = 0.0f;     // 谷底 tick の走行量
  bool vertex_done_ = true;   // 頂点補間を適用済みか
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
