#pragma once
#include "gen_code_simple_pid/simple_pid_controller.h"
#include "planning/astraea_types.hpp"
#include "planning/ego_estimator.hpp"
#include "planning/motor_actuator.hpp"
#include "planning/sensor_processor.hpp"
#include "planning/suction_esc_actuator.hpp"
#include "planning/trajectory_generator.hpp"
#include "structs.hpp"
#include <memory>

// PID 制御・デューティ計算・モーター出力をまとめる。
// init() で他サブシステムのポインタを受け取り、
// calc() を 1kHz tick の都度呼ぶ。
class ControlLaw {
public:
  // 走行開始時の keep_dist ヒステリシス一回限り無効化(structs.hpp
  // input_param_t::keep_dist_th_start_skip参照)。PlanningTask::cp_request()
  // から走行開始のSTRAIGHT指令受理時に呼ぶ。
  void skip_keep_dist_once();
  // ---- ライフサイクル ----
  void init(MotorActuator *motor, SuctionEscActuator *esc,
            SensorProcessor *sensor, TrajectoryGenerator *trj,
            EgoEstimator *ego, std::shared_ptr<motion_tgt_val_t> tgt_val,
            std::shared_ptr<sensing_result_entity_t> sensing_result,
            std::shared_ptr<input_param_t> param);

  // ---- メインエントリ (1kHz tick) ----
  void calc(bool motor_en, bool suction_en, bool search_mode,
            float last_tgt_angle, float dt);

  // ---- コマンドリクエスト処理 ----
  void pl_req_activate(const planning_req_t &pl_req);

  // ---- 吸引セッター ----
  void set_suction_target(float duty, float duty_low) {
    tgt_duty.duty_suction     = duty;
    tgt_duty.duty_suction_low = duty_low;
  }

  // 吸引duty指令が目標値へ向けてまだランプ中かどうか。旧BldcActuator::
  // is_ramping()の代替(PlanningTask::is_suction_ramping()から呼ばれる)。
  bool is_suction_ramping() const {
    return suction_en_ && (suction_pulse_us_ != suction_target_us_);
  }

  // 目標パルス幅へのランプ速度(us/sec)。system.yamlの
  // suction_esc_ramp_us_per_secから起動時に一度だけ設定される
  // (main_task.cpp load_param_after()参照)。
  void set_suction_ramp_rate(float us_per_sec) {
    suction_ramp_us_per_sec_ = us_per_sec;
  }

  // バッテリー電圧(batt_kf、昇順)→吸引パルス幅への上乗せus値の区分線形LUT。
  // system.yamlのsuction_batt_boost_v_table/suction_batt_boost_us_tableから
  // 起動時に一度だけ設定される(main_task.cpp load_param_after()参照)。
  void set_suction_batt_boost_table(std::vector<float> v_table,
                                    std::vector<float> us_table) {
    suction_batt_boost_v_table_  = std::move(v_table);
    suction_batt_boost_us_table_ = std::move(us_table);
  }

  // 現在のパルス幅(us、昇順)→その位置でのランプ速度(us/sec)の区分線形LUT。
  // system.yamlのsuction_ramp_rate_us_x/suction_ramp_rate_us_yから起動時に
  // 一度だけ設定される(main_task.cpp load_param_after()参照)。空(未指定)
  // ならsuction_ramp_us_per_sec_の固定レートのまま(set_next_duty()参照)。
  void set_suction_ramp_rate_table(std::vector<float> x_table,
                                   std::vector<float> y_table) {
    suction_ramp_rate_us_x_ = std::move(x_table);
    suction_ramp_rate_us_y_ = std::move(y_table);
  }

  // ---- 公開データ ----
  std::shared_ptr<pid_error_entity_t> ee;

private:
  // ---- サブシステム参照 ----
  MotorActuator       *motor_  = nullptr;
  SuctionEscActuator  *esc_    = nullptr;
  SensorProcessor     *sensor_ = nullptr;
  TrajectoryGenerator *trj_    = nullptr;
  EgoEstimator        *ego_    = nullptr;

  // ---- tick キャッシュ入力 ----
  std::shared_ptr<motion_tgt_val_t>        tgt_val_;
  std::shared_ptr<sensing_result_entity_t> sensing_result_;
  std::shared_ptr<input_param_t>           param_;
  bool          motor_en_       = false;
  bool          suction_en_     = false;
  bool          search_mode_    = false;
  unsigned char w_reset_        = 0;
  float         last_tgt_angle_ = 0.0f;
  float         dt_             = 0.0f;

  // ---- PID コントローラ ----
  Simple_PID_Controller vel_pid;
  Simple_PID_Controller gyro_pid;

  // ---- 吸引制御 ----
  // 目標パルス幅(us)へ suction_ramp_us_per_sec_ の速度で線形にランプする
  // (set_next_duty()参照)。AM32移行前は BldcActuator::get_ramp_rate() の
  // battery_v→ramp_gain LUT値(elec_hz空間)を共有していたが、ESC側は
  // パルス幅(us)指令のみで済むため、system.yaml一発値の固定レートランプに
  // 簡略化している。
  float  suction_ramp_us_per_sec_ = 2000.0f;
  float  suction_pulse_us_        = 1000.0f; // 現在のランプ済みパルス幅(us)
  float  suction_target_us_       = 1000.0f; // 直近tickの目標パルス幅(us)
  std::vector<float> suction_batt_boost_v_table_;
  std::vector<float> suction_batt_boost_us_table_;
  std::vector<float> suction_ramp_rate_us_x_;
  std::vector<float> suction_ramp_rate_us_y_;
  duty_t tgt_duty{};

  // ---- センサー制御状態 ----
  sensor_ctrl_keep_dist_t right_keep{};
  sensor_ctrl_keep_dist_t left_keep{};
  bool  enable_expand_right = false;
  bool  enable_expand_left  = false;
  float diff_old = 0.0f;
  float diff     = 0.0f;

  // ---- MPC オブザーバー ----
  float mpc_d_estimated = 0.0f;
  float mpc_w_prev      = 0.0f;
  float mpc_u_prev      = 0.0f;

  // ---- フェイルセーフ・積分状態 ----
  int   fail_check_ang             = 0;
  int   keep_wall_off_cnt          = 0;
  float last_accl                  = 0.0f;
  bool  gyro_pid_windup_histerisis = false;
  float gyro_pid_histerisis_i      = 0.0f;
  // アンチワインド・ヒステリシスのON/OFF判定デバウンス用カウンタ(2026-08-30)。
  // control_law.cpp calc_angle_velocity_ctrl()参照。判定条件がdeadband境界
  // 付近でノイズにより毎tick反転すると、脱出時の再点火(ee->ang.error_p/dt_、
  // 実質1000倍)が毎tick発火してw_error_i(ログg_pid_i2)が巨大値と小さい値を
  // 交互に繰り返すチャタリングを起こす(20260830_215101.csv解析、旋回直後の
  // 角度収束が遅い症状の一因と判明)。判定が数tick連続で一致するまで実際の
  // 状態遷移(=再点火含む)を保留することでチャタリングを防ぐ。
  int   gyro_pid_windup_debounce_cnt_ = 0;
  // SLALOM/SLA_BACK_STR限定のang.i_bias専用積分(turn_angle_fb.gain_i)。
  // 既存のw_error_i(アンチワインドヒステリシス付き)を再利用すると実機で
  // 発散したため(2026-08-23、20260823_050137.csv)、これとは独立の単純な
  // クランプ付き積分として新設する(control_law.cpp calc_angle_velocity_ctrl()
  // 参照、旋回開始でゼロクリア)。
  float turn_angle_fb_integral_    = 0.0f;
  float turn_angle_fb_i_bias_prev_ = 0.0f; // D項用、旋回開始でゼロクリア
  // turn_end_brake用duty_rollスルーレート制限の前回値(2026-08-23夜)。
  // control_law.cpp calc_angle_velocity_ctrl()参照。
  float turn_end_brake_duty_prev_  = 0.0f;
  // hold_active(MotionPlanning::hold())専用の角度積分I項(2026-09-05)。
  // hold_active開始でゼロクリア、control_law.cpp calc_angle_velocity_ctrl()参照。
  float hold_ang_integral_ = 0.0f;
  // reset-on-move用: 積分開始/前回リセット時点のkim.theta[rad]と、
  // hold_activeの立ち上がり検出用の前tick値(structs.hpp
  // hold_ang_i_reset_ang_thのコメント参照)。
  float hold_i_ang_ref_    = 0.0f;
  float hold_kim_lp_       = 0.0f; // 判定用にLPFしたkim.theta[rad]
  bool  hold_active_prev_  = false;
  // 走り出し姿勢リセット(start_align、2026-09-06、structs.hpp start_align_t
  // 参照)。motor_enの立ち上がりで武装し、壁追従が収束したらego_in.ang/
  // kim.theta等を一度だけゼロへ再アンカーする(update_start_align()参照)。
  bool  motor_en_prev_       = false;
  bool  start_align_pending_ = false;
  int   start_align_cnt_     = 0;
  float start_align_dist_    = 0.0f; // 判定窓内の走行距離[mm]
  float start_align_fire_dist_ = -1.0e9f; // 直近の発火時のglobal_pos.dist[mm]
  // 横位置の壁基準補正用アンカー(update_start_align()参照)。武装後に最初に
  // 壁を見たtick、および各発火時点の「壁から見た横位置」「pos_y」「kim.x/y」。
  bool  start_align_anchor_valid_ = false;
  // アンカーの通路(motion_typeが変わるたびに+1するセグメント番号)と、通路の
  // 横方向単位ベクトル(世界座標)、アンカー時点の横位置(世界座標での射影)。
  // 横位置の合わせ込みは同じ通路内でのみ行い、補正は通路の横方向へ入れる
  // (2026-09-09: 旋回後も世界yへ入れていてpos_yが150〜300mm跳んだ、
  // 20260909_004035.csv idx790/1538/4622/5294)。
  int   start_align_anchor_seg_  = -1;
  float start_align_anchor_nx_   = 0.0f;
  float start_align_anchor_ny_   = 1.0f;
  float start_align_anchor_latw_ = 0.0f;
  int   motion_seg_id_ = 0;
  MotionType motion_type_prev_ = MotionType::NONE;
  bool  start_align_anchor_two_   = false; // アンカーが両壁基準か
  float start_align_anchor_lat_   = 0.0f; // 壁基準の横位置[mm、左が正]
  float start_align_anchor_pos_y_ = 0.0f;
  float start_align_anchor_kim_x_ = 0.0f;
  float start_align_anchor_kim_y_ = 0.0f;
  float start_align_ang0_    = 0.0f; // 判定窓開始時のego_in.ang
  // 壁基準ヘディング回帰用の累積和(structs.hpp start_align_t::slope_th参照)。
  // x=窓内走行距離[mm]、y=壁横位置−lat_k·Δang[mm]。窓開始でクリア。
  float start_align_fit_sx_  = 0.0f;
  float start_align_fit_sy_  = 0.0f;
  float start_align_fit_sxx_ = 0.0f;
  float start_align_fit_sxy_ = 0.0f;
  float start_align_fit_syy_ = 0.0f;
  float start_align_fit_sa_  = 0.0f; // Σ(ang−ang0)[rad]、ジャイロ融合用
  bool  start_align_fit_two_ = false; // 窓開始時の両壁/片壁モード
  // 走行中の壁基準平行推定(wall_fit、structs.hpp wall_fit_t参照)。
  // 状態 y[mm](壁基準横位置)、β[rad](ジャイロ座標系と格子のずれ)、共分散P。
  float wall_fit_y_    = 0.0f;
  float wall_fit_beta_ = 0.0f;
  float wall_fit_P00_  = 1.0e4f;
  float wall_fit_P01_  = 0.0f;
  float wall_fit_P11_  = 0.0f;
  int   wall_fit_mode_prev_ = 0; // 0:なし 1:左壁 2:右壁 3:両壁
  MotionType wall_fit_mt_prev_ = MotionType::NONE;
  float start_align_err0_    = 0.0f; // 判定窓開始時のsen.error_p
  // ego_in.ang(=生ジャイロ積分ヘディング、sensing_task.cpp calc_vel()
  // 参照)は壁を検出していない間は無補正でドリフトし続ける。壁を新規に
  // 検出した瞬間(calc_sensor_pid()参照)にゼロクリアして、信頼できる基準
  // (壁と正対=0)へスナップし直すための直前tickの壁検出状態。
  bool  wall_found_prev_           = false;
  // 前tickでセンサー制御が成立していたか(type != None)。calc_sensor_pid()は
  // 今tickのerror_pを求める前に前tickのerror_pを積分するため、その誤差が
  // 壁を見えている状態のものだったかを判定するのに使う。
  bool  sen_ctrl_active_prev_      = false;
  // 2026-08-30: mpc_tgt_calc.cpp(Simulink自動生成)のsign()実装が、入力が
  // ちょうど0.0fを跨ぐ瞬間だけ0を返す仕様のため、ff_front_torque/
  // ff_friction_torque_r/lが走行中(速度が明確に非ゼロ)でも数tickおきに
  // 瞬間的に0へ落ちるチャタリングを確認(20260904_171508.csv)。モデル側
  // (.slx)を直さずに、走行中の異常な0だけ直前値で保持するガードを
  // calc_translational_ctrl()に追加する。
  float ff_front_torque_prev_      = 0.0f;
  float ff_friction_torque_r_prev_ = 0.0f;
  float ff_friction_torque_l_prev_ = 0.0f;
  // 2026-09-06: ff_roll_torqueはalpha2(角加速度)に比例するだけでsign()を
  // 経由しないため単独ではチャタリングしないはずだが、上流のalpha2生成側
  // (Merge/Switchブロック)でも同種の瞬間ゼロ落ちが起こり得るため、
  // front/frictionと同じ保持ガードを揃える。
  float ff_roll_torque_prev_       = 0.0f;
  // 2026-09-04: turn_duty_floor(下限クランプ)だけでは「落ちきってから
  // 頭打ち」にしかならず、落ちる速度自体が速いと間に合わずスリップする
  // (apply_duty_limitter()参照)。SLALOM/SLA_BACK_STR中のduty変化速度自体を
  // スルーレート制限するための前回値。
  float turn_duty_r_prev_ = 0.0f;
  float turn_duty_l_prev_ = 0.0f;

  // ---- デューティ中間値 ----
  float duty_c                    = 0.0f;
  float duty_roll                 = 0.0f;
  float duty_roll_ang             = 0.0f;
  // STRAIGHT Kanayamaカスケード用のΔw[rad/s]。duty_roll_angと同じく
  // calc_angle_velocity_ctrl()のoffsetに一時的に加算されるだけで、
  // tgt_val_->ego_in.w(計画自身が毎tick自己伝播する状態)は書き換えない。
  float sen_kanayama_dw            = 0.0f;
  float duty_front_ctrl_roll      = 0.0f;
  float duty_front_ctrl_trans     = 0.0f;
  float duty_front_ctrl_roll_keep = 0.0f;
  float duty_sen                  = 0.0f;
  float sen_ang                   = 0.0f;

  // ---- 内部計算メソッド ----
  void  calc_tgt_duty();
  void  calc_pid_val();
  void  calc_pid_val_ang();
  void  calc_pid_val_ang_vel();
  void  calc_pid_val_front_ctrl();
  void  reset_pid_val();
  void  calc_angle_i_bias();
  bool  angle_i_bias_active(MotionType mt) const;
  void  calc_translational_ctrl();
  void  calc_angle_velocity_ctrl();
  void  calc_front_ctrl_duty();
  void  summation_duty();
  void  apply_duty_limitter();
  void  clear_ctrl_val();
  void  check_fail_safe();
  float calc_sensor_pid();
  float calc_sensor_pid_dia();
  void  update_start_align(SensingControlType type);
  void  update_wall_fit(SensingControlType type);
  void  reset_wall_fit();
  float check_sen_error(SensingControlType &type);
  float check_sen_error_dia(SensingControlType &type);
  void  check_left_sensor_error(float &error, int &check,
                                bool range_check_left, bool dist_check_left,
                                bool check_diff_left, bool expand_left,
                                bool range_check_left_expand);
  void  check_right_sensor_error(float &error, int &check,
                                 bool range_check_right, bool dist_check_right,
                                 bool check_diff_right, bool expand_right,
                                 bool range_check_right_expand);
  void  limitter(float &kp, float &ki, float &kb, float &kd, pid_param_t &lim);
  void  set_next_duty(float duty_l, float duty_r, float duty_suction);
  void  set_ctrl_val(pid_error2_t &val, float error_p, float error_i,
                     float error_i2, float error_d, float val_p, float val_i,
                     float val_i2, float val_d, float zz, float z);
};
