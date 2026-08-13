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

  // ---- デューティ中間値 ----
  float duty_c                    = 0.0f;
  float duty_roll                 = 0.0f;
  float duty_roll_ang             = 0.0f;
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
  void  calc_translational_ctrl();
  void  calc_angle_velocity_ctrl();
  void  calc_front_ctrl_duty();
  void  summation_duty();
  void  apply_duty_limitter();
  void  clear_ctrl_val();
  void  check_fail_safe();
  float calc_sensor_pid();
  float calc_sensor_pid_dia();
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
