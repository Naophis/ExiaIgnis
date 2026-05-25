#pragma once
#include "gen_code_simple_pid/simple_pid_controller.h"
#include "planning/astraea_types.hpp"
#include "planning/ego_estimator.hpp"
#include "planning/motor_actuator.hpp"
#include "planning/sensor_processor.hpp"
#include "planning/trajectory_generator.hpp"
#include "structs.hpp"
#include <memory>

// PID 制御・デューティ計算・モーター出力をまとめる。
// init() で他サブシステムのポインタを受け取り、
// calc() を 1kHz tick の都度呼ぶ。
class ControlLaw {
public:
  // ---- ライフサイクル ----
  void init(MotorActuator *motor, SensorProcessor *sensor,
            TrajectoryGenerator *trj, EgoEstimator *ego,
            std::shared_ptr<motion_tgt_val_t> tgt_val,
            std::shared_ptr<sensing_result_entity_t> sensing_result,
            std::shared_ptr<input_param_t> param);

  // ---- メインエントリ (1kHz tick) ----
  void calc(bool motor_en, bool suction_en, bool search_mode,
            unsigned char w_reset, float last_tgt_angle, float dt);

  // ---- コマンドリクエスト処理 ----
  void pl_req_activate(const planning_req_t &pl_req);

  // ---- 吸引セッター ----
  void set_suction_target(float duty, float duty_low) {
    tgt_duty.duty_suction     = duty;
    tgt_duty.duty_suction_low = duty_low;
  }
  void set_suction_gain(float gain) { suction_gain = gain; }

  // ---- 公開データ ----
  std::shared_ptr<pid_error_entity_t> ee;

private:
  // ---- サブシステム参照 ----
  MotorActuator       *motor_  = nullptr;
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
  float  suction_gain = 200.0f;
  float  gain_cnt     = 0.0f;
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
