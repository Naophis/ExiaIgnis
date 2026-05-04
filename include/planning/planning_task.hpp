#pragma once

#include "gen_code_mpc/mpc_tgt_calc.h"
#include "gen_code_simple_pid/simple_pid_controller.h"
#include "pico/types.h"
#include "planning/astraea_types.hpp"
#include "structs.hpp"
#include "utils/kalman_filter.hpp"
#include "utils/kalman_filter_matrix.hpp"
#include <memory>
#include <stdint.h>

class SensingTask;

// 1kHz ハードウェアタイマー IRQ (TIMER1 alarm 0, Core0) で動作する
// planning / control タスク。
// モーター・吸引 PWM の初期化も本クラスが担う。
// Core0 の main ループから send_command() で目標値を投入し、
// IRQ 内で軌道生成・PID 制御・モーター出力を行う。
class PlanningTask {
public:
  enum class MotionMode : uint8_t {
    IDLE = 0,     // モーター停止
    STRAIGHT = 1, // 直進 (台形速度プロファイル)
    PIVOT = 2,    // 超信地旋回 (台形角速度プロファイル)
    STOP = 3,     // 現在速度から減速停止
  };

  struct Command {
    MotionMode mode = MotionMode::IDLE;
    float v_max = 0.0f;        // 目標速度   [mm/s]
    float v_end = 0.0f;        // 終端速度   [mm/s]
    float accl = 0.0f;         // 加速度     [mm/s^2]
    float decel = 0.0f;        // 減速度     [mm/s^2]
    float dist = 0.0f;         // 目標距離   [mm]
    float w_max = 0.0f;        // 目標角速度 [rad/s]
    float alpha = 0.0f;        // 角加速度   [rad/s^2]
    float ang = 0.0f;          // 目標角度   [rad] (PIVOT 時)
    float duty_suction = 0.0f; // 吸引モーター duty [%]
    uint32_t timestamp = 0;

    Command() = default;
    // volatile Command からのコピー (クロスコア受け取り用)
    Command(const volatile Command &o)
        : mode(o.mode), v_max(o.v_max), v_end(o.v_end), accl(o.accl),
          decel(o.decel), dist(o.dist), w_max(o.w_max), alpha(o.alpha),
          ang(o.ang), duty_suction(o.duty_suction), timestamp(o.timestamp) {}
  };

  // IRQ が毎 tick 更新する状態。Core0 main ループから読み取り可能。
  struct State {
    float img_v = 0.0f;        // 指令速度     [mm/s]
    float img_w = 0.0f;        // 指令角速度   [rad/s]
    float img_dist = 0.0f;     // 累積走行距離 [mm]
    float img_ang = 0.0f;      // 累積旋回角度 [rad]
    float v_est = 0.0f;        // 推定速度     [mm/s]
    float w_est = 0.0f;        // 推定角速度   [rad/s]
    float duty_l = 0.0f;       // 左モーター duty [%]
    float duty_r = 0.0f;       // 右モーター duty [%]
    float duty_suction = 0.0f; // 吸引モーター duty [%]
    MotionMode mode = MotionMode::IDLE;
    uint32_t tick = 0;

    State() = default;
    State(const volatile State &o)
        : img_v(o.img_v), img_w(o.img_w), img_dist(o.img_dist),
          img_ang(o.img_ang), v_est(o.v_est), w_est(o.w_est), duty_l(o.duty_l),
          duty_r(o.duty_r), duty_suction(o.duty_suction), mode(o.mode),
          tick(o.tick) {}
  };

  static std::shared_ptr<PlanningTask> create();

  // Core0 の main() から呼ぶ。モーター/吸引 PWM の GPIO・スライス設定のみ。
  void init(std::shared_ptr<SensingTask> sensing);

  // Core1 エントリから呼ぶ。TIMER1 IRQ を Core1
  // に登録して計測ループを開始する。
  void start_irq();

  // Core0 の MainTask から呼ぶ (IRQ は Core1 側)。__dmb() で cross-core 安全。
  // void send_command(const Command &cmd);

  // Astraea 互換: motion_tgt_val_t ポインタを渡して次の IRQ tick で
  // cp_request() を実行。 xTaskNotify(*th, (uint32_t)tgt_val.get(), ...)
  // に相当。
  void send_command(std::shared_ptr<motion_tgt_val_t> tgt);

  volatile State state{};

  // ---- Astraea MotionPlanning 互換インターフェース ----
  float last_tgt_angle = 0.0f;

  void motor_enable() {} // TODO: モーター有効化
  void motor_disable() { // IDLE コマンドでモーター停止
    Command cmd;
    cmd.mode = MotionMode::IDLE;
    // send_command(cmd);
  }
  void suction_enable(float duty, float /*duty_low*/) {
    Command cmd;
    cmd.mode = MotionMode::IDLE;
    cmd.duty_suction = duty;
    // send_command(cmd);
  }
  void suction_disable() {
    Command cmd;
    cmd.mode = MotionMode::IDLE;
    cmd.duty_suction = 0.0f;
    // send_command(cmd);
  }
  void reset_kf_state(bool /*full*/) {} // TODO: カルマンフィルタリセット
  void reset_pos(float /*x*/, float /*y*/, float /*ang*/) {
  } // TODO: 位置リセット

  void
  set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_sensing_result);

  void set_input_param_entity(std::shared_ptr<input_param_t> &_param);

  KalmanFilter kf_w;
  KalmanFilter kf_w2;
  KalmanFilter kf_v;
  KalmanFilter kf_v_r;
  KalmanFilter kf_v_l;
  KalmanFilter kf_dist;
  KalmanFilter kf_ang;
  KalmanFilter kf_ang2;
  KalmanFilter kf_batt;
  KalmanFilterMatrix pos;
  kinematics_t odm = {0};
  kinematics_t kim = {0};

private:
  PlanningTask() = default;

  bool motor_en = false;
  bool suction_en = false;
  bool search_mode = false;
  bool mode_select = false;
  static void timer_irq_handler();

  void tick(uint32_t dt_us);
  void update_trajectory(float dt);
  void update_control(float dt);
  void apply_motor();
  void update_ego_motion();
  void calc_sensor_dist_all();
  float calc_sensor(float data, float a, float b);
  void calc_sensor_dist_diff();
  float interp1d(vector<float> &vx, vector<float> &vy, float x,
                 bool extrapolate);
  int interp1d(vector<int> &vx, vector<int> &vy, float x, bool extrapolate);
  void generate_trajectory();
  void calc_kanamaya_ctrl();
  void cp_tgt_val();
  void calc_tgt_duty();
  float calc_sensor_pid();
  float calc_sensor_pid_dia();
  float check_sen_error(SensingControlType &type);
  float check_sen_error_dia(SensingControlType &type);
  void check_fail_safe();
  void calc_pid_val();
  void calc_pid_val_ang();
  void calc_pid_val_ang_vel();
  void calc_pid_val_front_ctrl();
  void reset_pid_val();
  void calc_angle_i_bias();
  void calc_front_ctrl_duty();
  void calc_translational_ctrl();
  void calc_angle_velocity_ctrl();
  void summation_duty();
  void apply_duty_limitter();
  void clear_ctrl_val();
  void limitter(float &kp, float &ki, float &kb, float &kd,
                pid_param_t &limitter);
  void set_next_duty(float duty_l, float duty_r, float duty_suction);
  std::shared_ptr<input_param_t> get_param() {
    return param; //
  }
  std::shared_ptr<motion_tgt_val_t> get_tgt_entity() { return tgt_val; }
  std::shared_ptr<sensing_result_entity_t> get_sensing_entity();
  void check_left_sensor_error(float &error, int &check, bool range_check_left,
                               bool dist_check_left, bool check_diff_left,
                               bool expand_left, bool range_check_left_expand);
  void check_right_sensor_error(float &error, int &check,
                                bool range_check_right, bool dist_check_right,
                                bool check_diff_right, bool expand_right,
                                bool range_check_right_expand);
  void set_ctrl_val(pid_error2_t &val, float error_p, float error_i,
                    float error_i2, float error_d, float val_p, float val_i,
                    float val_i2, float val_d, float zz, float z);
  static std::shared_ptr<PlanningTask> s_instance;

  std::shared_ptr<SensingTask> sensing_;
  std::shared_ptr<sensing_result_entity_t> sensing_result;
  std::shared_ptr<input_param_t> param;
  uint32_t interval_us_ = 1000;
  uint32_t next_alarm_ = 0;
  uint64_t prev_ts_ = 0;
  std::vector<float> log_table;

  // Core1 main_task → Core0 IRQ コマンドバッファ (Command 形式)
  // __dmb() + volatile でクロスコア可視性を保証
  volatile Command pending_cmd_{};
  volatile bool cmd_pending_ = false;

  // Astraea 互換: motion_tgt_val_t ベースのコマンドバッファ
  std::shared_ptr<motion_tgt_val_t> pending_tgt_;
  std::shared_ptr<motion_tgt_val_t> tgt_val;
  std::shared_ptr<pid_error_entity_t> ee;
  volatile bool tgt_cmd_pending_ = false;
  int32_t last_nmr_timestamp_ = -1;

  Command active_cmd_{};

  // 軌道生成 (台形速度プロファイル)
  float img_v_ = 0.0f;
  float img_w_ = 0.0f;
  float img_dist_ = 0.0f;
  float img_ang_ = 0.0f;

  // 推定値 (エンコーダ差分から)
  float v_est_ = 0.0f;
  float w_est_ = 0.0f;
  uint16_t enc_r_prev_ = 0;
  uint16_t enc_l_prev_ = 0;
  bool first_tick_ = true;

  // PID 積分項
  float vel_err_i_ = 0.0f;
  float gyro_err_i_ = 0.0f;

  // モーター・吸引 PWM
  uint slice_L_ = 0;
  uint slice_R_ = 0;
  uint slice_S_ = 0; // 吸引モーター
  uint32_t motor_wrap_ = 2999;

  //

  float suction_gain = 200;
  bool gyro_pid_windup_histerisis = false;
  float gyro_pid_histerisis_i = 0.0;

  Simple_PID_Controller vel_pid;
  Simple_PID_Controller gyro_pid;
  Simple_PID_Controller ang_pid;

  sensor_ctrl_keep_dist_t right_keep;
  sensor_ctrl_keep_dist_t left_keep;
  float mpc_d_estimated = 0;
  float mpc_w_prev = 0;
  float mpc_u_prev = 0;
  duty_t tgt_duty;
  int fail_check_ang = 0;
  int keep_wall_off_cnt = 0;
  int buzzer_time_cnt = 0;
  int buzzer_timestamp = 0;
  int motion_req_timestamp = 0;
  int pid_req_timestamp = 0;
  int motor_req_timestamp = 0;
  int suction_req_timestamp = 0;
  bool enable_expand_right = false;
  bool enable_expand_left = false;

  unsigned char w_reset = 0;
  float ideal_v_r, ideal_v_l;
  float v_cmd = 0;
  float w_cmd = 0;

  float duty_c = 0;
  float duty_c2 = 0;
  float duty_roll = 0;
  float duty_roll_ang = 0;
  float duty_front_ctrl_roll = 0;
  float duty_front_ctrl_trans = 0;
  float duty_front_ctrl_roll_keep = 0;
  float duty_sen = 0;
  float sen_ang = 0;

  int32_t mpc_mode;
  int32_t mpc_step;
  t_dynamics dynamics;
  t_ego mpc_next_ego;
  t_ego mpc_next_ego2;
  t_ego mpc_next_ego_prev;
  bool first_req = false;
  float last_accl = 0.0f;
  mpc_tgt_calcModelClass mpc_tgt_calc;
  std::vector<t_ego> trajectory_points;
  float duty_l_ = 0.0f;
  float duty_r_ = 0.0f;
  float duty_suction_ = 0.0f;
  float diff_old = 0;
  float diff = 0;
  float gain_cnt = 0;
  std::vector<float> axel_degenerate_x;
  std::vector<float> axel_degenerate_y;
  std::vector<float> axel_degenerate_dia_x;
  std::vector<float> axel_degenerate_dia_y;
  std::vector<float> sensor_deg_limitter_v;
  std::vector<float> sensor_deg_limitter_str;
  std::vector<float> sensor_deg_limitter_dia;
  std::vector<float> sensor_deg_limitter_piller;
  std::vector<int> trj_idx_v;
  std::vector<int> trj_idx_val;
};
