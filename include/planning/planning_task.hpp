#pragma once

#include "pico/types.h"
#include "planning/astraea_types.hpp"
#include "planning/control_law.hpp"
#include "planning/ego_estimator.hpp"
#include "planning/motor_actuator.hpp"
#include "planning/sensor_processor.hpp"
#include "planning/trajectory_generator.hpp"
#include "structs.hpp"
#include "utils/kalman_filter.hpp"
#include "utils/kalman_filter_matrix.hpp"
#include <memory>
#include <stdint.h>

class SensingTask;

// 1kHz ハードウェアタイマー IRQ (TIMER1 alarm 0, Core1) で動作する
// planning / control タスク。
// Core0 の main ループから send_command() で目標値を投入し、
// IRQ 内で軌道生成・PID 制御・モーター出力を行う。
class PlanningTask {
public:
  // ---- 型定義 ----
  enum class MotionMode : uint8_t {
    IDLE     = 0,
    STRAIGHT = 1,
    PIVOT    = 2,
    STOP     = 3,
  };

  struct Command {
    MotionMode mode         = MotionMode::IDLE;
    float      v_max        = 0.0f;
    float      v_end        = 0.0f;
    float      accl         = 0.0f;
    float      decel        = 0.0f;
    float      dist         = 0.0f;
    float      w_max        = 0.0f;
    float      alpha        = 0.0f;
    float      ang          = 0.0f;
    float      duty_suction = 0.0f;
    uint32_t   timestamp    = 0;

    Command() = default;
    Command(const volatile Command &o)
        : mode(o.mode), v_max(o.v_max), v_end(o.v_end), accl(o.accl),
          decel(o.decel), dist(o.dist), w_max(o.w_max), alpha(o.alpha),
          ang(o.ang), duty_suction(o.duty_suction), timestamp(o.timestamp) {}
  };

  // IRQ が毎 tick 更新する状態。Core0 から読み取り可能。
  struct State {
    float      img_v        = 0.0f;
    float      img_w        = 0.0f;
    float      img_dist     = 0.0f;
    float      img_ang      = 0.0f;
    float      v_est        = 0.0f;
    float      w_est        = 0.0f;
    float      duty_l       = 0.0f;
    float      duty_r       = 0.0f;
    float      duty_suction = 0.0f;
    MotionMode mode         = MotionMode::IDLE;
    uint32_t   tick         = 0;

    State() = default;
    State(const volatile State &o)
        : img_v(o.img_v), img_w(o.img_w), img_dist(o.img_dist),
          img_ang(o.img_ang), v_est(o.v_est), w_est(o.w_est), duty_l(o.duty_l),
          duty_r(o.duty_r), duty_suction(o.duty_suction), mode(o.mode),
          tick(o.tick) {}
  };

  // ---- ライフサイクル ----
  static std::shared_ptr<PlanningTask> create();
  void init(std::shared_ptr<SensingTask> sensing);
  void start_irq();

  // ---- Core0 → Core1 コマンド ----
  void send_command(std::shared_ptr<motion_tgt_val_t> tgt);

  // ---- 設定セッター ----
  void set_search_mode(bool v) { search_mode = v; }
  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_sensing_result);
  void set_input_param_entity(std::shared_ptr<input_param_t> &_param);
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> _tgt_val) { tgt_val = _tgt_val; }

  // ---- モーター・吸引制御 ----
  void motor_enable();
  void motor_disable();
  void suction_enable(float duty, float duty_low);
  void suction_disable() { suction_en = false; }

  // ---- 推定リセット ----
  void reset_kf_state(bool full) {
    if (sensing_result && param)
      ego.reset_kf_state(full, sensing_result, param);
  }
  void reset_pos(float /*x*/, float /*y*/, float /*ang*/) {} // TODO: 位置リセット

  // ---- センサー補正ユーティリティ ----
  float adjust_b_to_target90(float data, float a);
  float adjust_b_to_target45(float data, float a);

  // ---- 公開データ ----
  volatile State                    state{};
  float                             last_tgt_angle = 0.0f;
  std::shared_ptr<motion_tgt_val_t> tgt_val;
  EgoEstimator                      ego;
  MotorActuator                     motor_;
  SensorProcessor                   sensor_;
  TrajectoryGenerator               trj_;
  ControlLaw                        ctl_;

private:
  PlanningTask() = default;

  static std::shared_ptr<PlanningTask> s_instance;

  // ---- IRQ ハンドラ・内部処理 ----
  static void timer_irq_handler();
  void tick(uint32_t dt_us);
  void cp_request();
  void pl_req_activate();
  std::shared_ptr<sensing_result_entity_t> get_sensing_entity();

  // ---- 制御フラグ ----
  bool          motor_en    = false;
  bool          suction_en  = false;
  bool          search_mode = false;
  unsigned char w_reset     = 0;

  // ---- 共有エンティティ ----
  std::shared_ptr<SensingTask>              sensing_;
  std::shared_ptr<sensing_result_entity_t>  sensing_result;
  std::shared_ptr<input_param_t>            param;

  // ---- タイマー状態 ----
  uint32_t interval_us_ = 1000;
  uint32_t next_alarm_  = 0;
  uint64_t prev_ts_     = 0;

  // ---- コマンドパイプライン ----
  std::shared_ptr<motion_tgt_val_t> pending_tgt_;
  volatile bool tgt_cmd_pending_  = false;
  int32_t       last_nmr_timestamp_ = -1;
  int           motion_req_timestamp = 0;
  int           pid_req_timestamp    = 0;

  // ---- アクティブコマンド状態 ----
  motion_tgt_val_t                *receive_req = nullptr;
  std::shared_ptr<motion_tgt_val_t> active_tgt_;
  sensor_ctrl_keep_dist_t           right_keep;
  sensor_ctrl_keep_dist_t           left_keep;
  slip_t                            slip_param;
};
