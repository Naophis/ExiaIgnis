#pragma once

#include "pico/sync.h"
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
#include <atomic>
#include <memory>
#include <stdint.h>

class SensingTask;

// Core0 → Core1 コマンド転送用の軽量構造体。
// shared_ptr を排除し、double buffer + atomic index で受け渡す。
struct PlanningCmd {
  new_motion_req_t nmr;
  planning_req_t   pl_req;
};

// 1kHz ハードウェアタイマー IRQ (TIMER1 alarm 0, Core1) で動作する
// planning / control タスク。
// Core0 の main ループから send_command() で目標値を投入し、
// IRQ 内で軌道生成・PID 制御・モーター出力を行う。
class PlanningTask {
public:

  // ---- ライフサイクル ----
  static std::shared_ptr<PlanningTask> create();
  void init(std::shared_ptr<SensingTask> sensing);
  void start_irq();

  // ---- Core0 → Core1 コマンド ----
  // tgt の nmr / pl_req を double buffer へコピーして atomic index を publish する。
  // IRQ 内で shared_ptr 操作が起きないため安全。
  void send_command(const motion_tgt_val_t &tgt);

  // ---- tick 同期 (Core0 から呼ぶ) ----
  void wait_tick();

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
  bool first_req = false; // Core1 専用。初回コマンド受信後に true のまま。

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
  uint64_t start_time_z = 0;
  semaphore_t tick_sem_;

  // ---- コマンドパイプライン (double buffer) ----
  // Core0: cmd_buf_[write_cmd_idx_] に書き込み → pending_cmd_idx_.store(release)
  // Core1: pending_cmd_idx_.load(acquire) → active_cmd_ にコピー → CAS で -1 に戻す
  PlanningCmd      cmd_buf_[2];                     // double buffer 実体
  int              write_cmd_idx_ = 0;              // Core0 専用 (send_command 内のみ)
  std::atomic<int> pending_cmd_idx_{-1};            // -1=なし, 0/1=受取可能バッファ index
  int32_t          last_nmr_timestamp_ = -1;
  int              motion_req_timestamp = 0;
  int              pid_req_timestamp    = 0;

  // ---- アクティブコマンド状態 (Core1 専用) ----
  PlanningCmd      active_cmd_{};                   // IRQ 内でのみ読み書き
  PlanningCmd     *receive_req = nullptr;           // &active_cmd_ を指す
  sensor_ctrl_keep_dist_t right_keep;
  sensor_ctrl_keep_dist_t           left_keep;
  slip_t                            slip_param;
};
