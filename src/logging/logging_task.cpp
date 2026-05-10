#include "logging/logging_task.hpp"
#include "define.hpp"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <stdio.h>

// ============================================================
// psram_heap: バンプアロケータ実装
// ============================================================
namespace psram_heap {
static uint8_t *g_ptr = nullptr;
static uint8_t *g_end = nullptr;

void init(void *base, size_t size) {
  g_ptr = static_cast<uint8_t *>(base);
  g_end = g_ptr + size;
}

void *alloc(size_t bytes, size_t align) {
  uintptr_t p = (reinterpret_cast<uintptr_t>(g_ptr) + align - 1) & ~(align - 1);
  uint8_t *np = reinterpret_cast<uint8_t *>(p) + bytes;
  if (np > g_end)
    return nullptr;
  g_ptr = np;
  return reinterpret_cast<void *>(p);
}

size_t available() {
  return (g_ptr < g_end) ? static_cast<size_t>(g_end - g_ptr) : 0;
}
} // namespace psram_heap

// ============================================================
// LoggingTask
// ============================================================
std::shared_ptr<LoggingTask> LoggingTask::s_instance;

std::shared_ptr<LoggingTask> LoggingTask::create() {
  s_instance = std::shared_ptr<LoggingTask>(new LoggingTask());
  return s_instance;
}

void LoggingTask::init(void *psram_base, size_t psram_size,
                       size_t max_entries) {
  psram_heap::init(psram_base, psram_size);
  log_cap_ = max_entries;
  log_vec_.reserve(max_entries); // PSRAM に一括確保 (以降 realloc なし)
  printf("[LoggingTask] PSRAM %zu KB, cap=%zu entries (%zu KB), entry=%zu B\n",
         psram_size / 1024, max_entries,
         (max_entries * sizeof(log_data_t2)) / 1024, sizeof(log_data_t2));
}

void LoggingTask::start() {
  log_vec_.clear(); // size=0 に戻す (capacity・PSRAM 確保は維持)
  __dmb();
  active_ = true;
  printf("[LoggingTask] start\n");
}

void LoggingTask::stop() {
  active_ = false;
  __dmb(); // Core1 の最後の push_back が Core0 から見えるようにバリア
  printf("[LoggingTask] stop: %zu entries\n", log_vec_.size());
}

// ============================================================
// Core1 (planning IRQ) から呼ぶ高速パス
// ============================================================
void LoggingTask::append_from_irq(const sensing_result_entity_t &sr,
                                   const motion_tgt_val_t &tv) {
  auto *self = s_instance.get();
  if (!self || !self->active_)
    return;

  if (tv.calc_time_diff > 3000)
    return;

  if (self->log_vec_.size() >= self->log_cap_) {
    self->active_ = false;
    return;
  }

  log_data_t2 ld{};

  ld.img_v    = floatToHalf(tv.ego_in.v);
  ld.v_l      = floatToHalf(sr.ego.v_l);
  ld.v_c      = floatToHalf(sr.ego.v_c);
  ld.v_c2     = floatToHalf(sr.ego.v_kf);
  ld.v_r      = floatToHalf(sr.ego.v_r);
  ld.v_r_enc  = static_cast<int16_t>(sr.encoder.right);
  ld.v_l_enc  = static_cast<int16_t>(sr.encoder.left);
  ld.accl     = floatToHalf(tv.ego_in.accl / 1000);
  ld.accl_x   = floatToHalf(sr.ego.w_kf);
  ld.dist_kf  = floatToHalf(sr.ego.dist_kf);

  ld.img_w  = floatToHalf(tv.ego_in.w);
  ld.w_lp   = floatToHalf(sr.ego.w_lp);
  ld.alpha  = floatToHalf(tv.ego_in.alpha);

  ld.img_dist = floatToHalf(tv.ego_in.img_dist);
  ld.dist     = floatToHalf(tv.ego_in.dist);

  ld.img_ang = floatToHalf(tv.ego_in.img_ang * 180.0f / m_PI);
  ld.ang     = floatToHalf(tv.ego_in.ang     * 180.0f / m_PI);
  ld.ang_kf  = floatToHalf(sr.ego.ang_kf     * 180.0f / m_PI);

  ld.left90_lp   = static_cast<int16_t>(sr.led_sen.left90.raw);
  ld.left45_lp   = static_cast<int16_t>(sr.led_sen.left45.raw);
  ld.right45_lp  = static_cast<int16_t>(sr.led_sen.right45.raw);
  ld.right90_lp  = static_cast<int16_t>(sr.led_sen.right90.raw);
  ld.left45_2_lp  = static_cast<int16_t>(sr.led_sen.left45_2.raw);
  ld.right45_2_lp = static_cast<int16_t>(sr.led_sen.right45_2.raw);
  ld.left45_3_lp  = static_cast<int16_t>(sr.led_sen.left45_3.raw);
  ld.right45_3_lp = static_cast<int16_t>(sr.led_sen.right45_3.raw);

  ld.battery_lp = floatToHalf(sr.ego.battery_raw);

  ld.duty_l = floatToHalf(sr.ego.duty.duty_l);
  ld.duty_r = floatToHalf(sr.ego.duty.duty_r);

  ld.ff_duty_front = floatToHalf(sr.ego.duty.ff_duty_front);
  ld.ff_duty_roll  = floatToHalf(sr.ego.duty.ff_duty_roll);
  ld.ff_duty_rpm_r = floatToHalf(sr.ego.duty.ff_duty_rpm_r);
  ld.ff_duty_rpm_l = floatToHalf(sr.ego.duty.ff_duty_rpm_l);

  ld.motion_type     = static_cast<uint8_t>(tv.motion_type);
  ld.duty_sensor_ctrl = floatToHalf(sr.ego.duty.sen);

  ld.sen_log_l45   = floatToHalf(sr.sen.l45.sensor_dist);
  ld.sen_log_r45   = floatToHalf(sr.sen.r45.sensor_dist);
  ld.sen_log_l45_2 = floatToHalf(sr.sen.l45_2.sensor_dist);
  ld.sen_log_r45_2 = floatToHalf(sr.sen.r45_2.sensor_dist);
  ld.sen_log_l45_3 = floatToHalf(sr.sen.l45_3.sensor_dist);
  ld.sen_log_r45_3 = floatToHalf(sr.sen.r45_3.sensor_dist);

  ld.motion_timestamp = static_cast<int16_t>(tv.nmr.timstamp);
  ld.sen_calc_time    = sr.calc_time;
  ld.sen_calc_time2   = sr.calc_time2;
  ld.pln_calc_time    = tv.calc_time;
  ld.pln_time_diff    = tv.calc_time_diff;

  ld.pos_x = floatToHalf(sr.ego.pos_x);
  ld.pos_y = floatToHalf(sr.ego.pos_y);

  ld.knym_v    = floatToHalf(sr.ego.knym_v);
  ld.knym_w    = floatToHalf(sr.ego.knym_w);
  ld.odm_x     = floatToHalf(sr.ego.odm_x);
  ld.odm_y     = floatToHalf(sr.ego.odm_y);
  ld.odm_theta = floatToHalf(sr.ego.odm_theta);
  ld.kim_x     = floatToHalf(sr.ego.kim_x);
  ld.kim_y     = floatToHalf(sr.ego.kim_y);
  ld.kim_theta = floatToHalf(sr.ego.kim_theta);

  ld.duty_suction  = floatToHalf(tv.duty_suction);
  ld.ang_kf_sum    = floatToHalf(sr.ang_kf_sum);
  ld.img_ang_sum   = floatToHalf(sr.img_ang_sum);

  // error_entity (pid_error_entity_t) は引数外のため 0 のまま

  self->log_vec_.push_back(ld);
}

// ============================================================
// USB CDC に CSV 出力 (Core0 から stop() 後に呼ぶ)
// ============================================================
void LoggingTask::dump_csv() const {
  const size_t n = log_vec_.size();
  printf("ready___:%zu\n", sizeof(log_data_t2));
  sleep_ms(50);

  printf("index,"
         "img_v,v_c,v_c2,v_l,v_r,v_l_enc,v_r_enc,accl,accl_x,"
         "img_w,w_lp,alpha,img_dist,dist,img_ang,ang,ang_kf,"
         "left90,left45,right45,right90,"
         "left45_2,right45_2,left45_3,right45_3,"
         "battery,duty_l,duty_r,duty_sen,duty_suction,"
         "sen_l45,sen_r45,sen_l45_2,sen_r45_2,sen_l45_3,sen_r45_3,"
         "motion_type,timestamp,"
         "sen_calc_time,sen_calc_time2,pln_calc_time,pln_time_diff,"
         "ff_front,ff_roll,ff_rpm_r,ff_rpm_l,"
         "pos_x,pos_y,odm_x,odm_y,odm_theta,kim_x,kim_y,kim_theta,"
         "knym_v,knym_w,ang_kf_sum,img_ang_sum\n");

  int flush_cnt = 0;
  for (size_t i = 0; i < n; ++i) {
    const auto &e = log_vec_[i];
    printf("%zu,"
           "%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f,"
           "%.4f,%.4f,%.4f,%.2f,%.2f,%.4f,%.4f,%.4f,"
           "%d,%d,%d,%d,"
           "%d,%d,%d,%d,"
           "%.3f,%.3f,%.3f,%.3f,%.3f,"
           "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
           "%u,%d,"
           "%d,%d,%d,%d,"
           "%.3f,%.3f,%.3f,%.3f,"
           "%.2f,%.2f,%.2f,%.2f,%.4f,%.2f,%.2f,%.4f,"
           "%.3f,%.3f,%.4f,%.4f\n",
           i,
           halfToFloat(e.img_v), halfToFloat(e.v_c), halfToFloat(e.v_c2),
           halfToFloat(e.v_l), halfToFloat(e.v_r),
           (int)e.v_l_enc, (int)e.v_r_enc,
           halfToFloat(e.accl), halfToFloat(e.accl_x),
           halfToFloat(e.img_w), halfToFloat(e.w_lp), halfToFloat(e.alpha),
           halfToFloat(e.img_dist), halfToFloat(e.dist),
           halfToFloat(e.img_ang), halfToFloat(e.ang), halfToFloat(e.ang_kf),
           (int)e.left90_lp, (int)e.left45_lp,
           (int)e.right45_lp, (int)e.right90_lp,
           (int)e.left45_2_lp, (int)e.right45_2_lp,
           (int)e.left45_3_lp, (int)e.right45_3_lp,
           halfToFloat(e.battery_lp),
           halfToFloat(e.duty_l), halfToFloat(e.duty_r),
           halfToFloat(e.duty_sensor_ctrl), halfToFloat(e.duty_suction),
           halfToFloat(e.sen_log_l45), halfToFloat(e.sen_log_r45),
           halfToFloat(e.sen_log_l45_2), halfToFloat(e.sen_log_r45_2),
           halfToFloat(e.sen_log_l45_3), halfToFloat(e.sen_log_r45_3),
           (unsigned)e.motion_type, (int)e.motion_timestamp,
           (int)e.sen_calc_time, (int)e.sen_calc_time2,
           (int)e.pln_calc_time, (int)e.pln_time_diff,
           halfToFloat(e.ff_duty_front), halfToFloat(e.ff_duty_roll),
           halfToFloat(e.ff_duty_rpm_r), halfToFloat(e.ff_duty_rpm_l),
           halfToFloat(e.pos_x), halfToFloat(e.pos_y),
           halfToFloat(e.odm_x), halfToFloat(e.odm_y),
           halfToFloat(e.odm_theta),
           halfToFloat(e.kim_x), halfToFloat(e.kim_y),
           halfToFloat(e.kim_theta),
           halfToFloat(e.knym_v), halfToFloat(e.knym_w),
           halfToFloat(e.ang_kf_sum), halfToFloat(e.img_ang_sum));

    // USB CDC バッファあふれ防止: 50行ごとに 1ms 待つ
    if (++flush_cnt >= 50) {
      flush_cnt = 0;
      sleep_ms(1);
    }
  }
  printf("end___\n");
}

// ============================================================
// UART にバイナリ出力 (Core0 から stop() 後に呼ぶ)
// プロトコル: [magic:4][count:4][entry_sz:4][data:count*entry_sz][magic:4]
// ※ uart_init() は呼び出し元で実施すること
// ============================================================
void LoggingTask::dump_binary() const {
  uart_inst_t *u = UART_ID; // define.hpp: uart1

  const uint32_t magic = 0x4C4F4700u; // "LOG\0"
  const uint32_t count = static_cast<uint32_t>(log_vec_.size());
  const uint32_t entry_sz = static_cast<uint32_t>(sizeof(log_data_t2));

  uart_write_blocking(u, reinterpret_cast<const uint8_t *>(&magic), 4);
  uart_write_blocking(u, reinterpret_cast<const uint8_t *>(&count), 4);
  uart_write_blocking(u, reinterpret_cast<const uint8_t *>(&entry_sz), 4);
  uart_write_blocking(u, reinterpret_cast<const uint8_t *>(log_vec_.data()),
                      log_vec_.size() * sizeof(log_data_t2));
  uart_write_blocking(u, reinterpret_cast<const uint8_t *>(&magic), 4);

  printf("[LoggingTask] binary dump done: %zu entries\n", log_vec_.size());
}

void LoggingTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
}
std::shared_ptr<sensing_result_entity_t> LoggingTask::get_sensing_entity() {
  return sensing_result;
}
void LoggingTask::set_planning_task(std::shared_ptr<PlanningTask> &_pt) {
  pt = _pt;
}
void LoggingTask::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param) {
  param = _param;
}