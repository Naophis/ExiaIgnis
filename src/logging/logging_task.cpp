#include "logging/logging_task.hpp"
#include "define.hpp"
#include "hardware/structs/qmi.h"
#include "hardware/structs/xip.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "hardware/xip_cache.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
extern "C" {
#include "sfe_psram.h"
}
#include "tusb.h"
#include <stdio.h>
#include <string.h>

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
  // LittleFS 書き込み時に flash_exit_xip() が QMI M1 wfmt/wcmd
  // をリセットするため、 全 LittleFS 操作完了後に QMI M1 を sfe_setup_psram
  // と同じ設定に復元する。 PSRAM ハードウェアは QPI
  // モードを維持しているため、レジスタ復元のみ行う。 sfe_psram_update_timing()
  // は direct_csr を使わないので Core1 動作中も安全。
  sfe_psram_update_timing();

  // rfmt: QPI Quad Read (0xEB), 24 ダミークロック、サフィックスなし
  // (sfe_psram.c と同一)
  qmi_hw->m[1].rfmt =
      (QMI_M1_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M1_RFMT_PREFIX_WIDTH_LSB) |
      (QMI_M1_RFMT_ADDR_WIDTH_VALUE_Q << QMI_M1_RFMT_ADDR_WIDTH_LSB) |
      (QMI_M1_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M1_RFMT_SUFFIX_WIDTH_LSB) |
      (QMI_M1_RFMT_DUMMY_WIDTH_VALUE_Q << QMI_M1_RFMT_DUMMY_WIDTH_LSB) |
      (QMI_M1_RFMT_DUMMY_LEN_VALUE_24 << QMI_M1_RFMT_DUMMY_LEN_LSB) |
      (QMI_M1_RFMT_DATA_WIDTH_VALUE_Q << QMI_M1_RFMT_DATA_WIDTH_LSB) |
      (QMI_M1_RFMT_PREFIX_LEN_VALUE_8 << QMI_M1_RFMT_PREFIX_LEN_LSB) |
      (QMI_M1_RFMT_SUFFIX_LEN_VALUE_NONE << QMI_M1_RFMT_SUFFIX_LEN_LSB);
  qmi_hw->m[1].rcmd = 0xEBu << QMI_M1_RCMD_PREFIX_LSB; // QUAD_READ, no suffix

  // wfmt: QPI Quad Write (0x38), ダミーなし、サフィックスなし
  qmi_hw->m[1].wfmt =
      (QMI_M1_WFMT_PREFIX_WIDTH_VALUE_Q << QMI_M1_WFMT_PREFIX_WIDTH_LSB) |
      (QMI_M1_WFMT_ADDR_WIDTH_VALUE_Q << QMI_M1_WFMT_ADDR_WIDTH_LSB) |
      (QMI_M1_WFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M1_WFMT_SUFFIX_WIDTH_LSB) |
      (QMI_M1_WFMT_DUMMY_WIDTH_VALUE_Q << QMI_M1_WFMT_DUMMY_WIDTH_LSB) |
      (QMI_M1_WFMT_DUMMY_LEN_VALUE_NONE << QMI_M1_WFMT_DUMMY_LEN_LSB) |
      (QMI_M1_WFMT_DATA_WIDTH_VALUE_Q << QMI_M1_WFMT_DATA_WIDTH_LSB) |
      (QMI_M1_WFMT_PREFIX_LEN_VALUE_8 << QMI_M1_WFMT_PREFIX_LEN_LSB) |
      (QMI_M1_WFMT_SUFFIX_LEN_VALUE_NONE << QMI_M1_WFMT_SUFFIX_LEN_LSB);
  qmi_hw->m[1].wcmd = 0x38u << QMI_M1_WCMD_PREFIX_LSB; // QUAD_WRITE, no suffix

  hw_set_bits(&xip_ctrl_hw->ctrl, XIP_CTRL_WRITABLE_M1_BITS);

  psram_heap::init(psram_base, psram_size);

  // ── PSRAM 疎通確認 ──────────────────────────────────────────────────
  printf("[LoggingTask] XIP_CTRL=0x%08X\n", (unsigned)xip_ctrl_hw->ctrl);
  printf("[LoggingTask] QMI M1: timing=0x%08X rfmt=0x%08X rcmd=0x%08X "
         "wfmt=0x%08X wcmd=0x%08X\n",
         (unsigned)qmi_hw->m[1].timing, (unsigned)qmi_hw->m[1].rfmt,
         (unsigned)qmi_hw->m[1].rcmd, (unsigned)qmi_hw->m[1].wfmt,
         (unsigned)qmi_hw->m[1].wcmd);

  volatile uint32_t *p = reinterpret_cast<volatile uint32_t *>(0x11000000u);

  // Test A: キャッシュ書き込み → キャッシュ読み出し (write-back 動作確認)
  p[0] = 0xDEADBEEFu;
  p[1] = 0x12345678u;
  __dsb();
  __isb();
  bool ok_a = (p[0] == 0xDEADBEEFu && p[1] == 0x12345678u);
  printf("[PSRAM] A cached-W/R    : %s  0x%08X 0x%08X\n", ok_a ? "OK" : "FAIL",
         (unsigned)p[0], (unsigned)p[1]);

  // Test B: clean_all (wfmt で PSRAM 書き込み) → キャッシュミスで rfmt
  // 読み出し確認 RP2350-E11 workaround により clean_all
  // はフラッシュ後にキャッシュラインを無効化する。
  // 次の読み出しはキャッシュミスとなり rfmt (0xEB) で PSRAM から直接読む。
  xip_cache_clean_all();
  __dsb();
  __isb();
  bool ok_b = (p[0] == 0xDEADBEEFu && p[1] == 0x12345678u);
  printf("[PSRAM] B clean+miss-R  : %s  0x%08X 0x%08X\n", ok_b ? "OK" : "FAIL",
         (unsigned)p[0], (unsigned)p[1]);

  // Test C: 別オフセットに書き込み → clean_range + invalidate_range →
  // キャッシュミス読み出し XIP_CACHE_LINE_SIZE=8 byte
  // 単位でアライン。オフセット=0x11000008-0x10000000=0x01000008
  p[2] = 0xCAFEBABEu; // 0x11000008
  p[3] = 0xABCDEF01u; // 0x1100000C
  __dsb();
  xip_cache_clean_range(0x01000008u, 8u); // wfmt で PSRAM に書き込み
  xip_cache_invalidate_range(0x01000008u,
                             8u); // キャッシュを無効化して rfmt 読み出しを強制
  __dsb();
  __isb();
  bool ok_c = (p[2] == 0xCAFEBABEu && p[3] == 0xABCDEF01u);
  printf("[PSRAM] C clean+inv+R   : %s  0x%08X 0x%08X\n", ok_c ? "OK" : "FAIL",
         (unsigned)p[2], (unsigned)p[3]);

  const bool psram_ok = ok_a && ok_b;
  printf("[LoggingTask] PSRAM %s\n", psram_ok ? "OK" : "FAIL");

  log_cap_ = max_entries;
  log_vec_.reserve(max_entries); // PSRAM に一括確保 (以降 realloc なし)

  constexpr size_t send_buf_bytes = 504 * SEND_BATCH;  // 480 (ls1-10) + 24 (ls11)
  send_buf_ = static_cast<uint8_t *>(psram_heap::alloc(send_buf_bytes));
  if (!send_buf_) panic("PSRAM OOM: send_buf");

  printf("[LoggingTask] PSRAM %zu KB, cap=%zu entries (%zu KB), entry=%zu B, send_buf=%zu KB\n",
         psram_size / 1024, max_entries,
         (max_entries * sizeof(log_data_t2)) / 1024, sizeof(log_data_t2),
         send_buf_bytes / 1024);
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
  // RP2350 の XIP キャッシュは write-back のため、Core1 の PSRAM 書き込みが
  // キャッシュに留まっている可能性がある。clean_all() で物理 PSRAM に flush
  // する。
  xip_cache_clean_all();
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

  ld.img_v = floatToHalf(tv.ego_in.v);
  ld.v_l = floatToHalf(sr.ego.v_l);
  ld.v_c = floatToHalf(sr.ego.v_c);
  ld.v_c2 = floatToHalf(sr.ego.v_kf);
  ld.v_r = floatToHalf(sr.ego.v_r);
  ld.v_r_enc = static_cast<int16_t>(sr.encoder.right);
  ld.v_l_enc = static_cast<int16_t>(sr.encoder.left);
  ld.accl = floatToHalf(tv.ego_in.accl / 1000);
  ld.accl_x = floatToHalf(sr.ego.w_kf);
  ld.dist_kf = floatToHalf(sr.ego.dist_kf);

  ld.img_w = floatToHalf(tv.ego_in.w);
  ld.w_lp = floatToHalf(sr.ego.w_lp);
  ld.alpha = floatToHalf(tv.ego_in.alpha);

  ld.img_dist = floatToHalf(tv.ego_in.img_dist);
  ld.dist = floatToHalf(tv.ego_in.dist);

  ld.img_ang = floatToHalf(tv.ego_in.img_ang * 180.0f / m_PI);
  ld.ang = floatToHalf(tv.ego_in.ang * 180.0f / m_PI);
  ld.ang_kf = floatToHalf(sr.ego.ang_kf * 180.0f / m_PI);

  ld.left90_lp = static_cast<int16_t>(sr.led_sen.left90.raw);
  ld.left45_lp = static_cast<int16_t>(sr.led_sen.left45.raw);
  ld.right45_lp = static_cast<int16_t>(sr.led_sen.right45.raw);
  ld.right90_lp = static_cast<int16_t>(sr.led_sen.right90.raw);
  ld.left45_2_lp = static_cast<int16_t>(sr.led_sen.left45_2.raw);
  ld.right45_2_lp = static_cast<int16_t>(sr.led_sen.right45_2.raw);
  ld.left45_3_lp = static_cast<int16_t>(sr.led_sen.left45_3.raw);
  ld.right45_3_lp = static_cast<int16_t>(sr.led_sen.right45_3.raw);

  ld.battery_lp = floatToHalf(sr.ego.battery_raw);

  ld.duty_l = floatToHalf(sr.ego.duty.duty_l);
  ld.duty_r = floatToHalf(sr.ego.duty.duty_r);

  ld.ff_duty_front = floatToHalf(sr.ego.duty.ff_duty_front);
  ld.ff_duty_roll = floatToHalf(sr.ego.duty.ff_duty_roll);
  ld.ff_duty_rpm_r = floatToHalf(sr.ego.duty.ff_duty_rpm_r);
  ld.ff_duty_rpm_l = floatToHalf(sr.ego.duty.ff_duty_rpm_l);

  ld.motion_type = static_cast<uint8_t>(tv.motion_type);
  ld.duty_sensor_ctrl = floatToHalf(sr.ego.duty.sen);

  ld.sen_log_l45 = floatToHalf(sr.sen.l45.sensor_dist);
  ld.sen_log_r45 = floatToHalf(sr.sen.r45.sensor_dist);
  ld.sen_log_l45_2 = floatToHalf(sr.sen.l45_2.sensor_dist);
  ld.sen_log_r45_2 = floatToHalf(sr.sen.r45_2.sensor_dist);
  ld.sen_log_l45_3 = floatToHalf(sr.sen.l45_3.sensor_dist);
  ld.sen_log_r45_3 = floatToHalf(sr.sen.r45_3.sensor_dist);

  ld.motion_timestamp = static_cast<int16_t>(tv.nmr.timstamp);
  ld.sen_calc_time = sr.calc_time;
  ld.sen_calc_time2 = sr.calc_time2;
  ld.pln_calc_time = tv.calc_time;
  ld.pln_time_diff = tv.calc_time_diff;
  ld.pln_t_ego      = tv.pln_t_ego;
  ld.pln_t_sensor   = tv.pln_t_sensor;
  ld.pln_t_trj      = tv.pln_t_trj;
  ld.pln_t_kanayama = tv.pln_t_kanayama;
  ld.pln_t_copy     = tv.pln_t_copy;
  ld.pln_t_ctl      = tv.pln_t_ctl;

  ld.pos_x = floatToHalf(sr.ego.pos_x);
  ld.pos_y = floatToHalf(sr.ego.pos_y);

  ld.knym_v = floatToHalf(sr.ego.knym_v);
  ld.knym_w = floatToHalf(sr.ego.knym_w);
  ld.odm_x = floatToHalf(sr.ego.odm_x);
  ld.odm_y = floatToHalf(sr.ego.odm_y);
  ld.odm_theta = floatToHalf(sr.ego.odm_theta);
  ld.kim_x = floatToHalf(sr.ego.kim_x);
  ld.kim_y = floatToHalf(sr.ego.kim_y);
  ld.kim_theta = floatToHalf(sr.ego.kim_theta);

  ld.duty_suction = floatToHalf(tv.duty_suction);
  ld.ang_kf_sum = floatToHalf(sr.ang_kf_sum);
  ld.img_ang_sum = floatToHalf(sr.img_ang_sum);

  const auto *ee = self->error_entity.get();
  if (ee) {
    ld.m_pid_p = floatToHalf(ee->v_val.p);
    ld.m_pid_i = floatToHalf(ee->v_val.i);
    ld.m_pid_i2 = floatToHalf(ee->v_val.i2);
    ld.m_pid_d = floatToHalf(ee->v_val.d);
    if (self->log_vec_.empty() || static_cast<int>(tv.motion_type) == 0 ||
        (ee->v_val.i_val == 0)) {
      ld.m_pid_p_v = floatToHalf(0.0f);
    } else {
      ld.m_pid_p_v = floatToHalf(ee->v_val.p_val);
    }
    ld.m_pid_i_v = floatToHalf(ee->v_val.i_val);
    ld.m_pid_i2_v = floatToHalf(ee->v_val.i2_val);
    ld.m_pid_d_v = floatToHalf(ee->v_val.d_val);

    ld.g_pid_p = floatToHalf(ee->w_val.p);
    ld.g_pid_i = floatToHalf(ee->w_val.i);
    ld.g_pid_i2 = floatToHalf(ee->w_val.i2);
    ld.g_pid_d = floatToHalf(ee->w_val.d);
    ld.g_pid_p_v = floatToHalf(ee->w_val.p_val);
    ld.g_pid_i_v = floatToHalf(ee->w_val.i_val);
    ld.g_pid_i2_v = floatToHalf(ee->w_val.i2_val);
    if (self->log_vec_.empty() || static_cast<int>(tv.motion_type) == 0 ||
        (ee->v_val.i_val == 0)) {
      ld.g_pid_d_v = floatToHalf(0.0f);
    } else {
      ld.g_pid_d_v = floatToHalf(ee->w_val.d_val);
    }

    ld.ang_pid_p = floatToHalf(ee->ang_val.p);
    ld.ang_pid_i = floatToHalf(ee->ang_val.i);
    ld.ang_pid_d = floatToHalf(ee->ang_val.d);
    ld.ang_pid_p_v = floatToHalf(ee->ang_val.p_val);
    ld.ang_pid_i_v = floatToHalf(ee->ang_val.i_val);
    ld.ang_pid_d_v = floatToHalf(ee->ang_val.d_val);

    ld.s_pid_p = floatToHalf(ee->s_val.p);
    ld.s_pid_i = floatToHalf(ee->s_val.i);
    ld.s_pid_i2 = floatToHalf(ee->s_val.i2);
    ld.s_pid_d = floatToHalf(ee->s_val.d);
    ld.s_pid_p_v = floatToHalf(ee->s_val.p_val);
    ld.s_pid_i_v = floatToHalf(ee->s_val.i_val);
    ld.s_pid_i2_v = floatToHalf(ee->s_val.i2_val);
    ld.s_pid_d_v = floatToHalf(ee->s_val.d_val);

    ld.ang_i_bias = floatToHalf(ee->ang_val.i2);
    ld.ang_i_bias_val = floatToHalf(ee->ang_val.i2_val);
    ld.duty_roll = floatToHalf(ee->aw_log.duty_roll);
    ld.duty_roll_before = floatToHalf(ee->aw_log.duty_roll_before);
  }

  self->log_vec_.emplace_back(std::move(ld));
}

// ============================================================
// USB CDC にバイナリ出力 (console.sh / rx_term.js 互換プロトコル)
// Core0 から stop() 後に呼ぶ。
//
// Astraea の print_header() + dump_log() と同じ方式:
//   ready___:<record_bytes>\n   … dump_to_csv_ready フラグ
//   name:type:sizeof(field)\n × 120 フィールド
//   start___\n
//   LogStruct1〜10 を memcpy した 480 byte × n エントリ
//   終端レコード (index=-1)
// ============================================================
void LoggingTask::dump_csv() const {
  const size_t n = log_vec_.size();

  LogStruct1 ls1{};
  LogStruct2 ls2{};
  LogStruct3 ls3{};
  LogStruct4 ls4{};
  LogStruct5 ls5{};
  LogStruct6 ls6{};
  LogStruct7 ls7{};
  LogStruct8 ls8{};
  LogStruct9 ls9{};
  LogStruct10 ls10{};
  LogStruct11 ls11{};

  const int size = sizeof(LogStruct1) + sizeof(LogStruct2) +
                   sizeof(LogStruct3) + sizeof(LogStruct4) +
                   sizeof(LogStruct5) + sizeof(LogStruct6) +
                   sizeof(LogStruct7) + sizeof(LogStruct8) +
                   sizeof(LogStruct9) + sizeof(LogStruct10) +
                   sizeof(LogStruct11);

  printf("ready___:%d\n", size);
  fflush(stdout);
  sleep_ms(50);

  // LogStruct1
  printf("index:int:%d\n", (int)sizeof(ls1.index));
  printf("ideal_v:float:%d\n", (int)sizeof(ls1.ideal_v));
  printf("v_c:float:%d\n", (int)sizeof(ls1.v_c));
  printf("v_c2:float:%d\n", (int)sizeof(ls1.v_c2));
  printf("v_l:float:%d\n", (int)sizeof(ls1.v_l));
  printf("v_r:float:%d\n", (int)sizeof(ls1.v_r));
  printf("v_l_enc:int:%d\n", (int)sizeof(ls1.v_l_enc));
  printf("v_r_enc:int:%d\n", (int)sizeof(ls1.v_r_enc));
  printf("v_l_enc_sin:float:%d\n", (int)sizeof(ls1.v_l_enc_sin));
  printf("v_r_enc_sin:float:%d\n", (int)sizeof(ls1.v_r_enc_sin));
  printf("accl:float:%d\n", (int)sizeof(ls1.accl));
  printf("accl_x:float:%d\n", (int)sizeof(ls1.accl_x));
  // LogStruct2
  printf("ideal_w:float:%d\n", (int)sizeof(ls2.ideal_w));
  printf("w_lp:float:%d\n", (int)sizeof(ls2.w_lp));
  printf("alpha:float:%d\n", (int)sizeof(ls2.alpha));
  printf("ideal_dist:float:%d\n", (int)sizeof(ls2.ideal_dist));
  printf("dist:float:%d\n", (int)sizeof(ls2.dist));
  printf("dist_kf:float:%d\n", (int)sizeof(ls2.dist_kf));
  printf("ideal_ang:float:%d\n", (int)sizeof(ls2.ideal_ang));
  printf("ang:float:%d\n", (int)sizeof(ls2.ang));
  printf("ang_kf:float:%d\n", (int)sizeof(ls2.ang_kf));
  printf("left90:int:%d\n", (int)sizeof(ls2.left90));
  printf("left45:int:%d\n", (int)sizeof(ls2.left45));
  printf("front:int:%d\n", (int)sizeof(ls2.front));
  // LogStruct3
  printf("right45:int:%d\n", (int)sizeof(ls3.right45));
  printf("right90:int:%d\n", (int)sizeof(ls3.right90));
  printf("left90_d:float:%d\n", (int)sizeof(ls3.left90_d));
  printf("left45_d:float:%d\n", (int)sizeof(ls3.left45_d));
  printf("front_d:float:%d\n", (int)sizeof(ls3.front_d));
  printf("right45_d:float:%d\n", (int)sizeof(ls3.right45_d));
  printf("right90_d:float:%d\n", (int)sizeof(ls3.right90_d));
  printf("left90_far_d:float:%d\n", (int)sizeof(ls3.left90_far_d));
  printf("front_far_d:float:%d\n", (int)sizeof(ls3.front_far_d));
  printf("right90_far_d:float:%d\n", (int)sizeof(ls3.right90_far_d));
  printf("battery:float:%d\n", (int)sizeof(ls3.battery));
  printf("duty_l:float:%d\n", (int)sizeof(ls3.duty_l));
  // LogStruct4
  printf("duty_r:float:%d\n", (int)sizeof(ls4.duty_r));
  printf("motion_state:int:%d\n", (int)sizeof(ls4.motion_state));
  printf("duty_sen:float:%d\n", (int)sizeof(ls4.duty_sen));
  printf("dist_mod90:float:%d\n", (int)sizeof(ls4.dist_mod90));
  printf("sen_dist_l45:float:%d\n", (int)sizeof(ls4.sen_dist_l45));
  printf("sen_dist_r45:float:%d\n", (int)sizeof(ls4.sen_dist_r45));
  printf("timestamp:int:%d\n", (int)sizeof(ls4.timestamp));
  printf("sen_calc_time:int:%d\n", (int)sizeof(ls4.sen_calc_time));
  printf("sen_calc_time2:int:%d\n", (int)sizeof(ls4.sen_calc_time2));
  printf("pln_calc_time:int:%d\n", (int)sizeof(ls4.pln_calc_time));
  printf("pln_calc_time2:int:%d\n", (int)sizeof(ls4.pln_calc_time2));
  printf("pln_time_diff:int:%d\n", (int)sizeof(ls4.pln_time_diff));
  // LogStruct5
  printf("m_pid_p:float:%d\n", (int)sizeof(ls5.m_pid_p));
  printf("m_pid_i:float:%d\n", (int)sizeof(ls5.m_pid_i));
  printf("m_pid_i2:float:%d\n", (int)sizeof(ls5.m_pid_i2));
  printf("m_pid_d:float:%d\n", (int)sizeof(ls5.m_pid_d));
  printf("m_pid_p_v:float:%d\n", (int)sizeof(ls5.m_pid_p_v));
  printf("m_pid_i_v:float:%d\n", (int)sizeof(ls5.m_pid_i_v));
  printf("m_pid_i2_v:float:%d\n", (int)sizeof(ls5.m_pid_i2_v));
  printf("m_pid_d_v:float:%d\n", (int)sizeof(ls5.m_pid_d_v));
  printf("g_pid_p:float:%d\n", (int)sizeof(ls5.g_pid_p));
  printf("g_pid_i:float:%d\n", (int)sizeof(ls5.g_pid_i));
  printf("g_pid_i2:float:%d\n", (int)sizeof(ls5.g_pid_i2));
  printf("g_pid_d:float:%d\n", (int)sizeof(ls5.g_pid_d));
  // LogStruct6
  printf("g_pid_p_v:float:%d\n", (int)sizeof(ls6.g_pid_p_v));
  printf("g_pid_i_v:float:%d\n", (int)sizeof(ls6.g_pid_i_v));
  printf("g_pid_i2_v:float:%d\n", (int)sizeof(ls6.g_pid_i2_v));
  printf("g_pid_d_v:float:%d\n", (int)sizeof(ls6.g_pid_d_v));
  printf("s_pid_p:float:%d\n", (int)sizeof(ls6.s_pid_p));
  printf("s_pid_i:float:%d\n", (int)sizeof(ls6.s_pid_i));
  printf("s_pid_i2:float:%d\n", (int)sizeof(ls6.s_pid_i2));
  printf("s_pid_d:float:%d\n", (int)sizeof(ls6.s_pid_d));
  printf("s_pid_p_v:float:%d\n", (int)sizeof(ls6.s_pid_p_v));
  printf("s_pid_i_v:float:%d\n", (int)sizeof(ls6.s_pid_i_v));
  printf("s_pid_i2_v:float:%d\n", (int)sizeof(ls6.s_pid_i2_v));
  printf("s_pid_d_v:float:%d\n", (int)sizeof(ls6.s_pid_d_v));
  // LogStruct7
  printf("ang_pid_p:float:%d\n", (int)sizeof(ls7.ang_pid_p));
  printf("ang_pid_i:float:%d\n", (int)sizeof(ls7.ang_pid_i));
  printf("ang_pid_d:float:%d\n", (int)sizeof(ls7.ang_pid_d));
  printf("ang_pid_p_v:float:%d\n", (int)sizeof(ls7.ang_pid_p_v));
  printf("ang_pid_i_v:float:%d\n", (int)sizeof(ls7.ang_pid_i_v));
  printf("ang_pid_d_v:float:%d\n", (int)sizeof(ls7.ang_pid_d_v));
  printf("ff_duty_front:float:%d\n", (int)sizeof(ls7.ff_duty_front));
  printf("ff_duty_roll:float:%d\n", (int)sizeof(ls7.ff_duty_roll));
  printf("ff_duty_rpm_r:float:%d\n", (int)sizeof(ls7.ff_duty_rpm_r));
  printf("ff_duty_rpm_l:float:%d\n", (int)sizeof(ls7.ff_duty_rpm_l));
  printf("x:float:%d\n", (int)sizeof(ls7.x));
  printf("y:float:%d\n", (int)sizeof(ls7.y));
  // LogStruct8
  printf("right45_2:int:%d\n", (int)sizeof(ls8.right45_2));
  printf("right45_3:int:%d\n", (int)sizeof(ls8.right45_3));
  printf("left45_2:int:%d\n", (int)sizeof(ls8.left45_2));
  printf("left45_3:int:%d\n", (int)sizeof(ls8.left45_3));
  printf("right45_2_d:float:%d\n", (int)sizeof(ls8.right45_2_d));
  printf("right45_3_d:float:%d\n", (int)sizeof(ls8.right45_3_d));
  printf("left45_2_d:float:%d\n", (int)sizeof(ls8.left45_2_d));
  printf("left45_3_d:float:%d\n", (int)sizeof(ls8.left45_3_d));
  printf("sen_dist_l45_2:float:%d\n", (int)sizeof(ls8.sen_dist_l45_2));
  printf("sen_dist_r45_2:float:%d\n", (int)sizeof(ls8.sen_dist_r45_2));
  printf("sen_dist_l45_3:float:%d\n", (int)sizeof(ls8.sen_dist_l45_3));
  printf("sen_dist_r45_3:float:%d\n", (int)sizeof(ls8.sen_dist_r45_3));
  // LogStruct9
  printf("knym_v:float:%d\n", (int)sizeof(ls9.knym_v));
  printf("knym_w:float:%d\n", (int)sizeof(ls9.knym_w));
  printf("odm_x:float:%d\n", (int)sizeof(ls9.odm_x));
  printf("odm_y:float:%d\n", (int)sizeof(ls9.odm_y));
  printf("odm_theta:float:%d\n", (int)sizeof(ls9.odm_theta));
  printf("kim_x:float:%d\n", (int)sizeof(ls9.kim_x));
  printf("kim_y:float:%d\n", (int)sizeof(ls9.kim_y));
  printf("kim_theta:float:%d\n", (int)sizeof(ls9.kim_theta));
  printf("ang_i_bias:float:%d\n", (int)sizeof(ls9.ang_i_bias));
  printf("ang_i_bias_val:float:%d\n", (int)sizeof(ls9.ang_i_bias_val));
  printf("left90_d_diff:float:%d\n", (int)sizeof(ls9.left90_d_diff));
  printf("right90_d_diff:float:%d\n", (int)sizeof(ls9.right90_d_diff));
  // LogStruct10
  printf("right45_3_d_diff:float:%d\n", (int)sizeof(ls10.right45_3_d_diff));
  printf("right45_2_d_diff:float:%d\n", (int)sizeof(ls10.right45_2_d_diff));
  printf("right45_d_diff:float:%d\n", (int)sizeof(ls10.right45_d_diff));
  printf("left45_d_diff:float:%d\n", (int)sizeof(ls10.left45_d_diff));
  printf("left45_2_d_diff:float:%d\n", (int)sizeof(ls10.left45_2_d_diff));
  printf("left45_3_d_diff:float:%d\n", (int)sizeof(ls10.left45_3_d_diff));
  printf("duty_suction:float:%d\n", (int)sizeof(ls10.duty_suction));
  printf("duty_roll:float:%d\n", (int)sizeof(ls10.duty_roll));
  printf("ang_kf_sum:float:%d\n", (int)sizeof(ls10.ang_kf_sum));
  printf("img_ang_sum:float:%d\n", (int)sizeof(ls10.img_ang_sum));
  printf("duty_roll_before:float:%d\n", (int)sizeof(ls10.duty_roll_before));
  printf("reserve5:int:%d\n", (int)sizeof(ls10.reserve5));
  // LogStruct11
  printf("pln_t_ego:int:%d\n",      (int)sizeof(ls11.pln_t_ego));
  printf("pln_t_sensor:int:%d\n",   (int)sizeof(ls11.pln_t_sensor));
  printf("pln_t_trj:int:%d\n",      (int)sizeof(ls11.pln_t_trj));
  printf("pln_t_kanayama:int:%d\n", (int)sizeof(ls11.pln_t_kanayama));
  printf("pln_t_copy:int:%d\n",     (int)sizeof(ls11.pln_t_copy));
  printf("pln_t_ctl:int:%d\n",      (int)sizeof(ls11.pln_t_ctl));

  fflush(stdout);
  sleep_ms(50);
  printf("start___\n");
  fflush(stdout);
  sleep_ms(100);

  // CRLF 変換を無効化してバイナリデータを送信 (\n バイトが壊れるのを防ぐ)
  stdio_set_translate_crlf(&stdio_usb, false);

  constexpr int BATCH = SEND_BATCH;
  uint8_t *const send_buf = send_buf_;
  size_t batch_off = 0;
  int batch_cnt = 0;

  ls1.index = 0;
  float left45_d_z = 0, right45_d_z = 0;
  float left45_2_d_z = 0, right45_2_d_z = 0;
  float left45_3_d_z = 0, right45_3_d_z = 0;
  constexpr float th = 10.0f;

  int flush_cnt = 0;
  long c = 0;
  // while(1){
  //       if(c>86000){
  //   break;
  //   }
  for (const auto &e : log_vec_) {
    ls1.index++;
    c++;
    // if(c>86000){
    // break;
    // }
    // if (ls1.index >= (int32_t)n) break;

    // --- ls1 ---
    ls1.ideal_v = halfToFloat(e.img_v);
    ls1.v_c = halfToFloat(e.v_c);
    ls1.v_c2 = halfToFloat(e.v_c2);
    ls1.v_l = halfToFloat(e.v_l);
    ls1.v_r = halfToFloat(e.v_r);
    ls1.v_l_enc = e.v_l_enc;
    ls1.v_r_enc = e.v_r_enc;
    ls1.v_l_enc_sin = 0.0f;
    ls1.v_r_enc_sin = 0.0f;
    ls1.accl = halfToFloat(e.accl) * 1000.0f;
    ls1.accl_x = halfToFloat(e.accl_x);
    // --- ls2 ---
    ls2.ideal_w = halfToFloat(e.img_w);
    ls2.w_lp = halfToFloat(e.w_lp);
    ls2.alpha = halfToFloat(e.alpha);
    ls2.ideal_dist = halfToFloat(e.img_dist);
    ls2.dist = halfToFloat(e.dist);
    ls2.dist_kf = halfToFloat(e.dist_kf);
    ls2.ideal_ang = halfToFloat(e.img_ang);
    ls2.ang = halfToFloat(e.ang);
    ls2.ang_kf = halfToFloat(e.ang_kf);
    ls2.left90 = e.left90_lp;
    ls2.left45 = e.left45_lp;
    ls2.front = (e.left90_lp + e.right90_lp) / 2;
    // --- ls3 ---
    ls3.right45 = e.right45_lp;
    ls3.right90 = e.right90_lp;
    ls3.left90_d = 0.0f;
    ls3.left45_d = halfToFloat(e.sen_log_l45);
    ls3.front_d = 0.0f;
    ls3.right45_d = halfToFloat(e.sen_log_r45);
    ls3.right90_d = 0.0f;
    ls3.left90_far_d = 0.0f;
    ls3.front_far_d = 0.0f;
    ls3.right90_far_d = 0.0f;
    ls3.battery = halfToFloat(e.battery_lp);
    ls3.duty_l = halfToFloat(e.duty_l);
    // --- ls4 ---
    ls4.duty_r = halfToFloat(e.duty_r);
    ls4.motion_state = e.motion_type;
    ls4.duty_sen = halfToFloat(e.duty_sensor_ctrl);
    ls4.dist_mod90 = 0.0f;
    ls4.sen_dist_l45 = halfToFloat(e.sen_log_l45);
    ls4.sen_dist_r45 = halfToFloat(e.sen_log_r45);
    ls4.timestamp = e.motion_timestamp;
    ls4.sen_calc_time = e.sen_calc_time;
    ls4.sen_calc_time2 = e.sen_calc_time2;
    ls4.pln_calc_time = e.pln_calc_time;
    ls4.pln_calc_time2 = 0;
    ls4.pln_time_diff = e.pln_time_diff;
    // --- ls5 ---
    ls5.m_pid_p = halfToFloat(e.m_pid_p);
    ls5.m_pid_i = halfToFloat(e.m_pid_i);
    ls5.m_pid_i2 = halfToFloat(e.m_pid_i2);
    ls5.m_pid_d = halfToFloat(e.m_pid_d);
    ls5.m_pid_p_v = halfToFloat(e.m_pid_p_v);
    ls5.m_pid_i_v = halfToFloat(e.m_pid_i_v);
    ls5.m_pid_i2_v = halfToFloat(e.m_pid_i2_v);
    ls5.m_pid_d_v = halfToFloat(e.m_pid_d_v);
    ls5.g_pid_p = halfToFloat(e.g_pid_p);
    ls5.g_pid_i = halfToFloat(e.g_pid_i);
    ls5.g_pid_i2 = halfToFloat(e.g_pid_i2);
    ls5.g_pid_d = halfToFloat(e.g_pid_d);
    // --- ls6 ---
    ls6.g_pid_p_v = halfToFloat(e.g_pid_p_v);
    ls6.g_pid_i_v = halfToFloat(e.g_pid_i_v);
    ls6.g_pid_i2_v = halfToFloat(e.g_pid_i2_v);
    ls6.g_pid_d_v = halfToFloat(e.g_pid_d_v);
    ls6.s_pid_p = halfToFloat(e.s_pid_p);
    ls6.s_pid_i = halfToFloat(e.s_pid_i);
    ls6.s_pid_i2 = halfToFloat(e.s_pid_i2);
    ls6.s_pid_d = halfToFloat(e.s_pid_d);
    ls6.s_pid_p_v = halfToFloat(e.s_pid_p_v);
    ls6.s_pid_i_v = halfToFloat(e.s_pid_i_v);
    ls6.s_pid_i2_v = halfToFloat(e.s_pid_i2_v);
    ls6.s_pid_d_v = halfToFloat(e.s_pid_d_v);
    // --- ls7 ---
    ls7.ang_pid_p = halfToFloat(e.ang_pid_p);
    ls7.ang_pid_i = halfToFloat(e.ang_pid_i);
    ls7.ang_pid_d = halfToFloat(e.ang_pid_d);
    ls7.ang_pid_p_v = halfToFloat(e.ang_pid_p_v);
    ls7.ang_pid_i_v = halfToFloat(e.ang_pid_i_v);
    ls7.ang_pid_d_v = halfToFloat(e.ang_pid_d_v);
    ls7.ff_duty_front = halfToFloat(e.ff_duty_front);
    ls7.ff_duty_roll = halfToFloat(e.ff_duty_roll);
    ls7.ff_duty_rpm_r = halfToFloat(e.ff_duty_rpm_r);
    ls7.ff_duty_rpm_l = halfToFloat(e.ff_duty_rpm_l);
    ls7.x = halfToFloat(e.pos_x);
    ls7.y = halfToFloat(e.pos_y);
    // --- ls8 ---
    ls8.right45_2 = e.right45_2_lp;
    ls8.right45_3 = e.right45_3_lp;
    ls8.left45_2 = e.left45_2_lp;
    ls8.left45_3 = e.left45_3_lp;
    ls8.right45_2_d = halfToFloat(e.sen_log_r45_2);
    ls8.right45_3_d = halfToFloat(e.sen_log_r45_3);
    ls8.left45_2_d = halfToFloat(e.sen_log_l45_2);
    ls8.left45_3_d = halfToFloat(e.sen_log_l45_3);
    ls8.sen_dist_l45_2 = halfToFloat(e.sen_log_l45_2);
    ls8.sen_dist_r45_2 = halfToFloat(e.sen_log_r45_2);
    ls8.sen_dist_l45_3 = halfToFloat(e.sen_log_l45_3);
    ls8.sen_dist_r45_3 = halfToFloat(e.sen_log_r45_3);
    // --- ls9 ---
    ls9.knym_v = halfToFloat(e.knym_v);
    ls9.knym_w = halfToFloat(e.knym_w);
    ls9.odm_x = halfToFloat(e.odm_x);
    ls9.odm_y = halfToFloat(e.odm_y);
    ls9.odm_theta = halfToFloat(e.odm_theta) * 180.0f / m_PI;
    ls9.kim_x = halfToFloat(e.kim_x);
    ls9.kim_y = halfToFloat(e.kim_y);
    ls9.kim_theta = halfToFloat(e.kim_theta) * 180.0f / m_PI;
    ls9.ang_i_bias = halfToFloat(e.ang_i_bias);
    ls9.ang_i_bias_val = halfToFloat(e.ang_i_bias_val);
    ls9.left90_d_diff = 0.0f;
    ls9.right90_d_diff = 0.0f;
    // --- ls10 ---
    {
      float l45 = ls3.left45_d, r45 = ls3.right45_d;
      float l45_2 = ls8.left45_2_d, r45_2 = ls8.right45_2_d;
      float l45_3 = ls8.left45_3_d, r45_3 = ls8.right45_3_d;
      auto clamp = [](float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
      };
      ls10.right45_3_d_diff = clamp(r45_3 - right45_3_d_z, -th, th);
      ls10.right45_2_d_diff = clamp(r45_2 - right45_2_d_z, -th, th);
      ls10.right45_d_diff = clamp(r45 - right45_d_z, -th, th);
      ls10.left45_d_diff = clamp(l45 - left45_d_z, -th, th);
      ls10.left45_2_d_diff = clamp(l45_2 - left45_2_d_z, -th, th);
      ls10.left45_3_d_diff = clamp(l45_3 - left45_3_d_z, -th, th);
      left45_d_z = l45;
      right45_d_z = r45;
      left45_2_d_z = l45_2;
      right45_2_d_z = r45_2;
      left45_3_d_z = l45_3;
      right45_3_d_z = r45_3;
    }
    ls10.duty_suction = halfToFloat(e.duty_suction);
    ls10.duty_roll = halfToFloat(e.duty_roll);
    ls10.ang_kf_sum = halfToFloat(e.ang_kf_sum) * 180.0f / m_PI;
    ls10.img_ang_sum = halfToFloat(e.img_ang_sum) * 180.0f / m_PI;
    ls10.duty_roll_before = halfToFloat(e.duty_roll_before);
    ls10.reserve5 = 0;
    // --- ls11 ---
    ls11.pln_t_ego      = e.pln_t_ego;
    ls11.pln_t_sensor   = e.pln_t_sensor;
    ls11.pln_t_trj      = e.pln_t_trj;
    ls11.pln_t_kanayama = e.pln_t_kanayama;
    ls11.pln_t_copy     = e.pln_t_copy;
    ls11.pln_t_ctl      = e.pln_t_ctl;

    size_t off = batch_off;
    memcpy(send_buf + off, &ls1, sizeof(ls1));
    off += sizeof(ls1);
    memcpy(send_buf + off, &ls2, sizeof(ls2));
    off += sizeof(ls2);
    memcpy(send_buf + off, &ls3, sizeof(ls3));
    off += sizeof(ls3);
    memcpy(send_buf + off, &ls4, sizeof(ls4));
    off += sizeof(ls4);
    memcpy(send_buf + off, &ls5, sizeof(ls5));
    off += sizeof(ls5);
    memcpy(send_buf + off, &ls6, sizeof(ls6));
    off += sizeof(ls6);
    memcpy(send_buf + off, &ls7, sizeof(ls7));
    off += sizeof(ls7);
    memcpy(send_buf + off, &ls8, sizeof(ls8));
    off += sizeof(ls8);
    memcpy(send_buf + off, &ls9, sizeof(ls9));
    off += sizeof(ls9);
    memcpy(send_buf + off, &ls10, sizeof(ls10));
    off += sizeof(ls10);
    memcpy(send_buf + off, &ls11, sizeof(ls11));

    batch_off += size;
    batch_cnt++;

    if (batch_cnt >= BATCH) {
      // fwrite(send_buf, 1, batch_off, stdout);
      cdc_write_all(send_buf, batch_off);
      batch_off = 0;
      batch_cnt = 0;
    }

    if (++flush_cnt >= 50) {
      flush_cnt = 0;
      // fflush(stdout);
      // sleep_ms(1);
    }
  }
  // }

  // 終端レコード (index = -1)
  ls1.index = -1;
  {
    size_t off = 0;
    memcpy(send_buf + off, &ls1, sizeof(ls1));
    off += sizeof(ls1);
    memcpy(send_buf + off, &ls2, sizeof(ls2));
    off += sizeof(ls2);
    memcpy(send_buf + off, &ls3, sizeof(ls3));
    off += sizeof(ls3);
    memcpy(send_buf + off, &ls4, sizeof(ls4));
    off += sizeof(ls4);
    memcpy(send_buf + off, &ls5, sizeof(ls5));
    off += sizeof(ls5);
    memcpy(send_buf + off, &ls6, sizeof(ls6));
    off += sizeof(ls6);
    memcpy(send_buf + off, &ls7, sizeof(ls7));
    off += sizeof(ls7);
    memcpy(send_buf + off, &ls8, sizeof(ls8));
    off += sizeof(ls8);
    memcpy(send_buf + off, &ls9, sizeof(ls9));
    off += sizeof(ls9);
    memcpy(send_buf + off, &ls10, sizeof(ls10));
    off += sizeof(ls10);
    memcpy(send_buf + off, &ls11, sizeof(ls11));
    fwrite(send_buf, 1, size, stdout);
    if (batch_off > 0) {
      // fwrite(send_buf, 1, batch_off, stdout);
      cdc_write_all(send_buf, size);
    }
  }
  fflush(stdout);
  sleep_ms(100);

  stdio_set_translate_crlf(&stdio_usb, true);
  printf("[LoggingTask] dump done: %zu entries\n", n);
}

// ============================================================
// USB CDC にテキスト CSV 出力 (rx_term.js テキストプロトコル)
// プロトコル:
//   csv___\n     … テキスト CSV モード開始
//   header\n     … カンマ区切りフィールド名
//   row\n × n   … データ行
//   end___\n     … 終了 → rx_term.js が logs/ に保存
// ============================================================
void LoggingTask::dump_csv_text() const {
  const size_t n = log_vec_.size();

  printf("csv___\n");
  fflush(stdout);
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
         "pln_t_ego,pln_t_sensor,pln_t_trj,pln_t_kanayama,pln_t_copy,pln_t_ctl,"
         "ff_front,ff_roll,ff_rpm_r,ff_rpm_l,"
         "pos_x,pos_y,odm_x,odm_y,odm_theta,kim_x,kim_y,kim_theta,"
         "knym_v,knym_w,ang_kf_sum,img_ang_sum\n");

  int flush_cnt = 0;
  for (size_t i = 0; i < n; ++i) {
    const auto &e = log_vec_[i];
    printf(
        "%zu,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f,"
        "%.4f,%.4f,%.4f,%.2f,%.2f,%.4f,%.4f,%.4f,"
        "%d,%d,%d,%d,"
        "%d,%d,%d,%d,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,"
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
        "%u,%d,"
        "%d,%d,%d,%d,"
        "%d,%d,%d,%d,%d,%d,"
        "%.3f,%.3f,%.3f,%.3f,"
        "%.2f,%.2f,%.2f,%.2f,%.4f,%.2f,%.2f,%.4f,"
        "%.3f,%.3f,%.4f,%.4f\n",
        i, halfToFloat(e.img_v), halfToFloat(e.v_c), halfToFloat(e.v_c2),
        halfToFloat(e.v_l), halfToFloat(e.v_r), (int)e.v_l_enc, (int)e.v_r_enc,
        halfToFloat(e.accl), halfToFloat(e.accl_x), halfToFloat(e.img_w),
        halfToFloat(e.w_lp), halfToFloat(e.alpha), halfToFloat(e.img_dist),
        halfToFloat(e.dist), halfToFloat(e.img_ang), halfToFloat(e.ang),
        halfToFloat(e.ang_kf), (int)e.left90_lp, (int)e.left45_lp,
        (int)e.right45_lp, (int)e.right90_lp, (int)e.left45_2_lp,
        (int)e.right45_2_lp, (int)e.left45_3_lp, (int)e.right45_3_lp,
        halfToFloat(e.battery_lp), halfToFloat(e.duty_l), halfToFloat(e.duty_r),
        halfToFloat(e.duty_sensor_ctrl), halfToFloat(e.duty_suction),
        halfToFloat(e.sen_log_l45), halfToFloat(e.sen_log_r45),
        halfToFloat(e.sen_log_l45_2), halfToFloat(e.sen_log_r45_2),
        halfToFloat(e.sen_log_l45_3), halfToFloat(e.sen_log_r45_3),
        (unsigned)e.motion_type, (int)e.motion_timestamp, (int)e.sen_calc_time,
        (int)e.sen_calc_time2, (int)e.pln_calc_time, (int)e.pln_time_diff,
        (int)e.pln_t_ego, (int)e.pln_t_sensor, (int)e.pln_t_trj,
        (int)e.pln_t_kanayama, (int)e.pln_t_copy, (int)e.pln_t_ctl,
        halfToFloat(e.ff_duty_front), halfToFloat(e.ff_duty_roll),
        halfToFloat(e.ff_duty_rpm_r), halfToFloat(e.ff_duty_rpm_l),
        halfToFloat(e.pos_x), halfToFloat(e.pos_y), halfToFloat(e.odm_x),
        halfToFloat(e.odm_y), halfToFloat(e.odm_theta), halfToFloat(e.kim_x),
        halfToFloat(e.kim_y), halfToFloat(e.kim_theta), halfToFloat(e.knym_v),
        halfToFloat(e.knym_w), halfToFloat(e.ang_kf_sum),
        halfToFloat(e.img_ang_sum));

    if (++flush_cnt >= 50) {
      flush_cnt = 0;
      sleep_ms(1);
    }
  }

  printf("end___\n");
  printf("[LoggingTask] text dump done: %zu entries\n", n);
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
void LoggingTask::set_error_entity(std::shared_ptr<pid_error_entity_t> &_ee) {
  error_entity = _ee;
}
void LoggingTask::cdc_write_all(const uint8_t *p, size_t len) {
  while (len > 0) {
    tud_task(); // USB処理を回す

    if (!tud_cdc_connected()) {
      sleep_ms(1);
      continue;
    }

    uint32_t avail = tud_cdc_write_available();
    if (avail == 0) {
      tud_cdc_write_flush();
      sleep_ms(0);
      continue;
    }

    uint32_t n = (len < avail) ? len : avail;
    uint32_t written = tud_cdc_write(p, n);
    tud_cdc_write_flush();

    p += written;
    len -= written;
  }
}