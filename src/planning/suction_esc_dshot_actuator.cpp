#include "planning/suction_esc_dshot_actuator.hpp"
#include "define.hpp"
#include "driver/dshot_gcr.hpp"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include <cstdio>

namespace {

// ---- ESCape32 の入力変換に合わせた定数(出典: ESCape32 src/io.c) ----------
// setthrot(): throt_mode=0(前進のみ)のとき
//     x <= throt_min+50 -> throt = 0
//     それ以上          -> throt = scale(x, throt_min+50, throt_max, 0, 2000)
// DShot側: iotim_dma_isr() が throt = value - 47 (value=48〜2047 -> 1〜2000)、
//          value=0(DSHOT_CMD_MOTOR_STOP) -> throt = 0
// よって「サーボPWMで出していたパルス幅」と同じ実スロットルを作るには、
// 上の式で throt を求めてから value = throt + 47 に戻せばよい。
// これにより system.yaml の suction_duty 等(us単位、AM32/サーボPWM時代に
// 実機でチューニングした値)がそのまま同じ吸引力を生む。
constexpr float kEsc32ThrotDeadbandUs = 50.0f;  // throt_min + 50
constexpr float kEsc32ThrotMax        = 2000.0f;
constexpr int   kEsc32DshotThrotBias  = 47;     // value = throt + 47

// DShot特殊コマンド番号(ESCape32 src/io.c のcase文と対応)。
constexpr uint16_t kCmdSpinDirection1  = 7;   // cfg.revdir = 0
constexpr uint16_t kCmdSpinDirection2  = 8;   // cfg.revdir = 1
constexpr uint16_t kCmdSaveSettings    = 12;  // savecfg()

// ESC通電から「DShotのレート/極性を判別しarm(250msゼロスロットル)を終える」
// までの待ち時間。write_am32_param()が設定通信前に置いている1500msと同値。
constexpr uint32_t kEscBootMs      = 1500;
// 同一コマンドの保持時間。ESCape32は6フレーム連続で1回だけ適用する
// (2.5kHzなら6フレーム=2.4ms)。取りこぼしを考えても十分な余裕を取る。
constexpr uint32_t kCmdHoldMs      = 60;
// SAVE_SETTINGS はESC側でフラッシュ書き込み+ビープを伴うため長めに待つ。
constexpr uint32_t kSaveSettleMs   = 400;

uint16_t esc32_value_from_pulse_us(float pulse_us) {
  const float min_us = (float)SUCTION_ESC_PULSE_MIN_US;
  const float max_us = (float)SUCTION_ESC_PULSE_MAX_US;
  if (pulse_us < min_us) pulse_us = min_us;
  if (pulse_us > max_us) pulse_us = max_us;

  const float lo = min_us + kEsc32ThrotDeadbandUs;
  if (pulse_us <= lo) return dshot::kCmdMotorStop;  // 停止

  float throt = (pulse_us - lo) * kEsc32ThrotMax / (max_us - lo);
  if (throt < 1.0f) throt = 1.0f;
  if (throt > kEsc32ThrotMax) throt = kEsc32ThrotMax;

  int value = (int)(throt + 0.5f) + kEsc32DshotThrotBias;
  if (value < (int)dshot::kThrottleMin) value = (int)dshot::kThrottleMin;
  if (value > (int)dshot::kThrottleMax) value = (int)dshot::kThrottleMax;
  return (uint16_t)value;
}

} // namespace

void SuctionEscDshotActuator::init() {
  gpio_init(SUCTION_POWER_EN);
  gpio_set_dir(SUCTION_POWER_EN, GPIO_OUT);
  gpio_put(SUCTION_POWER_EN, false); // 初期状態はOFF(enable()まで通電しない)

  // init()は main() 冒頭(ESCへ早く信号を出し始めるため)と
  // PlanningTask::init() の2回呼ばれる。PWM版はレジスタ再設定だけで副作用が
  // 無かったが、DShot版で作り直すとPIO/DMAを一度解放する間だけ信号線が
  // 浮くため、2回目以降は何もしない。
  if (dshot_.is_initialized()) return;

  if (!dshot_.init(SUCTION_ESC_DSHOT_PIO, SUCTION_ESC_PWM,
                   SUCTION_ESC_DSHOT_BITRATE_HZ, SUCTION_ESC_DSHOT_FRAME_HZ)) {
    // PIO/DMA資源不足。ここで失敗すると吸引が一切効かない(黙って0%相当に
    // なるだけ)ので、原因が追えるようログを残す。
    printf("[suction_esc] ERROR: DShot init failed -- suction will not run\n");
    return;
  }
  apply_us((float)SUCTION_ESC_PULSE_MIN_US); // 停止コマンドを即座に送り始める
}

void SuctionEscDshotActuator::apply(float duty_pct) {
  if (duty_pct < 0.0f) duty_pct = 0.0f;
  if (duty_pct > 100.0f) duty_pct = 100.0f;
  const float pulse_us =
      (float)SUCTION_ESC_PULSE_MIN_US +
      ((float)SUCTION_ESC_PULSE_MAX_US - (float)SUCTION_ESC_PULSE_MIN_US) *
          duty_pct / 100.0f;
  apply_us(pulse_us);
}

__attribute__((noinline, section(".time_critical.suction_esc")))
void SuctionEscDshotActuator::apply_us(float pulse_us) {
  // 特殊コマンド送出中は無視する(cmd_mode_のコメント参照)。
  if (cmd_mode_) return;
  send_throttle_value(esc32_value_from_pulse_us(pulse_us));
}

__attribute__((noinline, section(".time_critical.suction_esc")))
void SuctionEscDshotActuator::send_throttle_value(uint16_t value) {
  // スロットルフレームではtelemetry要求bitを立てない。非反転DShotでは
  // ESC側が別ピンのシリアルテレメトリを毎フレーム送ろうとしてしまうため
  // (ESCape32 src/io.c: tlm -> telreq = 1)、要求はコマンド送出時のみにする。
  dshot_.set_frame(
      dshot::build_command_frame(value, /*telemetry_bit=*/false,
                                 /*bidirectional=*/false));
}

// enable/disableはどちらもフラグ + 電源ゲート管理のみ。実際のスロットルは
// ControlLaw::set_next_duty()が毎tick suction_ramp_us_per_sec_(またはLUT)で
// 滑らかに追従させる。ここで即座に停止コマンドへ叩き落とすと、まだ高速
// 回転しているモーターに急ブレーキ相当の指令を送ることになるため行わない
// (PWM版と同じ設計。suction_esc_actuator.hppのコメント参照)。
void SuctionEscDshotActuator::enable() {
  enabled_ = true;
  power_on();
}

void SuctionEscDshotActuator::disable() {
  enabled_ = false;
  power_off();
}

void SuctionEscDshotActuator::power_on()  { gpio_put(SUCTION_POWER_EN, true); }
void SuctionEscDshotActuator::power_off() { gpio_put(SUCTION_POWER_EN, false); }

void SuctionEscDshotActuator::reattach_pin() { dshot_.attach_pin(); }

// ------------------------------------------------------------------
// 回転方向設定
// ------------------------------------------------------------------
void SuctionEscDshotActuator::send_command(uint16_t cmd, uint32_t hold_ms) {
  // コマンドフレームはtelemetry要求bitを立てる必要がある
  // (ESCape32: `if (!tlm || ertm) return;`)。
  dshot_.set_frame(
      dshot::build_command_frame(cmd, /*telemetry_bit=*/true,
                                 /*bidirectional=*/false));
  sleep_ms(hold_ms);
  // 停止コマンドへ戻す。値が変わることでESC側の連続カウンタがリセットされ、
  // 次のコマンドがまた6フレーム目で適用される。
  send_throttle_value(dshot::kCmdMotorStop);
  sleep_ms(kCmdHoldMs);
}

bool SuctionEscDshotActuator::set_spin_direction(bool reversed,
                                                 bool save_to_esc) {
  if (!dshot_.is_initialized()) {
    printf("[suction_esc] set_spin_direction: DShot not initialized\n");
    return false;
  }

  // Core1のControlLawが毎tick apply_us()でフレームを書き換えるのを止める。
  // 呼び出し側は事前に suction_disable() 済み(=ランプ目標が最小パルス)で
  // あること(この関数を抜けた次のtickでその値が反映される)。
  cmd_mode_ = true;
  __dmb();
  send_throttle_value(dshot::kCmdMotorStop);
  power_on();
  printf("[suction_esc] spin direction: waiting %ums for ESC boot/DShot lock\n",
         (unsigned)kEscBootMs);
  sleep_ms(kEscBootMs);

  const uint16_t cmd = reversed ? kCmdSpinDirection2 : kCmdSpinDirection1;
  printf("[suction_esc] send DSHOT_CMD_SPIN_DIRECTION_%d (cmd=%u, revdir=%d)\n",
         reversed ? 2 : 1, (unsigned)cmd, reversed ? 1 : 0);
  send_command(cmd, kCmdHoldMs);

  if (save_to_esc) {
    printf("[suction_esc] send DSHOT_CMD_SAVE_SETTINGS (cmd=%u)\n",
           (unsigned)kCmdSaveSettings);
    send_command(kCmdSaveSettings, kCmdHoldMs);
    sleep_ms(kSaveSettleMs); // ESC側のフラッシュ書き込み + ビープ完了待ち
  }

  send_throttle_value(dshot::kCmdMotorStop);
  power_off();
  __dmb();
  cmd_mode_ = false;  // 以降はControlLawのランプ出力へ戻る
  return true;
}
