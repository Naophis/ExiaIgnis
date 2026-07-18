#include "sensing_task.hpp"
#include "driver/spi_util.hpp"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "pico/flash.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <stdio.h>

// --- センサー LED ---
#define R90_LED_PIN 16
#define R45_LED_PIN2 19
#define R45_LED_PIN 20
#define L45_LED_PIN 21
#define L45_LED_PIN2 24
#define L90_LED_PIN 25

// --- ADC (フォトトランジスタ) ---
#define R90_SEN_PIN 26 // ADC0
#define R45_SEN_PIN 27 // ADC1
#define L45_SEN_PIN 28 // ADC2
#define L90_SEN_PIN 29 // ADC3

// --- ジャイロ (ASM330LHH) SPI1 ---
#define GYRO_SPI spi1
#define GYRO_MISO_PIN 12 // SPI1 RX
#define GYRO_CS_PIN 13   // SPI1 CSn
#define GYRO_CLK_PIN 14  // SPI1 SCK
#define GYRO_MOSI_PIN 15 // SPI1 TX

// --- 右エンコーダ (AS5147P) SPI1 共有 ---
#define ENC_R_CS_PIN 17 // GPIO17

// --- 左エンコーダ (AS5147P) SPI1 共有 ---
#define ENC_L_CS_PIN 1   // GPIO1

// --- バッテリADC (ADS7042I) SPI1 共有 ---
#define BATTERY_CS_PIN 3 // GPIO3

// static メンバの定義
std::shared_ptr<SensingTask> SensingTask::s_instance;

// ============================================================
// ファクトリ
// ============================================================
std::shared_ptr<SensingTask> SensingTask::create() {
  s_instance = std::shared_ptr<SensingTask>(new SensingTask());
  return s_instance;
}

void SensingTask::configure(uint32_t interval_us) {
  interval_us_ = interval_us;
}

void SensingTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
}
std::shared_ptr<sensing_result_entity_t> SensingTask::get_sensing_entity() {
  return sensing_result;
}

// ============================================================
// 直接ハードウェアアラーム IRQ ハンドラ
// alarm_pool を介さないため dispatch overhead なし (~12 cycle = 0.08us)
// ============================================================

// alarm 1: 1kHz 定周期 + センサー読み取りシーケンス
__attribute__((noinline, section(".time_critical.sensing_irq")))
void SensingTask::timer_b_irq_handler() {
  // ハンドラ入口で即計測 — 以後の処理による可変遅延を period 計測に含めない
  const uint64_t sense_start = time_us_64();
  timer_hw->intr = 1u << 1;

  auto *self = s_instance.get();

  // 次回アラームを絶対時刻で設定 (ドリフトなし)
  self->next_alarm_a_ += self->interval_us_;
  {
    // sense_start を使い回すことで余分な time_us_64() 呼び出しを省く
    const uint32_t now32 = (uint32_t)sense_start;
    if ((int32_t)(now32 - self->next_alarm_a_) > (int32_t)self->interval_us_)
      self->next_alarm_a_ = now32 + self->interval_us_;
  }
  timer_hw->alarm[1] = self->next_alarm_a_;

  const auto &se = self->sensing_result;
  se->calc_time = (int16_t)(sense_start - self->start_time_z);
  self->start_time_z = sense_start;
  self->prev_timestamp_ = sense_start;

  self->read_spi_sensors();
  se->t_spi = (int16_t)(time_us_64() - sense_start);

  const auto &tv = self->tgt_val;
  const bool search_mode = self->pt->get_search_mode();

  // LED点灯条件フラグを決定
  bool r90 = true, l90 = true, r45 = true, l45 = true;
  if (search_mode && tv->motion_type == MotionType::STRAIGHT) {
    r90 = l90 = false;
  }
  if (search_mode && tv->nmr.sct == SensorCtrlType::NONE) {
    r90 = l90 = false;
    r45 = l45 = false;
  }
  if (tv->nmr.sct == SensorCtrlType::Dia) {
    r45 = l45 = true;
  }
  if (tv->nmr.sct == SensorCtrlType::Straight) {
    r90 = l90 = true;
    r45 = l45 = true;
  }
  if (tv->motion_type == MotionType::READY) {
    r90 = l90 = true;
    r45 = l45 = false;
  }
  if (tv->motion_type == MotionType::FRONT_CTRL) {
    r90 = l90 = true;
    r45 = l45 = false;
  }
  if (tv->motion_type == MotionType::SENSING_DUMP) {
    r90 = l90 = true;
    r45 = l45 = true;
  }
  if (tv->motion_type == MotionType::SLALOM) {
    r90 = l90 = r45 = l45 = false;
    if (tv->tt == TurnType::Normal) {
      if (tv->td == TurnDirection::Right) {
        l45 = true;
      } else {
        r45 = true;
      }
    }
  }

  // 1. ambient ADC (LED消灯) — skip_sensing で R90/L90 と R45/L45 を交互に取得
  self->skip_sensing_ = !self->skip_sensing_;
  const bool skip_sensing = self->skip_sensing_;
  if (skip_sensing) {
    if (r90) {
      adc_select_input(0);
      se->led_sen_before.right90.raw = adc_read();
    } else {
      se->led_sen_before.right90.raw = 0;
    }
    if (l90) {
      adc_select_input(3);
      se->led_sen_before.left90.raw = adc_read();
    } else {
      se->led_sen_before.left90.raw = 0;
    }
  } else {
    if (r45) {
      adc_select_input(1);
      se->led_sen_before.right45.raw = adc_read();
      se->led_sen_before.right45_2.raw =
          se->led_sen_before.right45_3.raw = se->led_sen_before.right45.raw;
    } else {
      se->led_sen_before.right45.raw = 0;
    }
    if (l45) {
      adc_select_input(2);
      se->led_sen_before.left45.raw = adc_read();
      se->led_sen_before.left45_2.raw =
          se->led_sen_before.left45_3.raw = se->led_sen_before.left45.raw;
    } else {
      se->led_sen_before.left45.raw = 0;
    }
  }
  se->t_ambient = (int16_t)(time_us_64() - sense_start);

  // 2. LED点灯 + ADC取得 (alarm 2 駆動の非同期ステートマシンに委譲)
  //    busy_wait_us_32()でスピンする代わりに、ここでは最初の
  //    ステップだけ着手してISRをreturnする。続きはled_seq_advance()
  //    (led_seq_irq_handler経由)が担う。led_on==falseの場合は元々waitが
  //    一切発生しない処理だったので、そのまま同期的にfinalizeする。
  bool led_on = true;
  if (tv->motion_type == MotionType::PIVOT) {
    led_on = false;
  }
  if (tv->motion_type == MotionType::SLALOM) {
    led_on = true;
  }

  self->seq_sense_start_ = sense_start;
  self->seq_r90_ = r90;
  self->seq_l90_ = l90;
  self->seq_r45_ = r45;
  self->seq_l45_ = l45;
  self->seq_extended_ = (tv->motion_type == MotionType::WALL_OFF ||
                        tv->motion_type == MotionType::WALL_OFF_DIA ||
                        tv->motion_type == MotionType::SENSING_DUMP ||
                        tv->motion_type == MotionType::SLA_BACK_STR);

  if (!led_on) {
    self->finalize_sensing(false);
    return;
  }
  self->led_seq_start();
}

// ============================================================
// LED点灯シーケンス 非同期ステートマシン (alarm 2, TIMER0_IRQ_2)
// ============================================================

__attribute__((noinline, section(".time_critical.sensing_irq")))
void SensingTask::led_seq_irq_handler() {
  timer_hw->intr = 1u << 2;
  s_instance->led_seq_advance();
}

__attribute__((noinline, section(".time_critical.sensing_irq")))
uint32_t SensingTask::wait_us_single() const {
  return (uint32_t)(param->led_light_delay_cnt * param->led_light_delay_us_per_cnt);
}

__attribute__((noinline, section(".time_critical.sensing_irq")))
uint32_t SensingTask::wait_us_extended() const {
  return (uint32_t)(param->led_light_delay_cnt2 * param->led_light_delay_us_per_cnt);
}

__attribute__((noinline, section(".time_critical.sensing_irq")))
bool SensingTask::try_start_r90() {
  if (!seq_r90_) {
    sensing_result->led_sen_after.right90.raw = 0;
    return false;
  }
  gpio_put(R90_LED_PIN, 1);
  adc_select_input(0);
  led_step_ = LedStep::R90;
  timer_hw->alarm[2] = (uint32_t)time_us_64() + wait_us_single();
  return true;
}

__attribute__((noinline, section(".time_critical.sensing_irq")))
bool SensingTask::try_start_l90() {
  if (!seq_l90_) {
    sensing_result->led_sen_after.left90.raw = 0;
    return false;
  }
  gpio_put(L90_LED_PIN, 1);
  adc_select_input(3);
  led_step_ = LedStep::L90;
  timer_hw->alarm[2] = (uint32_t)time_us_64() + wait_us_single();
  return true;
}

__attribute__((noinline, section(".time_critical.sensing_irq")))
bool SensingTask::try_start_r45() {
  if (!seq_r45_) {
    auto &se = sensing_result;
    se->led_sen_after.right45.raw = se->led_sen_after.right45_2.raw =
        se->led_sen_after.right45_3.raw = 0;
    return false;
  }
  adc_select_input(1);
  gpio_put(R45_LED_PIN, 1); // LED1 ON
  led_step_ = LedStep::R45_1;
  timer_hw->alarm[2] = (uint32_t)time_us_64() + wait_us_single();
  return true;
}

__attribute__((noinline, section(".time_critical.sensing_irq")))
bool SensingTask::try_start_l45() {
  if (!seq_l45_) {
    auto &se = sensing_result;
    se->led_sen_after.left45.raw = se->led_sen_after.left45_2.raw =
        se->led_sen_after.left45_3.raw = 0;
    return false;
  }
  adc_select_input(2);
  gpio_put(L45_LED_PIN, 1); // LED1 ON
  led_step_ = LedStep::L45_1;
  timer_hw->alarm[2] = (uint32_t)time_us_64() + wait_us_single();
  return true;
}

// ambient読み取り後、timer_b_irq_handlerから呼ばれる初回エントリ。
// r90→l90→r45→l45の順で該当する最初のステップを開始する。
// 該当ステップが1つもなければ(全フラグfalse)待たずにfinalizeする。
__attribute__((noinline, section(".time_critical.sensing_irq")))
void SensingTask::led_seq_start() {
  if (try_start_r90()) return;
  sensing_result->t_r90 = (int16_t)(time_us_64() - seq_sense_start_);
  if (try_start_l90()) return;
  sensing_result->t_l90 = (int16_t)(time_us_64() - seq_sense_start_);
  if (try_start_r45()) return;
  sensing_result->t_r45 = (int16_t)(time_us_64() - seq_sense_start_);
  if (try_start_l45()) return;
  sensing_result->t_l45 = (int16_t)(time_us_64() - seq_sense_start_);
  finalize_sensing(true);
}

// alarm 2 発火時 (=led_step_の待ち時間経過後) に呼ばれる。
// 直前ステップのADC読み取り+LED後処理を行い、次のステップへ進む。
__attribute__((noinline, section(".time_critical.sensing_irq")))
void SensingTask::led_seq_advance() {
  const auto &se = sensing_result;
  const uint64_t now = time_us_64();

  switch (led_step_) {
  case LedStep::R90:
    se->led_sen_after.right90.raw = adc_read();
    gpio_put(R90_LED_PIN, 0);
    se->t_r90 = (int16_t)(now - seq_sense_start_);
    if (try_start_l90()) return;
    se->t_l90 = (int16_t)(time_us_64() - seq_sense_start_);
    if (try_start_r45()) return;
    se->t_r45 = (int16_t)(time_us_64() - seq_sense_start_);
    if (try_start_l45()) return;
    se->t_l45 = (int16_t)(time_us_64() - seq_sense_start_);
    finalize_sensing(true);
    return;

  case LedStep::L90:
    se->led_sen_after.left90.raw = adc_read();
    gpio_put(L90_LED_PIN, 0);
    se->t_l90 = (int16_t)(now - seq_sense_start_);
    if (try_start_r45()) return;
    se->t_r45 = (int16_t)(time_us_64() - seq_sense_start_);
    if (try_start_l45()) return;
    se->t_l45 = (int16_t)(time_us_64() - seq_sense_start_);
    finalize_sensing(true);
    return;

  case LedStep::R45_1:
    se->led_sen_after.right45.raw = adc_read(); // LED1 single
    if (seq_extended_) {
      gpio_put(R45_LED_PIN2, 1); // LED2 ON (LED1 still on)
      adc_select_input(1);
      led_step_ = LedStep::R45_2;
      timer_hw->alarm[2] = (uint32_t)now + wait_us_extended();
      return;
    }
    gpio_put(R45_LED_PIN, 0);
    se->led_sen_after.right45_2.raw = se->led_sen_after.right45_3.raw = 0;
    se->t_r45 = (int16_t)(now - seq_sense_start_);
    if (try_start_l45()) return;
    se->t_l45 = (int16_t)(time_us_64() - seq_sense_start_);
    finalize_sensing(true);
    return;

  case LedStep::R45_2:
    se->led_sen_after.right45_2.raw = adc_read(); // LED1 + LED2
    gpio_put(R45_LED_PIN, 0);                     // LED1 OFF
    adc_select_input(1);
    led_step_ = LedStep::R45_3;
    timer_hw->alarm[2] = (uint32_t)now + wait_us_extended();
    return;

  case LedStep::R45_3:
    se->led_sen_after.right45_3.raw = adc_read(); // LED2 single
    gpio_put(R45_LED_PIN2, 0);
    se->t_r45 = (int16_t)(now - seq_sense_start_);
    if (try_start_l45()) return;
    se->t_l45 = (int16_t)(time_us_64() - seq_sense_start_);
    finalize_sensing(true);
    return;

  case LedStep::L45_1:
    se->led_sen_after.left45.raw = adc_read(); // LED1 single
    if (seq_extended_) {
      gpio_put(L45_LED_PIN2, 1); // LED2 ON (LED1 still on)
      adc_select_input(2);
      led_step_ = LedStep::L45_2;
      timer_hw->alarm[2] = (uint32_t)now + wait_us_extended();
      return;
    }
    gpio_put(L45_LED_PIN, 0);
    se->led_sen_after.left45_2.raw = se->led_sen_after.left45_3.raw = 0;
    se->t_l45 = (int16_t)(now - seq_sense_start_);
    finalize_sensing(true);
    return;

  case LedStep::L45_2:
    se->led_sen_after.left45_2.raw = adc_read(); // LED1 + LED2
    gpio_put(L45_LED_PIN, 0);                    // LED1 OFF
    adc_select_input(2);
    led_step_ = LedStep::L45_3;
    timer_hw->alarm[2] = (uint32_t)now + wait_us_extended();
    return;

  case LedStep::L45_3:
    se->led_sen_after.left45_3.raw = adc_read(); // LED2 single
    gpio_put(L45_LED_PIN2, 0);
    se->t_l45 = (int16_t)(now - seq_sense_start_);
    finalize_sensing(true);
    return;
  }
}

// diff計算・battery計算・calc_time2 (旧timer_b_irq_handler末尾)。
// led_on==falseの場合はled_sen_afterに触れず(元の実装通り)led_senのみ0クランプする。
__attribute__((noinline, section(".time_critical.sensing_irq")))
void SensingTask::finalize_sensing(bool led_on) {
  const auto &se = sensing_result;

  // diff 計算 (負になる場合は 0 にクランプ)
  if (led_on) {
    se->led_sen.right90.raw = std::max(
        se->led_sen_after.right90.raw - se->led_sen_before.right90.raw, 0);
    se->led_sen.right45.raw = std::max(
        se->led_sen_after.right45.raw - se->led_sen_before.right45.raw, 0);
    se->led_sen.right45_2.raw = std::max(
        se->led_sen_after.right45_2.raw - se->led_sen_before.right45_2.raw, 0);
    se->led_sen.right45_3.raw = std::max(
        se->led_sen_after.right45_3.raw - se->led_sen_before.right45_3.raw, 0);
    se->led_sen.left45.raw = std::max(
        se->led_sen_after.left45.raw - se->led_sen_before.left45.raw, 0);
    se->led_sen.left45_2.raw = std::max(
        se->led_sen_after.left45_2.raw - se->led_sen_before.left45_2.raw, 0);
    se->led_sen.left45_3.raw = std::max(
        se->led_sen_after.left45_3.raw - se->led_sen_before.left45_3.raw, 0);
    se->led_sen.left90.raw = std::max(
        se->led_sen_after.left90.raw - se->led_sen_before.left90.raw, 0);
    se->led_sen.front.raw =
        (se->led_sen.left90.raw + se->led_sen.right90.raw) / 2;
  } else {
    se->led_sen.right90.raw = se->led_sen.right45.raw =
        se->led_sen.right45_2.raw = se->led_sen.right45_3.raw =
            se->led_sen.left45.raw = se->led_sen.left45_2.raw =
                se->led_sen.left45_3.raw = se->led_sen.left90.raw =
                    se->led_sen.front.raw = 0;
  }

  // se->battery.raw = self->battery_.read();
  se->battery.data = param->battery_gain * 4 * se->battery.raw / 4096;
  se->calc_time2 = (uint32_t)(time_us_64() - seq_sense_start_);
}

// ============================================================
// TIMER0 IRQ 登録 + タイマー起動 (ブロックしない)
// 呼び出したコアで IRQ が動作する。Core0 から呼ぶこと。
// ============================================================
void SensingTask::start_irq() {
  irq_set_exclusive_handler(TIMER0_IRQ_1, timer_b_irq_handler);

  irq_set_priority(TIMER0_IRQ_1, PICO_HIGHEST_IRQ_PRIORITY);

  irq_set_enabled(TIMER0_IRQ_1, true);

  hw_set_bits(&timer_hw->inte, 1u << 1);

  // alarm 2: LED点灯シーケンスの非同期継続 (led_seq_start/advance が使う)
  irq_set_exclusive_handler(TIMER0_IRQ_2, led_seq_irq_handler);
  irq_set_priority(TIMER0_IRQ_2, PICO_HIGHEST_IRQ_PRIORITY);
  irq_set_enabled(TIMER0_IRQ_2, true);
  hw_set_bits(&timer_hw->inte, 1u << 2);

  next_alarm_a_ = (uint32_t)time_us_64() + interval_us_;
  timer_hw->alarm[1] = next_alarm_a_;
}

// ============================================================
// 旧 Core1 エントリ (後方互換のため残す)
// ============================================================
void SensingTask::run() {
  start_irq();
  while (true) {
    __wfi();
  }
}

void SensingTask::core1_entry() {
  flash_safe_execute_core_init();
  s_instance->run();
}

// ============================================================
// 初期化 (コア0の main から呼ぶ)
// ============================================================
void SensingTask::init() {
  // センサー LED
  const uint led_pins[] = {R90_LED_PIN, R45_LED_PIN,  R45_LED_PIN2,
                           L45_LED_PIN, L45_LED_PIN2, L90_LED_PIN};
  for (uint pin : led_pins) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
  }

  // ADC
  adc_init();
  adc_gpio_init(R90_SEN_PIN);
  adc_gpio_init(R45_SEN_PIN);
  adc_gpio_init(L45_SEN_PIN);
  adc_gpio_init(L90_SEN_PIN);

  // ジャイロ (ASM330LHH) — SPI バス初期化込み
  gyro_.init(GYRO_SPI, GYRO_MISO_PIN, GYRO_CS_PIN, GYRO_CLK_PIN, GYRO_MOSI_PIN);
  gyro_.setup();

  // 右エンコーダ (AS5147P) — SPI1 はジャイロ init 済み、CS のみ設定
  enc_r_.init(GYRO_SPI, ENC_R_CS_PIN);
  enc_r_.setup();

  // バッテリADC (ADS7042I) — enc_l_.setup() より前に CS HIGH に設定して bus
  // contention を防ぐ (SPI1 バスはジャイロ init 済み)
  battery_.init(GYRO_SPI, BATTERY_CS_PIN);

  // 左エンコーダ (AS5147P) — CS のみ設定 (SPI1 共有)
  enc_l_.init(GYRO_SPI, ENC_L_CS_PIN);
  enc_l_.setup();

  // SPI1 MISO 疎通確認: bat≠0 なら GPIO12(MISO) は生きている
  uint16_t bat0 = battery_.read();
  uint16_t bat1 = battery_.read();
  printf("[bat] raw0=%u raw1=%u  (SPI1 MISO %s)\n", bat0, bat1,
         (bat0 || bat1) ? "OK" : "STUCK LOW");

  init_dma();
}

void SensingTask::init_dma() {
  dma_tx_spi1_ = dma_claim_unused_channel(true);
  dma_rx_spi1_ = dma_claim_unused_channel(true);
  dma_tx_bat_ = dma_claim_unused_channel(true);
  dma_rx_bat_ = dma_claim_unused_channel(true);

  // SPI1 TX (gyro, 8-bit): バッファ → SPI DR (固定アドレス)
  dma_cfg_tx_spi1_ = dma_channel_get_default_config(dma_tx_spi1_);
  channel_config_set_transfer_data_size(&dma_cfg_tx_spi1_, DMA_SIZE_8);
  channel_config_set_dreq(&dma_cfg_tx_spi1_, spi_get_dreq(spi1, true));
  channel_config_set_read_increment(&dma_cfg_tx_spi1_, true);
  channel_config_set_write_increment(&dma_cfg_tx_spi1_, false);

  // SPI1 RX (gyro, 8-bit): SPI DR (固定アドレス) → バッファ
  dma_cfg_rx_spi1_ = dma_channel_get_default_config(dma_rx_spi1_);
  channel_config_set_transfer_data_size(&dma_cfg_rx_spi1_, DMA_SIZE_8);
  channel_config_set_dreq(&dma_cfg_rx_spi1_, spi_get_dreq(spi1, false));
  channel_config_set_read_increment(&dma_cfg_rx_spi1_, false);
  channel_config_set_write_increment(&dma_cfg_rx_spi1_, true);

  // SPI1 TX (bat, 16-bit): 固定ゼロ値 → SPI DR
  dma_cfg_tx_bat_ = dma_channel_get_default_config(dma_tx_bat_);
  channel_config_set_transfer_data_size(&dma_cfg_tx_bat_, DMA_SIZE_16);
  channel_config_set_dreq(&dma_cfg_tx_bat_, spi_get_dreq(spi1, true));
  channel_config_set_read_increment(&dma_cfg_tx_bat_, false);
  channel_config_set_write_increment(&dma_cfg_tx_bat_, false);

  // SPI1 RX (bat, 16-bit): SPI DR → 1ワードバッファ
  dma_cfg_rx_bat_ = dma_channel_get_default_config(dma_rx_bat_);
  channel_config_set_transfer_data_size(&dma_cfg_rx_bat_, DMA_SIZE_16);
  channel_config_set_dreq(&dma_cfg_rx_bat_, spi_get_dreq(spi1, false));
  channel_config_set_read_increment(&dma_cfg_rx_bat_, false);
  channel_config_set_write_increment(&dma_cfg_rx_bat_, false);
}

__attribute__((noinline, section(".time_critical.sensing_irq"))) void
SensingTask::read_spi_sensors() {
  const auto &se = sensing_result;

  const auto accl_l = (tgt_val->ego_in.v_l - vl_old) / dt;
  const auto accl_r = (tgt_val->ego_in.v_r - vr_old) / dt;

  const float tire = param->tire;
  const float tread = param->tire_tread;

  se->ego.v_l_old = se->ego.v_l;
  se->ego.v_r_old = se->ego.v_r;
  se->encoder.left_old = se->encoder.left;
  se->encoder.right_old = se->encoder.right;
  gyro_timestamp_old = gyro_timestamp_now;
  enc_r_timestamp_old = enc_r_timestamp_now;
  enc_l_timestamp_old = enc_l_timestamp_now;

  const uint64_t spi_t0 = time_us_64();

  // ================================================================
  // Phase A: ジャイロ DMA (SPI1, mode3)
  // ================================================================

  // SPI1 を mode3 (CPOL=1, CPHA=1) に切り替え; 前回は bat が mode0 で終了
  spi_set_format_safe(GYRO_SPI, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  busy_wait_us_32(2);  // SCK settling after mode0→mode3 switch

  gyro_dma_tx_[0] = ASM330_OUTZ_L_G | 0x80;
  gyro_dma_tx_[1] = 0;
  gyro_dma_tx_[2] = 0;

  // ジャイロ CS アサート → TX/RX DMA を同時スタート → 完了待ち
  gyro_timestamp_now = time_us_64();
  gpio_put(GYRO_CS_PIN, 0);
  dma_channel_configure(dma_tx_spi1_, &dma_cfg_tx_spi1_,
                        &spi_get_hw(GYRO_SPI)->dr, gyro_dma_tx_, 3, false);
  dma_channel_configure(dma_rx_spi1_, &dma_cfg_rx_spi1_,
                        gyro_dma_rx_, &spi_get_hw(GYRO_SPI)->dr, 3, false);
  dma_start_channel_mask((1u << dma_tx_spi1_) | (1u << dma_rx_spi1_));
  dma_channel_wait_for_finish_blocking(dma_rx_spi1_);
  gpio_put(GYRO_CS_PIN, 1);
  se->t_gyro = (int16_t)(time_us_64() - spi_t0);
  int16_t gyro = static_cast<int16_t>(
      static_cast<uint16_t>(gyro_dma_rx_[2]) << 8 | gyro_dma_rx_[1]);

  // ================================================================
  // Phase B: 左エンコーダ CPU → 右エンコーダ CPU (SPI1, mode1)
  // ================================================================

  // mode3 → mode1 (CPOL=0, CPHA=1) に切り替え
  spi_set_format_safe(GYRO_SPI, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
  busy_wait_us_32(2);  // SCK settling after mode3→mode1 switch

  enc_l_timestamp_now = time_us_64();
  auto enc_l = enc_l_.read_angle();
  se->t_encl = (int16_t)(time_us_64() - spi_t0);

  // 左エンコーダと同一 mode1 のまま右エンコーダを読む
  enc_r_timestamp_now = time_us_64();
  auto enc_r = enc_r_.read_angle();
  se->t_encr = (int16_t)(time_us_64() - spi_t0);

  // ================================================================
  // Phase C: バッテリ DMA (SPI1, mode0, 16-bit)
  // ================================================================

  // mode1 → mode0 (CPOL=0, CPHA=0, 16-bit) に切り替え
  spi_set_format_safe(GYRO_SPI, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
  bat_dma_tx_ = 0x0000;

  gpio_put(BATTERY_CS_PIN, 0);
  dma_channel_configure(dma_tx_bat_, &dma_cfg_tx_bat_,
                        &spi_get_hw(GYRO_SPI)->dr, &bat_dma_tx_, 1, false);
  dma_channel_configure(dma_rx_bat_, &dma_cfg_rx_bat_,
                        &bat_dma_rx_, &spi_get_hw(GYRO_SPI)->dr, 1, false);
  dma_start_channel_mask((1u << dma_tx_bat_) | (1u << dma_rx_bat_));
  dma_channel_wait_for_finish_blocking(dma_rx_bat_);
  gpio_put(BATTERY_CS_PIN, 1);
  se->battery.raw = (bat_dma_rx_ >> 2) & 0x0FFF;
  se->t_bat = (int16_t)(time_us_64() - spi_t0);

  float gyro_dt = (float)(gyro_timestamp_now - gyro_timestamp_old) / 1000000;
  float enc_r_dt = (float)(enc_r_timestamp_now - enc_r_timestamp_old) / 1000000;
  float enc_l_dt = (float)(enc_l_timestamp_now - enc_l_timestamp_old) / 1000000;

  // if (se->ego.v_l > 100) {
  //   printf("v_l: %.1f v_r: %.1f,vc: %.1f x_dt: %.1f dist: %.1f\n",
  //   se->ego.v_l,
  //          se->ego.v_r, se->ego.v_c, se->ego.v_c * dt, tgt_val->ego_in.dist);
  //   printf("gyro: %d (dt: %.3f s), enc_r: %d (dt: %.3f s), enc_l: %d (dt: % "
  //          ".3fs)\n ",
  //          gyro, gyro_dt, enc_r, enc_r_dt, enc_l, enc_l_dt);
  // }

  pt->ego.kf_w.dt = gyro_dt;
  pt->ego.kf_v_r.dt = enc_r_dt;
  pt->ego.kf_v_l.dt = enc_l_dt;

  auto tmp_r_v =
      ABS(calc_enc_v(enc_r, se->encoder.right_old, pt->ego.kf_v_r.dt));
  auto tmp_l_v =
      ABS(calc_enc_v(enc_l, se->encoder.left_old, pt->ego.kf_v_l.dt));

  if (enc_r_dt > 0) {
    pt->ego.kf_v_r.dt = enc_r_dt;
    pt->ego.kf_v_r.predict(accl_r);
    if (enc_r == 0 && ABS(ABS(tmp_r_v) - ABS(se->ego.v_r)) > 50) {
      // エンコーダ取得失敗時更新中止
      enc_r_timestamp_now = enc_r_timestamp_old;
    } else if (enc_r >= 0) {
      se->encoder.right = enc_r;
      se->ego.v_r =
          -calc_enc_v(se->encoder.right, se->encoder.right_old, enc_r_dt);
      pt->ego.kf_v_r.update(se->ego.v_r);
    }
  }
  if (enc_l == 0 && ABS(ABS(tmp_l_v) - ABS(se->ego.v_l)) > 50) {
    // エンコーダ取得失敗時更新中止
    enc_l_timestamp_now = enc_l_timestamp_old;
  } else if (enc_l_dt > 0) {
    pt->ego.kf_v_l.dt = enc_l_dt;
    pt->ego.kf_v_l.predict(accl_l);
    if (enc_l >= 0) {
      se->encoder.left = enc_l;
      se->ego.v_l =
          calc_enc_v(se->encoder.left, se->encoder.left_old, enc_l_dt);
      pt->ego.kf_v_l.update(se->ego.v_l);
    }
  }
  // printf("enc_l: %d m/s, enc_r: %d \n", enc_l, enc_r);
  if (gyro_dt > 0) {
    se->gyro.raw = se->gyro.data = gyro;
    if ((gyro - tgt_val->gyro_zero_p_offset) >= 0) {
      se->ego.w_raw = param->gyro_param.gyro_w_gain_left *
                      (gyro - tgt_val->gyro_zero_p_offset);
    } else {
      se->ego.w_raw = param->gyro_param.gyro_w_gain_right *
                      (gyro - tgt_val->gyro_zero_p_offset);
    }
    const auto alpha = (tgt_val->ego_in.w - w_old) / dt;
    pt->ego.kf_w.predict(alpha);
    const float tread = param->tire_tread;
    const float w_enc = -(se->ego.v_r - se->ego.v_l) / tread;
    pt->ego.kf_w.update(se->ego.w_raw);
  }
  if (param->enable_kalman_gyro == 1) {
    se->ego.w_raw = se->ego.w_kf = pt->ego.kf_w.get_state();
    // se->ego.w_raw2 = se->ego.w_kf2 = pt->ego.kf_w2.get_state();
  } else if (param->enable_kalman_gyro == 2) {
    se->ego.w_kf = se->ego.w_raw;
    // se->ego.w_kf2 = se->ego.w_raw;
  } else {
    se->ego.w_kf = pt->ego.kf_w.get_state();
    // se->ego.w_kf2 = pt->ego.kf_w2.get_state();
  }

  calc_vel(gyro_dt, enc_l_dt, enc_r_dt);
}

__attribute__((noinline, section(".time_critical.sensing.calc_vel")))
void SensingTask::calc_vel(float gyro_dt, float enc_r_dt, float enc_l_dt) {
  const auto &se = sensing_result;
  const float tire = param->tire;

  // se->ego.v_l = pt->kf_v_l.get_state();
  // se->ego.v_r = pt->kf_v_r.get_state();
  se->ego.v_c = (se->ego.v_l + se->ego.v_r) / 2;

  se->ego.rpm.right = 30.0 * se->ego.v_r / (m_PI * tire / 2);
  se->ego.rpm.left = 30.0 * se->ego.v_l / (m_PI * tire / 2);

  const auto dt = (enc_l_dt + enc_r_dt) / 2;
  tgt_val->ego_in.dist += se->ego.v_c * dt;
  tgt_val->global_pos.dist += se->ego.v_c * dt;

  if (param->enable_kalman_gyro == 1) {
    tgt_val->ego_in.ang += se->ego.w_kf * gyro_dt;
    tgt_val->global_pos.ang += se->ego.w_kf * gyro_dt;
  } else if (param->enable_kalman_gyro == 2) {
    tgt_val->ego_in.ang += se->ego.w_raw * gyro_dt;
    tgt_val->global_pos.ang += se->ego.w_raw * gyro_dt;
  } else {
    tgt_val->ego_in.ang += se->ego.w_raw * gyro_dt;
    tgt_val->global_pos.ang += se->ego.w_raw * gyro_dt;
  }

  w_old = tgt_val->ego_in.w;
  vl_old = se->ego.v_l;
  vr_old = se->ego.v_r;
}
void SensingTask::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param) {
  param = _param;
}
void SensingTask::set_planning_task(std::shared_ptr<PlanningTask> &_pt) {
  pt = _pt;
}

__attribute__((noinline, section(".time_critical.sensing.calc_enc_v")))
float SensingTask::calc_enc_v(float now, float old, float dt) {
  const float tire = param->tire;
  const auto enc_delta = now - old;
  float enc_ang = 0.f;
  if (ABS(enc_delta) <
      MIN(ABS(enc_delta - ENC_RESOLUTION), ABS(enc_delta + ENC_RESOLUTION))) {
    enc_ang = 2 * m_PI * (float)enc_delta / (float)ENC_RESOLUTION;
  } else {
    if (ABS(enc_delta - ENC_RESOLUTION) < ABS(enc_delta + ENC_RESOLUTION)) {
      enc_ang = 2 * m_PI * (float)(enc_delta - ENC_RESOLUTION) /
                (float)ENC_RESOLUTION;
    } else {
      enc_ang = 2 * m_PI * (float)(enc_delta + ENC_RESOLUTION) /
                (float)ENC_RESOLUTION;
    }
  }
  return tire * enc_ang / dt / 2;
}