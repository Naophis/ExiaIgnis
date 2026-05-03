#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/flash.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "sensing_task.hpp"
#include "driver/spi_util.hpp"

// --- センサー LED ---
#define R90_LED_PIN     18
#define R45_LED_PIN2    20
#define R45_LED_PIN     21
#define L45_LED_PIN     23
#define L45_LED_PIN2    24
#define L90_LED_PIN     25

// --- ADC (フォトトランジスタ) ---
#define R90_SEN_PIN     26   // ADC0
#define R45_SEN_PIN     27   // ADC1
#define L45_SEN_PIN     28   // ADC2
#define L90_SEN_PIN     29   // ADC3

// --- ジャイロ (ASM330LHH) SPI1 ---
#define GYRO_SPI        spi1
#define GYRO_MISO_PIN   12   // SPI1 RX
#define GYRO_CS_PIN     13   // SPI1 CSn
#define GYRO_CLK_PIN    14   // SPI1 SCK
#define GYRO_MOSI_PIN   15   // SPI1 TX

// --- 右エンコーダ (AS5147P) SPI1 共有 ---
#define ENC_R_CS_PIN    9    // SPI1 CSn

// --- 左エンコーダ (AS5147P) SPI0 ---
#define ENC_L_SPI       spi0
#define ENC_L_MISO_PIN  0    // SPI0 RX
#define ENC_L_CS_PIN    1    // SPI0 CSn
#define ENC_L_CLK_PIN   2    // SPI0 SCK
#define ENC_L_MOSI_PIN  3    // SPI0 TX

// --- バッテリADC (ADS7042I) SPI0 共有 ---
#define BATTERY_CS_PIN  5    // SPI0 CSn


// static メンバの定義
std::shared_ptr<SensingTask> SensingTask::s_instance;


// ============================================================
// ファクトリ
// ============================================================
std::shared_ptr<SensingTask> SensingTask::create() {
    s_instance = std::shared_ptr<SensingTask>(new SensingTask());
    return s_instance;
}

void SensingTask::configure(uint32_t led_settle_us, uint32_t interval_us) {
    led_settle_us_ = led_settle_us;
    interval_us_   = interval_us;
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

// alarm 1: センサー読み取りシーケンス (one-shot)
void SensingTask::timer_b_irq_handler() {
    timer_hw->intr = 1u << 1;

    auto *self = s_instance.get();
    const auto se = self->get_sensing_entity();
    const uint64_t sense_start = time_us_64();

    // 1. 全LED消灯状態で ambient 読み取り
    adc_select_input(0); se->led_sen_before.left90.raw = adc_read();
    adc_select_input(1); se->led_sen_before.right45.raw = adc_read();
    adc_select_input(2); se->led_sen_before.left45.raw = adc_read();
    adc_select_input(3); se->led_sen_before.left90.raw = adc_read();

    // 2. 各LEDパターンを順次点灯して対応チャンネルを読み取り

    bool led_on = true;

    // R90 (single)
    gpio_put(R90_LED_PIN, 1);
    busy_wait_us_32(self->led_settle_us_);
    adc_select_input(0);
    se->led_sen_after.left90.raw= adc_read();
    gpio_put(R90_LED_PIN, 0);

    // R45 シーケンス: LED1 点灯中に LED2 を追加し、LED1 を消してから LED2 単独を読む
    adc_select_input(1);
    gpio_put(R45_LED_PIN, 1);                          // LED1 ON
    busy_wait_us_32(self->led_settle_us_);
    se->led_sen_after.right45.raw = adc_read();                 // LED1 single
    gpio_put(R45_LED_PIN2, 1);                         // LED2 ON (LED1 still on)
    busy_wait_us_32(self->led_settle_us_);
    se->led_sen_after.right45_2.raw = adc_read();              // LED1 + LED2
    gpio_put(R45_LED_PIN, 0);                          // LED1 OFF (LED2 still on)
    busy_wait_us_32(self->led_settle_us_);
    se->led_sen_after.right45_3.raw = adc_read();                 // LED2 single
    gpio_put(R45_LED_PIN2, 0);                         // LED2 OFF

    // L45 シーケンス: 同様
    adc_select_input(2);
    gpio_put(L45_LED_PIN, 1);                          // LED1 ON
    busy_wait_us_32(self->led_settle_us_);
    se->led_sen_after.left45.raw = adc_read();                 // LED1 single
    gpio_put(L45_LED_PIN2, 1);                         // LED2 ON (LED1 still on)
    busy_wait_us_32(self->led_settle_us_);
    se->led_sen_after.left45_2.raw = adc_read();              // LED1 + LED2
    gpio_put(L45_LED_PIN, 0);                          // LED1 OFF (LED2 still on)
    busy_wait_us_32(self->led_settle_us_);
    se->led_sen_after.left45_3.raw = adc_read();                 // LED2 single
    gpio_put(L45_LED_PIN2, 0);                         // LED2 OFF

    // L90 (single)
    gpio_put(L90_LED_PIN, 1);
    busy_wait_us_32(self->led_settle_us_);
    adc_select_input(3);
    se->led_sen_after.left90.raw = adc_read();
    gpio_put(L90_LED_PIN, 0);

    // 3. diff 計算 (負になる場合は 0 にクランプ)
    // auto dc = [](uint16_t l, uint16_t d) -> uint16_t {
    //     return l > d ? static_cast<uint16_t>(l - d) : 0u;
    // };
    if (led_on) {
      se->led_sen.right90.raw = std::max(
          se->led_sen_after.right90.raw - se->led_sen_before.right90.raw, 0);
      se->led_sen.right45.raw = std::max(
          se->led_sen_after.right45.raw - se->led_sen_before.right45.raw, 0);
      se->led_sen.right45_2.raw = std::max(se->led_sen_after.right45_2.raw -
                                               se->led_sen_before.right45_2.raw,
                                           0);
      se->led_sen.right45_3.raw = std::max(se->led_sen_after.right45_3.raw -
                                               se->led_sen_before.right45_3.raw,
                                           0);
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
    // 4. ジャイロ・エンコーダ・バッテリ (取得直前の時刻を記録)
    self->data.gz_ts_z  = self->data.gz_ts;
    self->data.gz_ts    = time_us_64();
    self->data.gz       = self->gyro_.read_gyro_z();
    self->data.gz_dt    = self->data.gz_ts_z ? (self->data.gz_ts - self->data.gz_ts_z) : 0;

    self->data.enc_r_ts_z = self->data.enc_r_ts;
    self->data.enc_r_ts   = time_us_64();
    self->data.enc_r      = self->enc_r_.read_angle();
    self->data.enc_r_dt   = self->data.enc_r_ts_z ? (self->data.enc_r_ts - self->data.enc_r_ts_z) : 0;
    // enc_r は mode3(CPOL=1) でジャイロと同一設定のため SCK 状態変化なし。restore 不要。

    self->data.enc_l_ts_z = self->data.enc_l_ts;
    self->data.enc_l_ts   = time_us_64();
    self->data.enc_l      = self->enc_l_.read_angle();
    self->data.enc_l_dt   = self->data.enc_l_ts_z ? (self->data.enc_l_ts - self->data.enc_l_ts_z) : 0;

    se->battery.raw  = self->battery_.read();
    // se->battery.data =
    //         param->battery_gain * 4 * sensing_result->battery.raw / 4096;
    self->data.sense_duration_us = (uint32_t)(time_us_64() - sense_start);
    self->data_ready = true;
}

// alarm 2: 1000us 周期 (1kHz)
void SensingTask::timer_a_irq_handler() {
    timer_hw->intr = 1u << 2;

    auto *self = s_instance.get();

    // 次回アラームをすぐに設定 (絶対時刻 → ドリフトなし)
    self->next_alarm_a_ += self->interval_us_;
    timer_hw->alarm[2] = self->next_alarm_a_;

    // アラーム鳴動直後の時刻を記録
    uint64_t now = time_us_64();
    self->data.dt_us = self->prev_timestamp_ ? (uint32_t)(now - self->prev_timestamp_) : 0;
    self->prev_timestamp_ = now;

    // alarm 1 でセンサー読み取りシーケンスをスケジュール
    timer_hw->alarm[1] = (uint32_t)now + 1;
}


// ============================================================
// TIMER0 IRQ 登録 + タイマー起動 (ブロックしない)
// 呼び出したコアで IRQ が動作する。Core0 から呼ぶこと。
// ============================================================
void SensingTask::start_irq() {
    irq_set_exclusive_handler(TIMER0_IRQ_1, timer_b_irq_handler);
    irq_set_exclusive_handler(TIMER0_IRQ_2, timer_a_irq_handler);

    irq_set_priority(TIMER0_IRQ_1, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_priority(TIMER0_IRQ_2, PICO_HIGHEST_IRQ_PRIORITY);

    irq_set_enabled(TIMER0_IRQ_1, true);
    irq_set_enabled(TIMER0_IRQ_2, true);

    hw_set_bits(&timer_hw->inte, (1u << 1) | (1u << 2));

    next_alarm_a_ = (uint32_t)time_us_64() + interval_us_;
    timer_hw->alarm[2] = next_alarm_a_;
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
    const uint led_pins[] = { R90_LED_PIN, R45_LED_PIN, R45_LED_PIN2, L45_LED_PIN, L45_LED_PIN2, L90_LED_PIN };
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

    // SPI0 バス初期化 (左エンコーダ + バッテリADC 共有)
    spi_init(ENC_L_SPI, 1000000);
    gpio_set_function(ENC_L_CLK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(ENC_L_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ENC_L_MISO_PIN, GPIO_FUNC_SPI);

    // バッテリADC (ADS7042I) — enc_l_.setup() より前に CS HIGH に設定して bus contention を防ぐ
    battery_.init(ENC_L_SPI, BATTERY_CS_PIN);

    // 左エンコーダ (AS5147P) — CS のみ設定
    enc_l_.init(ENC_L_SPI, ENC_L_CS_PIN);
    enc_l_.setup();

    // SPI0 MISO 疎通確認: bat≠0 なら GPIO0(MISO) は生きている
    uint16_t bat0 = battery_.read();
    uint16_t bat1 = battery_.read();
    printf("[bat] raw0=%u raw1=%u  (SPI0 MISO %s)\n",
           bat0, bat1,
           (bat0 || bat1) ? "OK" : "STUCK LOW");
}
