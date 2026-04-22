#include "config_loader.hpp"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/interp.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "main/main_task.hpp"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "planning/planning_task.hpp"
#include "sensing_task.hpp"
#include <stdio.h>

#include "define.hpp"

#include "blink.pio.h"

int main() {
  stdio_init_all();
  set_sys_clock_khz(150000, true);

  // Tactile switch: pull-up (active low)
  gpio_init(BTN_PIN);
  gpio_set_dir(BTN_PIN, GPIO_IN);
  gpio_pull_up(BTN_PIN);

  // 設定ファイル読み込み (multicore 起動前に実施)
  ConfigLoader::init();

  // SensingTask 初期化 (SPI / ADC / GPIO は Core0 で完結)
  auto sensing = SensingTask::create();
  sensing->init();
  sensing->configure(
      (uint32_t)ConfigLoader::get_int("sensing.led_settle_us", 13),
      (uint32_t)ConfigLoader::get_int("sensing.interval_us", 1000));
  sensing->start_irq();

  // PlanningTask 初期化 (モーター/吸引 PWM + TIMER1 IRQ を Core0 に登録)
  auto planning = PlanningTask::create();
  planning->init(sensing);

  // MainTask 生成 (Core1 で実行: printf / ボタン / planning 指示)
  auto main_task = MainTask::create(sensing, planning);

  // Core1 起動 → MainTask::run() へ
  multicore_launch_core1(MainTask::core1_entry);

  // Core0: TIMER0 IRQ (sensing 1kHz) を登録してから __wfi() ループ
  // planning の TIMER1 IRQ はすでに init() 内で登録済み

  // Core0 メインループ: Core1 が組み立てた文字列を USB CDC に書き出す。
  // TinyUSB はシングルコア前提のため、USB 関数は必ずここ (Core0) で呼ぶ。
  while (true) {
    if (MainTask::s_print_ready && stdio_usb_connected()) {
      __dmb();  // load barrier: s_print_buf の読み出しを s_print_ready より後に確定
      fputs(MainTask::s_print_buf, stdout);
      __dmb();
      MainTask::s_print_ready = false;
    }
    __wfi();
  }
}
