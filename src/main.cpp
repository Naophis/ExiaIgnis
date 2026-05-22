#include "config_loader.hpp"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/interp.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/xip.h"
#include "hardware/uart.h"
#include "logging/logging_task.hpp"
#include "main/main_task.hpp"
#include "pico/flash.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "planning/planning_task.hpp"
#include "sensing_task.hpp"
#include <stdio.h>

#include "blink.pio.h"
#include "define.hpp"

// ============================================================
// PSRAM 初期化 (QMI M1 / APS6404L 外付け)
// GPIO0 を XIP_CS1n (QMI M1 CS1) として設定し、QMI M1 タイミング/フォーマットを構成。
// multicore_launch_core1() より前に呼ぶ。
// LittleFS 操作後に wfmt/wcmd がリセットされるため、LoggingTask::init() でも再設定する。
// ============================================================
static void psram_init() {
  // GPIO0 → XIP_CS1n (function 9): QMI M1 の CS1 をハードウェア制御に設定。
  // XIP_CS1n 対応 GPIO: 0, 8, 19, 47 のみ。
  gpio_set_function(PSRAM_CS_PIN, GPIO_FUNC_XIP_CS1);
  // 75 MHz QSPI 向けに高ドライブ強度・高速スルーレートを設定
  gpio_set_drive_strength(PSRAM_CS_PIN, GPIO_DRIVE_STRENGTH_8MA);
  gpio_set_slew_rate(PSRAM_CS_PIN, GPIO_SLEW_RATE_FAST);

  // TIMING: CLKDIV=2 (75 MHz), RXDELAY=2, COOLDOWN=1
  qmi_hw->m[1].timing = (1u << 30) | (2u << 8) | (2u << 0);

  // APS6404L Quad I/O Read (0xEB): SPI command, Quad addr/mode/dummy/data
  qmi_hw->m[1].rfmt =
      (0u << 0)  |  // PREFIX_WIDTH = S
      (2u << 2)  |  // ADDR_WIDTH   = Q
      (2u << 4)  |  // SUFFIX_WIDTH = Q (mode byte 0xA0)
      (2u << 6)  |  // DUMMY_WIDTH  = Q
      (2u << 8)  |  // DATA_WIDTH   = Q
      (1u << 12) |  // PREFIX_LEN   = 8-bit (0xEB)
      (2u << 14) |  // SUFFIX_LEN   = 8-bit (0xA0)
      (1u << 16);   // DUMMY_LEN    = 4 clocks
  qmi_hw->m[1].rcmd = (0xA0u << 8) | 0xEBu;

  // SPI Write (0x02): flash_exit_xip() が PSRAM を serial state に戻すため
  // 0x38 Quad Write は使用不可。SDK リセット値と同じ 0x02 SPI Write を使用する。
  qmi_hw->m[1].wfmt = 0x00001000u;  // PREFIX_WIDTH=S, ADDR_WIDTH=S, DATA_WIDTH=S, PREFIX_LEN=8-bit
  qmi_hw->m[1].wcmd = 0x00000002u;  // SPI Write command 0x02

  // XIP M1 への書き込みを有効化 (デフォルトは read-only)
  hw_set_bits(&xip_ctrl_hw->ctrl, XIP_CTRL_WRITABLE_M1_BITS);

  sleep_us(200); // APS6404L tPU: power-on から 150μs 以上待機
}

// ============================================================
// Core1 RT エントリ: sensing + planning IRQ を Core1 に登録
// ============================================================
static SensingTask *s_rt_sensing = nullptr;
static PlanningTask *s_rt_planning = nullptr;

std::shared_ptr<sensing_result_entity_t> sensing_entity;
std::shared_ptr<input_param_t> param;
std::shared_ptr<motion_tgt_val_t> tgt_val;

static void rt_core1_entry() {
  flash_safe_execute_core_init();
  s_rt_sensing->start_irq();  // TIMER0 IRQ → Core1
  s_rt_planning->start_irq(); // TIMER1 IRQ → Core1
  while (true)
    __wfi();
}

// ============================================================
// Core0 main: 初期化 → Core1 起動 → MainTask (printf/UI) を実行
// ============================================================
int main() {
  sensing_entity = std::make_shared<sensing_result_entity_t>();
  param = std::make_shared<input_param_t>();
  tgt_val = std::make_shared<motion_tgt_val_t>();

  stdio_init_all();
  set_sys_clock_khz(150000, true);

  // Tactile switch: pull-up (active low)
  gpio_init(BTN_PIN);
  gpio_set_dir(BTN_PIN, GPIO_IN);
  gpio_pull_up(BTN_PIN);

  sleep_ms(1500);
  // 設定ファイル読み込み (multicore 起動前に実施)
  ConfigLoader::init();

  printf("[boot] step1: SensingTask create\n");
  auto sensing = SensingTask::create();
  auto planning = PlanningTask::create();
  printf("[boot] step2: sensing set_*\n");

  sensing->set_sensing_entity(sensing_entity);
  sensing->set_planning_task(planning);
  sensing->set_input_param_entity(param);
  sensing->set_tgt_val(tgt_val);
  printf("[boot] step3: sensing init\n");
  sensing->init();
  sensing->configure(
      (uint32_t)ConfigLoader::get_int("sensing.led_settle_us", 18),
      (uint32_t)ConfigLoader::get_int("sensing.interval_us", 1000));

  printf("[boot] step4: planning init\n");
  planning->set_sensing_entity(sensing_entity);
  planning->set_input_param_entity(param);
  planning->set_tgt_val(tgt_val);
  planning->init(sensing);

  printf("[boot] step5: LoggingTask + MainTask create\n");
  auto lt = LoggingTask::create();
  auto main_task = MainTask::create(sensing, planning, param);
  main_task->set_logging_task(lt);
  main_task->set_tgt_val(tgt_val);

  printf("[boot] step6: PSRAM init (RP2354A internal)\n");
  psram_init();

  printf("[boot] step7: multicore_launch_core1\n");
  s_rt_sensing = sensing.get();
  s_rt_planning = planning.get();
  multicore_launch_core1(rt_core1_entry);

  printf("[boot] step8: MainTask start\n");
  MainTask::start();
}
