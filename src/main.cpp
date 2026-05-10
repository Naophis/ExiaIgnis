#include "config_loader.hpp"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/interp.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "hardware/structs/qmi.h"
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
// PSRAM 初期化 (QMI M1 / APS6404L 想定)
// multicore_launch_core1() より前、かつ LoggingTask::init() より前に呼ぶ。
// ============================================================
static void psram_init(uint cs_pin) {
  // GPIO を XIP CS1 (QMI M1 チップセレクト) として設定
  gpio_set_function(cs_pin, GPIO_FUNC_XIP_CS1);

  // TIMING: CLKDIV=2 (75 MHz), RXDELAY=2 sys-clocks, COOLDOWN=1
  qmi_hw->m[1].timing = (1u << 30) | (2u << 8) | (2u << 0); // 0x40000202

  // APS6404L Quad I/O Read (0xEB):
  //   prefix=SPI(0xEB), addr=Quad, suffix(mode)=Quad(0xA0), dummy=4clk Quad, data=Quad
  // Bit layout (from RP2350 TRM):
  //   [1:0]=PREFIX_WIDTH, [3:2]=ADDR_WIDTH, [5:4]=SUFFIX_WIDTH, [7:6]=DUMMY_WIDTH,
  //   [9:8]=DATA_WIDTH, [12]=PREFIX_LEN(1=8bit), [15:14]=SUFFIX_LEN(2=8bit),
  //   [18:16]=DUMMY_LEN(1=4clk, 2=8clk)
  qmi_hw->m[1].rfmt =
      (0u << 0) |   // PREFIX_WIDTH = S
      (2u << 2) |   // ADDR_WIDTH   = Q
      (2u << 4) |   // SUFFIX_WIDTH = Q (mode byte 0xA0)
      (2u << 6) |   // DUMMY_WIDTH  = Q
      (2u << 8) |   // DATA_WIDTH   = Q
      (1u << 12) |  // PREFIX_LEN   = 8-bit (0xEB command)
      (2u << 14) |  // SUFFIX_LEN   = 8-bit (0xA0 mode byte) ← was missing, caused garbage reads
      (1u << 16);   // DUMMY_LEN    = 4 clocks (VALUE_4=1, OK at 75 MHz)
  qmi_hw->m[1].rcmd = (0xA0u << 8) | 0xEBu; // suffix=0xA0, prefix=0xEB

  // APS6404L Quad I/O Write (0x38):
  //   prefix=SPI, addr=Quad, data=Quad, no suffix/dummy
  qmi_hw->m[1].wfmt =
      (0u << 0) |  // PREFIX_WIDTH = S
      (2u << 2) |  // ADDR_WIDTH   = Q
      (2u << 8) |  // DATA_WIDTH   = Q
      (1u << 12);  // PREFIX_LEN   = 8-bit
  qmi_hw->m[1].wcmd = 0x38u; // prefix=0x38

  sleep_us(200); // PSRAM power-on / mode-change settle time
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

  // 設定ファイル読み込み (multicore 起動前に実施)
  ConfigLoader::init();
  sleep_ms(500);

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

  printf("[boot] step6: PSRAM init (GPIO%u → XIP_CS1)\n", PSRAM_CS_PIN);
  psram_init(PSRAM_CS_PIN);

  printf("[boot] step7: multicore_launch_core1\n");
  s_rt_sensing = sensing.get();
  s_rt_planning = planning.get();
  multicore_launch_core1(rt_core1_entry);

  printf("[boot] step8: MainTask start\n");
  MainTask::start();
}
