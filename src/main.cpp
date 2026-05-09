#include "config_loader.hpp"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/interp.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
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

  printf("[boot] step6: multicore_launch_core1\n");
  s_rt_sensing = sensing.get();
  s_rt_planning = planning.get();
  multicore_launch_core1(rt_core1_entry);

  printf("[boot] step7: MainTask start\n");
  MainTask::start();
}
