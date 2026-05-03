#include "config_loader.hpp"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/interp.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "main/main_task.hpp"
#include "pico/flash.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "planning/planning_task.hpp"
#include "sensing_task.hpp"
#include <stdio.h>

#include "define.hpp"
#include "blink.pio.h"

// ============================================================
// Core1 RT エントリ: sensing + planning IRQ を Core1 に登録
// ============================================================
static SensingTask*  s_rt_sensing  = nullptr;
static PlanningTask* s_rt_planning = nullptr;

std::shared_ptr<sensing_result_entity_t> sensing_entity =
    std::make_shared<sensing_result_entity_t>();

static void rt_core1_entry() {
    flash_safe_execute_core_init();
    s_rt_sensing->start_irq();   // TIMER0 IRQ → Core1
    s_rt_planning->start_irq();  // TIMER1 IRQ → Core1
    while (true) __wfi();
}

// ============================================================
// Core0 main: 初期化 → Core1 起動 → MainTask (printf/UI) を実行
// ============================================================
int main() {
    stdio_init_all();
    set_sys_clock_khz(150000, true);

    // Tactile switch: pull-up (active low)
    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);

    // 設定ファイル読み込み (multicore 起動前に実施)
    ConfigLoader::init();

    // SensingTask 初期化 (SPI / ADC / GPIO ハードウェア設定のみ; IRQ は Core1 で登録)
    auto sensing = SensingTask::create();
    sensing->set_sensing_entity(sensing_entity);
    sensing->init();
    sensing->configure(
        (uint32_t)ConfigLoader::get_int("sensing.led_settle_us", 12),
        (uint32_t)ConfigLoader::get_int("sensing.interval_us", 1000));

    // PlanningTask 初期化 (モーター/吸引 PWM ハードウェア設定のみ; IRQ は Core1 で登録)
    auto planning = PlanningTask::create();
    planning->set_sensing_entity(sensing_entity);
    planning->init(sensing);

    // MainTask 生成 (Core0 で実行: printf / ボタン / planning 指示)
    MainTask::create(sensing, planning);

    // Core1 起動: sensing/planning IRQ を Core1 に登録して __wfi() ループへ
    s_rt_sensing  = sensing.get();
    s_rt_planning = planning.get();
    multicore_launch_core1(rt_core1_entry);

    // Core0: MainTask を直接実行 (TinyUSB が Core0 固定のため printf を直接呼べる)
    MainTask::start();
}
