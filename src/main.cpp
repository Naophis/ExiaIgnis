#include "config_loader.hpp"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/interp.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
extern "C" {
#include "sfe_psram.h"
}
#include "driver/psram_check.hpp"
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

// Core1 用 8KB スタック (正規 SRAM 上)
// SCRATCH_X は 4KB 固定のため multicore_launch_core1_with_stack() 経由で確保。
static uint32_t g_core1_stack[0x2000 / sizeof(uint32_t)];

// ============================================================
// Core0 main: 初期化 → Core1 起動 → MainTask (printf/UI) を実行
// ============================================================
int main() {
  sensing_entity = std::make_shared<sensing_result_entity_t>();
  param = std::make_shared<input_param_t>();
  tgt_val = std::make_shared<motion_tgt_val_t>();

  stdio_init_all();
  set_sys_clock_khz(150000, true);

  // PIO/DMA(吸引ESCのDShot)など他の初期化が一切走る前の素の状態で1回。
  // この時点ではUSB未接続でprintfが届かないので、結果はstep6でまとめて出す。
  // 所要時間は数十us(5MHz×8byte)で、ESCへの信号開始は実質遅れない。
  psram_check::ProbeResult probe_early{};
  psram_check::probe(PSRAM_CS_PIN, &probe_early);

  // 吸引ESCは電源投入直後から有効なスロットル信号(停止指令)が来ている
  // ことを期待する。この後に続くConfigLoader::init()やsleep_ms(1500)等で
  // GPIOが未設定(フローティング)のまま数秒経過すると、ESCが信号ロストと
  // 判断してエラーブザーを鳴らし続けてしまう。他の初期化より先に、
  // PlanningTaskを生成してesc_.init()だけ済ませておくことで、電源投入から
  // できるだけ早く停止指令の出力を開始する。
  // DShot経路ではPIO+DMAが、サーボPWM経路ではハードウェアPWMスライスが、
  // どちらもCPU非関与でこの出力を維持し続ける(Core1起動前・planning IRQが
  // 止まっている間も途切れない)。
  // (PlanningTask::init()側でも再度esc_.init()が呼ばれるが、2回目以降は
  // 何もしない/レジスタ再設定のみで副作用はない)。
  auto planning = PlanningTask::create();
  planning->esc_.init();

  // Tactile switch: pull-up (active low)
  gpio_init(BTN_PIN);
  gpio_set_dir(BTN_PIN, GPIO_IN);
  gpio_pull_up(BTN_PIN);

  sleep_ms(1500);

  // // M_PWM_L1, M_PWM_L2, M_PWM_R1, M_PWM_R2
  // gpio_init(M_PWM_L1);
  // gpio_set_dir(M_PWM_L1, GPIO_OUT);
  // gpio_init(M_PWM_L2);
  // gpio_set_dir(M_PWM_L2, GPIO_OUT);
  // gpio_init(M_PWM_R1);
  // gpio_set_dir(M_PWM_R1, GPIO_OUT);
  // gpio_init(M_PWM_R2);
  // gpio_set_dir(M_PWM_R2, GPIO_OUT);

  // while (1) {
  //   printf("SW1 pressed: wait release\n");
  //   sleep_ms(100);
  //   if (gpio_get(BTN_PIN) == 0) {
  //     printf("SW1 pressed: motor active\n");
  //     gpio_put(M_PWM_L1, 1);
  //     gpio_put(M_PWM_L2, 0);
  //     gpio_put(M_PWM_R1, 1);
  //     gpio_put(M_PWM_R2, 0);
  //   } else {
  //     printf("SW1 released: motor inactive\n");
  //     gpio_put(M_PWM_L1, 0);
  //     gpio_put(M_PWM_L2, 0);
  //     gpio_put(M_PWM_R1, 0);
  //     gpio_put(M_PWM_R2, 0);
  //   }
  // }

  // 設定ファイル読み込み (multicore 起動前に実施)
  ConfigLoader::init();

  printf("[boot] step1: SensingTask create\n");
  auto sensing = SensingTask::create();
  // planningはesc_の早期init()のため、main()冒頭で既に作成済み。
  printf("[boot] step2: sensing set_*\n");

  sensing->set_sensing_entity(sensing_entity);
  sensing->set_planning_task(planning);
  sensing->set_input_param_entity(param);
  sensing->set_tgt_val(tgt_val);
  printf("[boot] step3: sensing init\n");
  sensing->init();
  sensing->configure(1000); // サンプリング周期 1000us (LED安定待ちはparam->led_light_delay_cnt(_2)*led_light_delay_us_per_cntで指定)

  printf("[boot] step4: planning init\n");
  planning->set_sensing_entity(sensing_entity);
  planning->set_input_param_entity(param);
  planning->set_tgt_val(tgt_val);
  planning->init(sensing);

  printf("[boot] step5: LoggingTask + MainTask create\n");
  auto lt = LoggingTask::create();
  lt->set_error_entity(planning->ctl_.ee);
  lt->set_tgt_val(tgt_val);
  lt->set_sensing_entity(sensing_entity);
  lt->set_input_param_entity(param);
  auto main_task = MainTask::create(sensing, planning, param);
  main_task->set_logging_task(lt);
  main_task->set_tgt_val(tgt_val);

  printf("[boot] step6: PSRAM init (sfe_setup_psram)\n");
  // step6 の各段は割り込み禁止区間/XIP 停止を含むため、途中で止まると USB の
  // 送信バッファに残った printf が一切出ない(2026-09-20: CE# 直列抵抗を外して
  // CE# が浮いた基板で、step5 の表示を最後に無言で hang した)。段ごとに
  // 吐き出してから進むことで、どこで止まったかをログに残す。
  auto sync_log = [] {
    stdio_flush();
    sleep_ms(5);
  };
  sync_log();

  // early が正常で late だけ異常 → その間のファーム初期化が原因。
  // 両方異常 → 基板側(電源/CS/SD線/チップ)。
  psram_check::print_probe("early", probe_early);
  printf("[boot] step6a: late probe\n");
  sync_log();
  psram_check::ProbeResult probe_late{};
  psram_check::probe(PSRAM_CS_PIN, &probe_late);
  psram_check::print_probe("late ", probe_late);

  printf("[boot] step6b: sfe_setup_psram\n");
  sync_log();
  size_t psram_sz = sfe_setup_psram(PSRAM_CS_PIN);
  if (psram_sz == 0) {
    // ID 判定はあくまで間接証拠。not detected のままだと QPI enable も
    // QMI M1 設定もされず、下の書き込みテストが「チップが生きていても」必ず
    // 落ちるので、ID を見ずに強制初期化してから実際に書いて確かめる。
    printf("[boot] PSRAM not detected! (ID read) -- forcing QPI init to "
           "verify by actual write/readback\n");
    printf("[boot] step6c: force_qpi_init\n");
    sync_log();
    psram_check::force_qpi_init(PSRAM_CS_PIN);
  } else {
    printf("[boot] PSRAM: %u KB detected\n", (unsigned)(psram_sz / 1024));
  }
  // 直接証拠: ログが使うのと同じ非キャッシュ窓へ実際に書いて読み戻す。
  {
    printf("[boot] step6d: rw_test\n");
    sync_log();
    const size_t test_sz = psram_sz ? psram_sz : (8u * 1024u * 1024u);
    const psram_check::RwResult rw = psram_check::rw_test(0x15000000u, test_sz);
    psram_check::print_rw("boot", rw);
    psram_check::set_boot_result(rw.pass());
    if (rw.pass() && psram_sz == 0) {
      printf("[boot] NOTE: write/readback PASSED although ID read failed -- "
             "PSRAM is usable, the ID path (SO line in SPI mode) is suspect\n");
    }
    sync_log();
  }

  printf("[boot] step7: multicore_launch_core1\n");
  s_rt_sensing = sensing.get();
  s_rt_planning = planning.get();
  multicore_launch_core1_with_stack(rt_core1_entry, g_core1_stack,
                                    sizeof(g_core1_stack));

  printf("[boot] step8: MainTask start\n");
  MainTask::start();
}
