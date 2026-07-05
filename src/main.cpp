// #include "config_loader.hpp"
// #include "hardware/clocks.h"
// #include "hardware/dma.h"
// #include "hardware/gpio.h"
// #include "hardware/i2c.h"
// #include "hardware/interp.h"
// #include "hardware/pio.h"
// #include "hardware/spi.h"
// extern "C" {
// #include "sfe_psram.h"
// }
// #include "hardware/uart.h"
// #include "logging/logging_task.hpp"
// #include "main/main_task.hpp"
// #include "pico/flash.h"
// #include "pico/multicore.h"
// #include "pico/stdlib.h"
// #include "planning/planning_task.hpp"
// #include "sensing_task.hpp"
// #include <stdio.h>

// #include "blink.pio.h"
// #include "define.hpp"

// // ============================================================
// // Core1 RT エントリ: sensing + planning IRQ を Core1 に登録
// // ============================================================
// static SensingTask *s_rt_sensing = nullptr;
// static PlanningTask *s_rt_planning = nullptr;

// std::shared_ptr<sensing_result_entity_t> sensing_entity;
// std::shared_ptr<input_param_t> param;
// std::shared_ptr<motion_tgt_val_t> tgt_val;

// static void rt_core1_entry() {
//   flash_safe_execute_core_init();
//   s_rt_sensing->start_irq();  // TIMER0 IRQ → Core1
//   s_rt_planning->start_irq(); // TIMER1 IRQ → Core1
//   while (true)
//     __wfi();
// }

// // Core1 用 8KB スタック (正規 SRAM 上)
// // SCRATCH_X は 4KB 固定のため multicore_launch_core1_with_stack() 経由で確保。
// static uint32_t g_core1_stack[0x2000 / sizeof(uint32_t)];

// // ============================================================
// // Core0 main: 初期化 → Core1 起動 → MainTask (printf/UI) を実行
// // ============================================================
// int main() {
//   sensing_entity = std::make_shared<sensing_result_entity_t>();
//   param = std::make_shared<input_param_t>();
//   tgt_val = std::make_shared<motion_tgt_val_t>();

//   stdio_init_all();
//   set_sys_clock_khz(150000, true);

//   // Tactile switch: pull-up (active low)
//   gpio_init(BTN_PIN);
//   gpio_set_dir(BTN_PIN, GPIO_IN);
//   gpio_pull_up(BTN_PIN);

//   sleep_ms(1500);
//   // 設定ファイル読み込み (multicore 起動前に実施)
//   ConfigLoader::init();

//   printf("[boot] step1: SensingTask create\n");
//   auto sensing = SensingTask::create();
//   auto planning = PlanningTask::create();
//   printf("[boot] step2: sensing set_*\n");

//   sensing->set_sensing_entity(sensing_entity);
//   sensing->set_planning_task(planning);
//   sensing->set_input_param_entity(param);
//   sensing->set_tgt_val(tgt_val);
//   printf("[boot] step3: sensing init\n");
//   sensing->init();
//   sensing->configure(22, 1000); // LED安定待ち 22us, サンプリング周期 1000us

//   printf("[boot] step4: planning init\n");
//   planning->set_sensing_entity(sensing_entity);
//   planning->set_input_param_entity(param);
//   planning->set_tgt_val(tgt_val);
//   planning->init(sensing);

//   printf("[boot] step5: LoggingTask + MainTask create\n");
//   auto lt = LoggingTask::create();
//   lt->set_error_entity(planning->ctl_.ee);
//   lt->set_tgt_val(tgt_val);
//   lt->set_sensing_entity(sensing_entity);
//   lt->set_input_param_entity(param);
//   auto main_task = MainTask::create(sensing, planning, param);
//   main_task->set_logging_task(lt);
//   main_task->set_tgt_val(tgt_val);

//   printf("[boot] step6: PSRAM init (sfe_setup_psram)\n");
//   size_t psram_sz = sfe_setup_psram(PSRAM_CS_PIN);
//   if (psram_sz == 0) {
//     printf("[boot] PSRAM not detected!\n");
//   } else {
//     printf("[boot] PSRAM: %u KB detected\n", (unsigned)(psram_sz / 1024));
//   }

//   printf("[boot] step7: multicore_launch_core1\n");
//   s_rt_sensing = sensing.get();
//   s_rt_planning = planning.get();
//   multicore_launch_core1_with_stack(rt_core1_entry, g_core1_stack, sizeof(g_core1_stack));

//   printf("[boot] step8: MainTask start\n");
//   MainTask::start();
// }

#include <math.h>
#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"

// ===== ピン設定 =====
#define U_PWM      9
#define V_PWM      10
#define W_PWM      11
#define ENABLE_PIN 8

// ===== PWM 設定 =====
#define PWM_FREQ_HZ  40000u                   // 40kHz: 1380Hz時 29サンプル/回転
#define PWM_WRAP     2047u                    // 11bit: 150MHz/(40kHz*2048)=1.83分周
#define CONTROL_HZ   40000u
#define CONTROL_US   (1000000u / CONTROL_HZ)  // 25 us

// ===== 数学定数 =====
#define TWO_PI    6.2831853f
#define PHASE_120 2.0943951f

// ===== モーターパラメータ =====
#define ALIGN_MS        600
#define START_ELEC_HZ   1.0f
#define TARGET_ELEC_HZ  120.0f   // デフォルト。fキーで変更可能
#define RAMP_HZ_PER_SEC 1000.0f  // 共振帯を瞬時通過

// V/Hz 制御: 基準周波数・振幅は実証値。以降は比例スケール
// amp = AMP_BASE * max(hz / AMP_BASE_HZ, 1.0) * g_amp_gain
#define AMP_BASE    0.06f
#define AMP_BASE_HZ 120.0f
#define MAX_AMP     0.35f

#define POLE_PAIRS 6u   // 極対数 (12極=6, 14極=7 など)
#define DIR        1.0f

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_ALIGN,
    MOTOR_RAMP,
    MOTOR_RUN,
} motor_state_t;

// IRQ と main() の両方からアクセスする変数は volatile
static volatile motor_state_t g_state      = MOTOR_STOP;
static volatile float         g_theta      = 0.0f;
static volatile float         g_elec_hz    = 0.0f;
static volatile float         g_target_hz  = TARGET_ELEC_HZ;
static volatile float         g_amp_gain   = 0.10f; // +/-キーで調整 (実証値: 3600Hz+)
static volatile uint32_t      g_state_cnt  = 0;

static repeating_timer_t g_ctrl_timer;
static bool              g_timer_running = false;

static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : x > hi ? hi : x;
}

static void pwm_write_01(uint gpio, float duty) {
    uint slice = pwm_gpio_to_slice_num(gpio);
    uint chan  = pwm_gpio_to_channel(gpio);
    pwm_set_chan_level(slice, chan, (uint16_t)(PWM_WRAP * clampf(duty, 0.0f, 1.0f)));
}

static void set_phase_spwm(float angle, float amp) {
    amp = clampf(amp, 0.0f, MAX_AMP);
    pwm_write_01(U_PWM, 0.5f + amp * sinf(angle));
    pwm_write_01(V_PWM, 0.5f + amp * sinf(angle - PHASE_120));
    pwm_write_01(W_PWM, 0.5f + amp * sinf(angle + PHASE_120));
}

static void set_all_pwm_low(void) {
    pwm_write_01(U_PWM, 0.0f);
    pwm_write_01(V_PWM, 0.0f);
    pwm_write_01(W_PWM, 0.0f);
}

static void init_pwm_pin(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint       slice = pwm_gpio_to_slice_num(gpio);
    pwm_config cfg   = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    float div = (float)clock_get_hz(clk_sys) / ((float)PWM_FREQ_HZ * (PWM_WRAP + 1u));
    pwm_config_set_clkdiv(&cfg, div);
    pwm_init(slice, &cfg, true);
}

// V/Hz: AMP_BASE_HZ 以下は AMP_BASE 固定（起動保証）、以上は gain でスケール
static inline float amp_from_hz(float hz) {
    if (hz <= AMP_BASE_HZ) return AMP_BASE;
    return clampf(AMP_BASE * (hz / AMP_BASE_HZ) * g_amp_gain, AMP_BASE, MAX_AMP);
}

// ===== 制御ループ: タイマーIRQから呼ばれる (4kHz 固定周期) =====
static bool motor_step_cb(repeating_timer_t *rt) {
    const float dt = 1.0f / (float)CONTROL_HZ;

    switch (g_state) {
    case MOTOR_STOP:
        break;

    case MOTOR_ALIGN:
        set_phase_spwm(0.0f, AMP_BASE);
        g_state_cnt++;
        if (g_state_cnt >= (ALIGN_MS * CONTROL_HZ / 1000u)) {
            g_state     = MOTOR_RAMP;
            g_state_cnt = 0;
            g_theta     = 0.0f;
            g_elec_hz   = START_ELEC_HZ;
        }
        break;

    case MOTOR_RAMP:
        g_elec_hz += RAMP_HZ_PER_SEC * dt;
        if (g_elec_hz > g_target_hz) g_elec_hz = g_target_hz;

        g_theta += DIR * TWO_PI * g_elec_hz * dt;
        if (g_theta >= TWO_PI) g_theta -= TWO_PI;
        if (g_theta < 0.0f) g_theta += TWO_PI;

        set_phase_spwm(g_theta, amp_from_hz(g_elec_hz));

        if (g_elec_hz >= g_target_hz) {
            g_state = MOTOR_RUN;
        }
        break;

    case MOTOR_RUN:
        if (g_elec_hz < g_target_hz) {
            g_elec_hz += RAMP_HZ_PER_SEC * dt;
            if (g_elec_hz > g_target_hz) g_elec_hz = g_target_hz;
        } else if (g_elec_hz > g_target_hz) {
            g_elec_hz -= RAMP_HZ_PER_SEC * dt;
            if (g_elec_hz < g_target_hz) g_elec_hz = g_target_hz;
        }

        g_theta += DIR * TWO_PI * g_elec_hz * dt;
        if (g_theta >= TWO_PI) g_theta -= TWO_PI;
        if (g_theta < 0.0f) g_theta += TWO_PI;

        set_phase_spwm(g_theta, amp_from_hz(g_elec_hz));
        break;
    }
    return true;
}

static void motor_start(void) {
    if (g_timer_running) {
        cancel_repeating_timer(&g_ctrl_timer);
        g_timer_running = false;
    }

    set_phase_spwm(0.0f, AMP_BASE);
    gpio_put(ENABLE_PIN, 1);
    sleep_ms(20);  // nSLEEP 起動待ち

    g_state     = MOTOR_ALIGN;
    g_theta     = 0.0f;
    g_elec_hz   = 0.0f;
    g_state_cnt = 0;

    add_repeating_timer_us(CONTROL_US, motor_step_cb, NULL, &g_ctrl_timer);
    g_timer_running = true;
}

static void motor_stop(void) {
    if (g_timer_running) {
        cancel_repeating_timer(&g_ctrl_timer);
        g_timer_running = false;
    }
    g_state   = MOTOR_STOP;
    g_theta   = 0.0f;
    g_elec_hz = 0.0f;

    set_all_pwm_low();
    gpio_put(ENABLE_PIN, 0);
}

static const char *state_name(motor_state_t s) {
    switch (s) {
    case MOTOR_STOP:  return "STOP";
    case MOTOR_ALIGN: return "ALIGN";
    case MOTOR_RAMP:  return "RAMP";
    case MOTOR_RUN:   return "RUN";
    default:          return "?";
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);

    printf("\nBLDC SPWM test (RP2350)\n");
    printf("r=start  s=stop  +/-=amp  f/d=freq\n");

    gpio_init(ENABLE_PIN);
    gpio_set_dir(ENABLE_PIN, GPIO_OUT);
    gpio_put(ENABLE_PIN, 0);

    init_pwm_pin(U_PWM);
    init_pwm_pin(V_PWM);
    init_pwm_pin(W_PWM);
    set_all_pwm_low();

    motor_start();

    uint64_t last_print_us = time_us_64();

    while (true) {
        int c = getchar_timeout_us(0);
        if (c >= 0) {
            if (c == 'r') {
                motor_start();
                printf("start\n");
            } else if (c == 's') {
                motor_stop();
                printf("stop\n");
            } else if (c == '+') {
                g_amp_gain = clampf(g_amp_gain + 0.01f, 0.025f, 5.0f);
                printf("amp_gain=%.2f\n", (double)g_amp_gain);
            } else if (c == '-') {
                g_amp_gain = clampf(g_amp_gain - 0.01f, 0.025f, 5.0f);
                printf("amp_gain=%.2f\n", (double)g_amp_gain);
            } else if (c == 'f') {
                g_target_hz += 10.0f;
                printf("target_hz=%.1f\n", (double)g_target_hz);
            } else if (c == 'd') {
                g_target_hz = clampf(g_target_hz - 10.0f, 1.0f, 10000.0f);
                printf("target_hz=%.1f\n", (double)g_target_hz);
            }
        }

        uint64_t now = time_us_64();
        if (now - last_print_us >= 500000u) {
            last_print_us = now;
            // volatile 変数はローカルにコピーしてから printf
            motor_state_t st   = g_state;
            float         hz   = g_elec_hz;
            float         tgt  = g_target_hz;
            float         gain = g_amp_gain;
            float rpm = hz * 60.0f / (float)POLE_PAIRS;
            printf("state=%s elec_hz=%.1f target=%.1f amp=%.3f gain=%.2f rpm=%.0f\n",
                   state_name(st), (double)hz, (double)tgt,
                   (double)amp_from_hz(hz), (double)gain, (double)rpm);
        }
    }
}
