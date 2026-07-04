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
#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// ===== Pin assign =====
#define U_PWM      9
#define V_PWM      10
#define W_PWM      11
#define ENABLE_PIN 8   // ENA/ENB/ENC/nSLEEP common

// ===== PWM / control settings =====
#define PWM_FREQ_HZ     20000u   // MP6540H入力PWM: 20kHz
#define PWM_WRAP        4095u
#define CONTROL_HZ      4000u    // SPWM duty更新周期。PWM周期ではない

#define TWO_PI          6.2831853071795864769f
#define PHASE_120       2.0943951023931954923f

// ===== Motor start settings =====
// 最初は弱く。BETAFPV 1103 3Sは電流が出やすいので上げすぎ注意
#define ALIGN_MS        400
#define ALIGN_AMP       0.06f
#define RUN_AMP_INIT    0.08f
#define MAX_AMP         0.25f

// electrical Hz。極対数を考慮した機械回転数ではない。
// mech_rpm = elec_hz * 60 / pole_pairs
#define START_ELEC_HZ   8.0f
#define TARGET_ELEC_HZ  120.0f
#define RAMP_HZ_PER_SEC 180.0f

// 回転方向。逆なら -1.0f にするか、モーター線2本を入れ替える
#define DIR             1.0f

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_ALIGN,
    MOTOR_RAMP,
    MOTOR_RUN,
} motor_state_t;

static motor_state_t state = MOTOR_STOP;

static float theta = 0.0f;
static float elec_hz = 0.0f;
static float target_elec_hz = TARGET_ELEC_HZ;
static float run_amp = RUN_AMP_INIT;
static uint32_t state_count = 0;

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void pwm_write_01(uint gpio, float duty)
{
    duty = clampf(duty, 0.0f, 1.0f);

    uint slice = pwm_gpio_to_slice_num(gpio);
    uint chan  = pwm_gpio_to_channel(gpio);

    // level=0で0%、level=TOP+1で100%相当だが、ここでは0..TOPに丸める
    uint16_t level = (uint16_t)((float)PWM_WRAP * duty);
    pwm_set_chan_level(slice, chan, level);
}

static void set_phase_spwm(float angle, float amp)
{
    amp = clampf(amp, 0.0f, MAX_AMP);

    // 中心50%の三相正弦波PWM
    // duty範囲 = 0.5 ± amp
    float du = 0.5f + amp * sinf(angle);
    float dv = 0.5f + amp * sinf(angle - PHASE_120);
    float dw = 0.5f + amp * sinf(angle + PHASE_120);

    pwm_write_01(U_PWM, du);
    pwm_write_01(V_PWM, dv);
    pwm_write_01(W_PWM, dw);
}

static void set_all_pwm_low(void)
{
    pwm_write_01(U_PWM, 0.0f);
    pwm_write_01(V_PWM, 0.0f);
    pwm_write_01(W_PWM, 0.0f);
}

static void init_pwm_pin(uint gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(gpio);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, PWM_WRAP);

    float clk_hz = (float)clock_get_hz(clk_sys);
    float div = clk_hz / ((float)PWM_FREQ_HZ * (float)(PWM_WRAP + 1u));
    pwm_config_set_clkdiv(&cfg, div);

    pwm_init(slice, &cfg, true);
}

static void motor_stop(void)
{
    state = MOTOR_STOP;
    theta = 0.0f;
    elec_hz = 0.0f;
    state_count = 0;

    set_all_pwm_low();

    // ENABLE_PINはnSLEEPも兼ねているのでLowでMP6540Hをsleepへ
    gpio_put(ENABLE_PIN, 0);
}

static void motor_start(void)
{
    // Enable前にPWMを既知状態へ
    set_phase_spwm(0.0f, ALIGN_AMP);

    gpio_put(ENABLE_PIN, 1);

    // nSLEEP立ち上げ後の起動待ち
    sleep_ms(20);

    state = MOTOR_ALIGN;
    theta = 0.0f;
    elec_hz = 0.0f;
    state_count = 0;
}

static void motor_step(void)
{
    const float dt = 1.0f / (float)CONTROL_HZ;

    switch (state) {
    case MOTOR_STOP:
        return;

    case MOTOR_ALIGN:
        // 初期位置合わせ。ここで「ぐっ」と励磁感が出るはず
        set_phase_spwm(0.0f, ALIGN_AMP);
        state_count++;

        if (state_count >= (ALIGN_MS * CONTROL_HZ / 1000u)) {
            state = MOTOR_RAMP;
            state_count = 0;
            theta = 0.0f;
            elec_hz = START_ELEC_HZ;
        }
        break;

    case MOTOR_RAMP:
        if (elec_hz < target_elec_hz) {
            elec_hz += RAMP_HZ_PER_SEC * dt;
            if (elec_hz > target_elec_hz) elec_hz = target_elec_hz;
        } else {
            state = MOTOR_RUN;
        }

        theta += DIR * TWO_PI * elec_hz * dt;
        if (theta > TWO_PI) theta -= TWO_PI;
        if (theta < 0.0f) theta += TWO_PI;

        set_phase_spwm(theta, run_amp);
        break;

    case MOTOR_RUN:
        theta += DIR * TWO_PI * target_elec_hz * dt;
        if (theta > TWO_PI) theta -= TWO_PI;
        if (theta < 0.0f) theta += TWO_PI;

        elec_hz = target_elec_hz;
        set_phase_spwm(theta, run_amp);
        break;
    }
}

static const char *state_name(motor_state_t s)
{
    switch (s) {
    case MOTOR_STOP:  return "STOP";
    case MOTOR_ALIGN: return "ALIGN";
    case MOTOR_RAMP:  return "RAMP";
    case MOTOR_RUN:   return "RUN";
    default:          return "?";
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(1500);

    printf("\nMP6540H BLDC SPWM test for RP2350\n");
    printf("U=%d V=%d W=%d ENABLE=%d\n", U_PWM, V_PWM, W_PWM, ENABLE_PIN);
    printf("keys: r=start, s=stop, +=amp up, -=amp down, f=freq up, d=freq down\n");

    gpio_init(ENABLE_PIN);
    gpio_set_dir(ENABLE_PIN, GPIO_OUT);
    gpio_put(ENABLE_PIN, 0);

    init_pwm_pin(U_PWM);
    init_pwm_pin(V_PWM);
    init_pwm_pin(W_PWM);

    set_all_pwm_low();

    // 自動開始。怖ければコメントアウトして、USBシリアルから r を押す
    motor_start();

    uint64_t next_us = time_us_64();
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
                run_amp = clampf(run_amp + 0.01f, 0.0f, MAX_AMP);
                printf("amp=%.3f\n", run_amp);
            } else if (c == '-') {
                run_amp = clampf(run_amp - 0.01f, 0.0f, MAX_AMP);
                printf("amp=%.3f\n", run_amp);
            } else if (c == 'f') {
                target_elec_hz += 10.0f;
                printf("target_elec_hz=%.1f\n", target_elec_hz);
            } else if (c == 'd') {
                target_elec_hz -= 10.0f;
                if (target_elec_hz < 1.0f) target_elec_hz = 1.0f;
                printf("target_elec_hz=%.1f\n", target_elec_hz);
            }
        }

        uint64_t now = time_us_64();
        if ((int64_t)(now - next_us) >= 0) {
            next_us += 1000000u / CONTROL_HZ;
            motor_step();
        }

        if (now - last_print_us > 500000u) {
            last_print_us = now;
            printf("state=%s amp=%.3f elec_hz=%.1f target=%.1f enable=%d\n",
                   state_name(state), run_amp, elec_hz, target_elec_hz,
                   gpio_get(ENABLE_PIN));
        }
    }
}