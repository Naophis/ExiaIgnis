#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "sensing_task.hpp"
#include "planning/planning_task.hpp"
#include "ui.hpp"
#include "config_loader.hpp"

#include "define.hpp"

// Data will be copied from src to dst
const char src[] = "Hello, world! (from DMA)";
char dst[count_of(src)];

#include "blink.pio.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}



int main()
{
    stdio_init_all();
    set_sys_clock_khz(150000, true);

    // Tactile switch input with pull-up (active low)
    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);

    // PWM setup: buzzer (周波数はui.play_tone/set_pwm_freqで設定)
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint pwm_slice   = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint pwm_channel = pwm_gpio_to_channel(BUZZER_PIN);
    pwm_set_enabled(pwm_slice, false);

    // UserInterface: I2C1 (LED driver) + PWM buzzer を初期化
    UserInterface ui;
    ui.init(pwm_slice, pwm_channel);
    ui.LED_headlight();
    ui.hello_exia();

    // 設定ファイル読み込み (multicore 起動前に実施)
    ConfigLoader::init();

    auto sensing = SensingTask::create();
    sensing->init();
    sensing->configure(
        (uint32_t)ConfigLoader::get_int("sensing.led_settle_us", 13),
        (uint32_t)ConfigLoader::get_int("sensing.interval_us",   1000)
    );
    sleep_ms(300);  // enc setup ログを画面クリア前に確認するための待機（デバッグ用）
    multicore_launch_core1(SensingTask::core1_entry);

    // PlanningTask: モーター/吸引 PWM 初期化 + TIMER1 alarm 0 (Core0) で 1kHz 動作
    auto planning = PlanningTask::create();
    planning->init(sensing);

    bool prev_btn = false;
    while (true) {
        bool btn_pressed = !gpio_get(BTN_PIN); // active low: LOW=pressed

        if (btn_pressed != prev_btn) {
            if (btn_pressed) {
                ui.play_tone(BUZZER_FREQ_HZ);
                ui.LED_on_all();
                // ボタン押下: 直進コマンドを投入 (速度 200mm/s, 実質無制限距離)
                PlanningTask::Command cmd;
                cmd.mode          = PlanningTask::MotionMode::STRAIGHT;
                cmd.v_max         = 200.0f;
                cmd.accl          = 1000.0f;
                cmd.dist          = 1e9f;
                cmd.duty_suction  = 5.0f;
                planning->send_command(cmd);
            } else {
                ui.stop_tone();
                ui.LED_headlight();
                // ボタン離し: 減速停止
                PlanningTask::Command cmd;
                cmd.mode = PlanningTask::MotionMode::STOP;
                planning->send_command(cmd);
            }
            prev_btn = btn_pressed;
        }

        if (sensing->data_ready) {
            SensingTask::Data d = sensing->data;  // volatile からコピー
            sensing->data_ready = false;

            const char ESC = '\033';
            printf("%c[2J",   ESC);
            printf("%c[0;0H", ESC);

            printf("=== ExiaIgnis sensor monitor ===\n");

            printf("[timing]  dt=%4lu us  |  sense=%4lu us"
                   "  |  gz->encR: %4llu us"
                   "  |  encR->encL: %4llu us\n",
                   d.dt_us, d.sense_duration_us,
                   d.enc_r_ts - d.gz_ts,
                   d.enc_l_ts - d.enc_r_ts);

            printf("[sensor]  L90=%4u"
                   "  |  L45  2=%4u both=%4u 1=%4u"
                   "  |  R45  1=%4u both=%4u 2=%4u"
                   "  |  R90=%4u\n",
                   d.diff.l90,
                   d.diff.l45_2, d.diff.l45_both, d.diff.l45_1,
                   d.diff.r45_1, d.diff.r45_both, d.diff.r45_2,
                   d.diff.r90);

            printf("[motion]  gz=%6d (dt=%4llu us)"
                   "  |  encL=%5u (dt=%4llu us)"
                   "  |  encR=%5u (dt=%4llu us)\n",
                   d.gz, d.gz_dt,
                   d.enc_l,  d.enc_l_dt,
                   d.enc_r, d.enc_r_dt);

            printf("[power]   bat=%4u\n", d.battery);

            PlanningTask::State ps = planning->state;  // volatile からコピー
            printf("[planning] mode=%u  v=%6.1f  w=%6.3f"
                   "  dist=%7.1f  ang=%6.3f\n",
                   (uint8_t)ps.mode,
                   ps.img_v, ps.img_w,
                   ps.img_dist, ps.img_ang);
            printf("[control]  duty_l=%6.1f%%  duty_r=%6.1f%%"
                   "  suction=%5.1f%%  tick=%lu\n",
                   ps.duty_l, ps.duty_r, ps.duty_suction, ps.tick);
        }

        sleep_ms(10);
    }
}
