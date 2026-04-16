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
#include "ui.hpp"

#include "define.hpp"

// Data will be copied from src to dst
const char src[] = "Hello, world! (from DMA)";
char dst[count_of(src)];

#include "blink.pio.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
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

    // PWM setup: GPIO16 = PWM0 A (周波数はui.play_tone/set_pwm_freqで設定)
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint pwm_slice   = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint pwm_channel = pwm_gpio_to_channel(BUZZER_PIN);
    pwm_set_enabled(pwm_slice, false); // 初期状態はOFF

    // UserInterface: I2C1 (LED driver) + PWM buzzer を初期化
    UserInterface ui;
    ui.init(pwm_slice, pwm_channel);
    ui.LED_headlight();
    ui.hello_exia();

    auto sensing = SensingTask::create();
    sensing->init();
    multicore_launch_core1(SensingTask::core1_entry);

    bool prev_btn = false;
    while (true) {
        bool btn_pressed = !gpio_get(BTN_PIN); // active low: LOW=pressed

        if (btn_pressed != prev_btn) {
            if (btn_pressed) {
                ui.play_tone(BUZZER_FREQ_HZ);
                ui.LED_on_all();
            } else {
                ui.stop_tone();
                ui.LED_headlight();
            }
            prev_btn = btn_pressed;
        }

        if (sensing->data_ready) {
            SensingTask::Data d = sensing->data;  // volatile からコピー
            sensing->data_ready = false;

            const char ESC = '\033';
            printf("%c[2J",   ESC);   // 画面消去
            printf("%c[0;0H", ESC);   // カーソルを先頭に戻す

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
        }

        sleep_ms(10);
    }
}
