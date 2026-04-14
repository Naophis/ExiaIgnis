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

    bool prev_btn = false;
    while (true) {
        bool btn_pressed = !gpio_get(BTN_PIN); // active low: LOW=pressed

        if (btn_pressed != prev_btn) {
            if (btn_pressed) {
                ui.play_tone(BUZZER_FREQ_HZ);  // ブザー優先
                ui.LED_on_all();
                printf("BTN: PRESSED  -> buzzer on, LED all on\n");
            } else {
                ui.stop_tone();               // ブザー優先
                ui.LED_headlight();
                printf("BTN: RELEASED -> buzzer off, LED headlight\n");
            }
            prev_btn = btn_pressed;
        }

        sleep_ms(10);
    }
}
