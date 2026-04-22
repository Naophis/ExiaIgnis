#include "main/main_task.hpp"
#include "define.hpp"
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <cstring>

std::shared_ptr<MainTask> MainTask::s_instance;
char          MainTask::s_print_buf[1024];
volatile bool MainTask::s_print_ready = false;

std::shared_ptr<MainTask> MainTask::create(
    std::shared_ptr<SensingTask>  sensing,
    std::shared_ptr<PlanningTask> planning)
{
    s_instance           = std::shared_ptr<MainTask>(new MainTask());
    s_instance->sensing_  = sensing;
    s_instance->planning_ = planning;
    return s_instance;
}

void MainTask::core1_entry() {
    flash_safe_execute_core_init();
    s_instance->run();
}

void MainTask::run() {
    // ブザー PWM 設定 (GPIO はどのコアからでも設定可)
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint pwm_slice   = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint pwm_channel = pwm_gpio_to_channel(BUZZER_PIN);
    pwm_set_enabled(pwm_slice, false);

    ui_.init(pwm_slice, pwm_channel);
    ui_.LED_headlight();
    ui_.hello_exia();

    bool prev_btn = false;

    while (true) {
        // ---- ボタン処理 ----
        bool btn = !gpio_get(BTN_PIN);  // active low

        if (btn != prev_btn) {
            // if (btn) {
            //     ui_.play_tone(BUZZER_FREQ_HZ);
            //     ui_.LED_on_all();

            //     // ボタン押下: planning に直進コマンドを投入
            //     PlanningTask::Command cmd;
            //     cmd.mode         = PlanningTask::MotionMode::STRAIGHT;
            //     cmd.v_max        = 200.0f;   // 200 mm/s
            //     cmd.accl         = 1000.0f;  // 1000 mm/s^2
            //     cmd.dist         = 1e9f;     // 実質無制限
            //     cmd.duty_suction = 5.0f;     // 5%
            //     planning_->send_command(cmd);
            // } else {
            //     ui_.stop_tone();
            //     ui_.LED_headlight();

            //     // ボタン離し: 減速停止
            //     PlanningTask::Command cmd;
            //     cmd.mode = PlanningTask::MotionMode::STOP;
            //     planning_->send_command(cmd);
            // }
            prev_btn = btn;
        }

        // ---- 表示文字列の組み立て (Core0 に渡す; USB 関数は Core0 のみが呼ぶ) ----
        // s_print_ready が false になるまで待つ (Core0 が前フレームを消化済み)
        if (sensing_->data_ready && !s_print_ready) {
            SensingTask::Data   d  = sensing_->data;    // volatile → コピー
            sensing_->data_ready   = false;
            PlanningTask::State ps = planning_->state;  // volatile → コピー

            char * const buf = s_print_buf;
            const int    SZ  = (int)sizeof(s_print_buf);
            const char   ESC = '\033';
            int pos = 0;

            pos += snprintf(buf + pos, SZ - pos, "%c[2J%c[0;0H", ESC, ESC);
            pos += snprintf(buf + pos, SZ - pos, "=== ExiaIgnis sensor monitor ===\n");

            pos += snprintf(buf + pos, SZ - pos,
                "[timing]  dt=%4lu us  |  sense=%4lu us"
                "  |  gz->encR: %4llu us"
                "  |  encR->encL: %4llu us\n",
                d.dt_us, d.sense_duration_us,
                d.enc_r_ts - d.gz_ts,
                d.enc_l_ts - d.enc_r_ts);

            pos += snprintf(buf + pos, SZ - pos,
                "[sensor]  L90=%4u"
                "  |  L45  2=%4u both=%4u 1=%4u"
                "  |  R45  1=%4u both=%4u 2=%4u"
                "  |  R90=%4u\n",
                d.diff.l90,
                d.diff.l45_2, d.diff.l45_both, d.diff.l45_1,
                d.diff.r45_1, d.diff.r45_both, d.diff.r45_2,
                d.diff.r90);

            pos += snprintf(buf + pos, SZ - pos,
                "[motion]  gz=%6d (dt=%4llu us)"
                "  |  encL=%5u (dt=%4llu us)"
                "  |  encR=%5u (dt=%4llu us)\n",
                d.gz, d.gz_dt,
                d.enc_l, d.enc_l_dt,
                d.enc_r, d.enc_r_dt);

            pos += snprintf(buf + pos, SZ - pos, "[power]   bat=%4u\n", d.battery);

            pos += snprintf(buf + pos, SZ - pos,
                "[planning] mode=%u  v=%6.1f  w=%6.3f"
                "  dist=%7.1f  ang=%6.3f\n",
                (uint8_t)ps.mode,
                ps.img_v, ps.img_w,
                ps.img_dist, ps.img_ang);

            pos += snprintf(buf + pos, SZ - pos,
                "[control]  duty_l=%6.1f%%  duty_r=%6.1f%%"
                "  suction=%5.1f%%  tick=%lu\n",
                ps.duty_l, ps.duty_r, ps.duty_suction, ps.tick);

            (void)pos;
            __dmb();            // store barrier: buf の書き込みを s_print_ready より先に確定
            s_print_ready = true;
        }

        sleep_ms(25);
    }
}
