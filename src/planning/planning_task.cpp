#include "planning/planning_task.hpp"
#include "logging/logging_task.hpp"
#include "define.hpp"              // M_PWM_L1/L2/R1/R2, SUCTION_PWM, MOTOR_PWM_FREQ_HZ
#include "pico/stdlib.h"
#include "hardware/clocks.h"      // clock_get_hz
#include "hardware/timer.h"       // timer1_hw, TIMER1_IRQ_0
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"        // __dmb
#include <cmath>
#include <algorithm>

// ============================================================
// 機体パラメータ (要チューニング)
// ============================================================
static constexpr float TIRE_DIAM_MM = 12.0f;        // 車輪直径 [mm]
static constexpr float GEAR_RATIO   = 37.0f / 8.0f; // モーター:車輪 ギア比
static constexpr float TREAD_MM     = 38.0f;         // 左右車輪間距離 [mm]
static constexpr float ENC_STEPS    = 16384.0f;      // 14-bit エンコーダ分解能
static constexpr float MM_PER_STEP  =
    (float)(M_PI) * TIRE_DIAM_MM / (ENC_STEPS * GEAR_RATIO);

// ============================================================
// PID ゲイン (初期値: 実機で要チューニング)
// ============================================================
static constexpr float VEL_KP  = 0.5f;
static constexpr float VEL_KI  = 0.1f;
static constexpr float GYRO_KP = 0.3f;
static constexpr float GYRO_KI = 0.05f;
static constexpr float DUTY_MAX = 90.0f;  // デューティ上限 [%]

// ============================================================
// static メンバの定義
// ============================================================
std::shared_ptr<PlanningTask> PlanningTask::s_instance;

std::shared_ptr<PlanningTask> PlanningTask::create() {
    s_instance = std::shared_ptr<PlanningTask>(new PlanningTask());
    return s_instance;
}

// ============================================================
// 初期化 (Core0 の main から呼ぶ)
// ============================================================
void PlanningTask::init(std::shared_ptr<SensingTask> sensing) {
    sensing_ = sensing;

    // --- モーター / 吸引 PWM GPIO 設定 ---
    const uint motor_pins[] = { M_PWM_L1, M_PWM_L2, M_PWM_R1, M_PWM_R2, SUCTION_PWM };
    for (uint pin : motor_pins) {
        gpio_set_function(pin, GPIO_FUNC_PWM);
    }

    slice_L_ = pwm_gpio_to_slice_num(M_PWM_L1);
    slice_R_ = pwm_gpio_to_slice_num(M_PWM_R1);
    slice_S_ = pwm_gpio_to_slice_num(SUCTION_PWM);

    // wrap = sys_clk / pwm_freq - 1  (クロック変更後に呼ぶこと)
    motor_wrap_ = (uint32_t)(clock_get_hz(clk_sys) / MOTOR_PWM_FREQ_HZ) - 1u;

    for (uint slice : { slice_L_, slice_R_, slice_S_ }) {
        pwm_set_clkdiv_int_frac4(slice, 1, 0);  // 分周なし
        pwm_set_wrap(slice, motor_wrap_);
        pwm_set_enabled(slice, true);
    }

    // 初期デューティ: 全チャンネル 0
    apply_motor();
}

// ============================================================
// TIMER1 IRQ 登録 (Core1 エントリから呼ぶ)
// ============================================================
void PlanningTask::start_irq() {
    irq_set_exclusive_handler(TIMER1_IRQ_0, timer_irq_handler);
    irq_set_priority(TIMER1_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(TIMER1_IRQ_0, true);
    hw_set_bits(&timer1_hw->inte, 1u << 0);

    next_alarm_ = timer1_hw->timelr + interval_us_;
    timer1_hw->alarm[0] = next_alarm_;
}

// ============================================================
// コマンド投入 (Core0 main ループから呼ぶ, IRQ-safe)
// ============================================================
// Core1 (MainTask) から Core0 (Planning IRQ) へ cross-core 安全に投入する。
// __dmb() で全フィールドの書き込みが cmd_pending_ より前に完了することを保証する。
void PlanningTask::send_command(const Command &cmd) {
    pending_cmd_.mode          = cmd.mode;
    pending_cmd_.v_max         = cmd.v_max;
    pending_cmd_.v_end         = cmd.v_end;
    pending_cmd_.accl          = cmd.accl;
    pending_cmd_.decel         = cmd.decel;
    pending_cmd_.dist          = cmd.dist;
    pending_cmd_.w_max         = cmd.w_max;
    pending_cmd_.alpha         = cmd.alpha;
    pending_cmd_.ang           = cmd.ang;
    pending_cmd_.duty_suction  = cmd.duty_suction;
    pending_cmd_.timestamp     = cmd.timestamp;
    __dmb();
    cmd_pending_ = true;
}

// Astraea 互換: motion_tgt_val_t ベースのコマンド。
// Astraea の xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite) に相当。
// IRQ が次の tick で cp_request() 相当の処理を実行する。
void PlanningTask::send_command(std::shared_ptr<motion_tgt_val_t> tgt) {
    pending_tgt_ = tgt;
    __dmb();
    tgt_cmd_pending_ = true;
}

// ============================================================
// TIMER1 alarm 0 IRQ ハンドラ (Core0)
// ============================================================
void PlanningTask::timer_irq_handler() {
    timer1_hw->intr = 1u << 0;

    auto *self = s_instance.get();

    // 次回アラームを絶対時刻で設定 (ドリフトなし)
    self->next_alarm_ += self->interval_us_;
    timer1_hw->alarm[0] = self->next_alarm_;

    uint64_t now   = time_us_64();
    uint32_t dt_us = self->prev_ts_ ? (uint32_t)(now - self->prev_ts_) : 0;
    self->prev_ts_ = now;

    self->tick(dt_us);
}

// ============================================================
// 1tick 処理 (IRQ ハンドラから呼ばれる)
// ============================================================
void PlanningTask::tick(uint32_t dt_us) {
    if (dt_us == 0) return;

    // --- コマンド受け取り (Core1 からの cross-core 書き込みを安全に読む) ---
    if (cmd_pending_) {
        __dmb();  // load barrier: cmd_pending_ を読んだ後、pending_cmd_ フィールドを確実に読む
        active_cmd_  = Command(pending_cmd_);
        cmd_pending_ = false;
        // 新コマンド受信: 積分項・軌道をリセット
        vel_err_i_  = 0.0f;
        gyro_err_i_ = 0.0f;
        img_v_      = 0.0f;
        img_w_      = 0.0f;
        img_dist_   = 0.0f;
        img_ang_    = 0.0f;
    }

    // --- センシングデータをスナップショット ---
    SensingTask::Data d = sensing_->data;

    // --- 初回 tick: エンコーダ初期値を記録してスキップ ---
    if (first_tick_) {
        enc_r_prev_ = d.enc_r;
        enc_l_prev_ = d.enc_l;
        first_tick_ = false;
        return;
    }

    float dt = dt_us * 1e-6f;

    // --- 速度推定 (エンコーダ差分) ---
    int16_t dr = (int16_t)(d.enc_r - enc_r_prev_);
    int16_t dl = (int16_t)(d.enc_l - enc_l_prev_);
    float vr = (float)dr * MM_PER_STEP / dt;
    float vl = (float)dl * MM_PER_STEP / dt;
    v_est_ = (vr + vl) * 0.5f;
    w_est_ = (vr - vl) / TREAD_MM;  // [rad/s]
    enc_r_prev_ = d.enc_r;
    enc_l_prev_ = d.enc_l;

    // --- 吸引デューティを最新コマンドから反映 ---
    duty_suction_ = active_cmd_.duty_suction;

    // --- 軌道生成 → 制御量計算 → モーター出力 ---
    update_trajectory(dt);
    update_control(dt);
    apply_motor();

    // --- 状態公開 (Core0 main ループが参照) ---
    state.img_v        = img_v_;
    state.img_w        = img_w_;
    state.img_dist     = img_dist_;
    state.img_ang      = img_ang_;
    state.v_est        = v_est_;
    state.w_est        = w_est_;
    state.duty_l       = duty_l_;
    state.duty_r       = duty_r_;
    state.duty_suction = duty_suction_;
    state.mode         = active_cmd_.mode;
    state.tick++;

    // --- ログ記録 (LoggingTask が active な場合のみ実行) ---
    {
        PlanningTask::State snap(state);        // volatile → non-volatile コピー
        LoggingTask::append_from_irq(d, snap);
    }
}

// ============================================================
// 軌道生成 (台形速度プロファイル)
// ============================================================
void PlanningTask::update_trajectory(float dt) {
    const Command &cmd = active_cmd_;

    switch (cmd.mode) {
    case MotionMode::IDLE:
        img_v_ = 0.0f;
        img_w_ = 0.0f;
        break;

    case MotionMode::STOP: {
        float dv = (cmd.decel > 0.0f ? cmd.decel : 3000.0f) * dt;
        if (img_v_ > 0.0f) {
            img_v_ = std::max(0.0f, img_v_ - dv);
        } else if (img_v_ < 0.0f) {
            img_v_ = std::min(0.0f, img_v_ + dv);
        }
        img_w_ = 0.0f;
        if (img_v_ == 0.0f) {
            active_cmd_.mode = MotionMode::IDLE;
        }
        break;
    }

    case MotionMode::STRAIGHT: {
        float remaining = cmd.dist - img_dist_;
        if (remaining <= 0.0f) {
            img_v_           = cmd.v_end;
            active_cmd_.mode = MotionMode::STOP;
            break;
        }
        float decel    = cmd.decel > 0.0f ? cmd.decel : cmd.accl;
        float decel_d  = std::max(0.0f,
            (img_v_ * img_v_ - cmd.v_end * cmd.v_end) / (2.0f * decel));

        if (remaining <= decel_d) {
            img_v_ -= decel * dt;
            if (img_v_ < cmd.v_end) img_v_ = cmd.v_end;
        } else {
            if (img_v_ < cmd.v_max) {
                img_v_ = std::min(cmd.v_max, img_v_ + cmd.accl * dt);
            }
        }
        img_dist_ += img_v_ * dt;
        img_w_ = 0.0f;
        break;
    }

    case MotionMode::PIVOT: {
        float sign      = cmd.ang >= 0.0f ? 1.0f : -1.0f;
        float remaining = std::fabs(cmd.ang) - std::fabs(img_ang_);
        if (remaining <= 0.0f) {
            img_w_           = 0.0f;
            active_cmd_.mode = MotionMode::IDLE;
            break;
        }
        float alpha   = cmd.alpha > 0.0f ? cmd.alpha : 30.0f;
        float decel_a = (img_w_ * img_w_) / (2.0f * alpha);
        if (remaining <= decel_a) {
            img_w_ -= sign * alpha * dt;
            if ((sign > 0.0f && img_w_ < 0.0f) ||
                (sign < 0.0f && img_w_ > 0.0f)) {
                img_w_ = 0.0f;
            }
        } else {
            if (std::fabs(img_w_) < cmd.w_max) {
                img_w_ += sign * alpha * dt;
                if (std::fabs(img_w_) > cmd.w_max)
                    img_w_ = sign * cmd.w_max;
            }
        }
        img_ang_ += img_w_ * dt;
        img_v_ = 0.0f;
        break;
    }
    }
}

// ============================================================
// PID 制御量計算
// ============================================================
void PlanningTask::update_control(float dt) {
    if (active_cmd_.mode == MotionMode::IDLE) {
        duty_l_     = 0.0f;
        duty_r_     = 0.0f;
        vel_err_i_  = 0.0f;
        gyro_err_i_ = 0.0f;
        return;
    }

    float vel_err  = img_v_ - v_est_;
    vel_err_i_    += vel_err * dt;
    float duty_c   = VEL_KP * vel_err + VEL_KI * vel_err_i_;

    float gyro_err = img_w_ - w_est_;
    gyro_err_i_   += gyro_err * dt;
    float duty_roll = GYRO_KP * gyro_err + GYRO_KI * gyro_err_i_;

    duty_r_ = std::clamp(duty_c + duty_roll, -DUTY_MAX, DUTY_MAX);
    duty_l_ = std::clamp(duty_c - duty_roll, -DUTY_MAX, DUTY_MAX);
}

// ============================================================
// モーター / 吸引 PWM 出力
// ============================================================
void PlanningTask::apply_motor() {
    // 左右モーター: duty > 0 = 正転 (CHAN_A), duty < 0 = 逆転 (CHAN_B)
    auto set_drive = [&](uint slice, float duty) {
        uint16_t level = (uint16_t)((float)motor_wrap_ * std::fabs(duty) / 100.0f);
        if (duty >= 0.0f) {
            pwm_set_chan_level(slice, PWM_CHAN_A, level);
            pwm_set_chan_level(slice, PWM_CHAN_B, 0);
        } else {
            pwm_set_chan_level(slice, PWM_CHAN_A, 0);
            pwm_set_chan_level(slice, PWM_CHAN_B, level);
        }
    };

    set_drive(slice_L_, duty_l_);
    set_drive(slice_R_, duty_r_);

    // 吸引モーター: 単方向 (CHAN_A のみ)
    uint16_t suction_level =
        (uint16_t)((float)motor_wrap_ * std::clamp(duty_suction_, 0.0f, 100.0f) / 100.0f);
    pwm_set_chan_level(slice_S_, PWM_CHAN_A, suction_level);
    pwm_set_chan_level(slice_S_, PWM_CHAN_B, 0);
}
