#pragma once
#include <stdint.h>
#include <memory>
#include "sensing_task.hpp"

// 1kHz ハードウェアタイマー IRQ (TIMER1 alarm 0, Core0) で動作する
// planning / control タスク。
// モーター・吸引 PWM の初期化も本クラスが担う。
// Core0 の main ループから send_command() で目標値を投入し、
// IRQ 内で軌道生成・PID 制御・モーター出力を行う。
class PlanningTask {
public:
    enum class MotionMode : uint8_t {
        IDLE     = 0,  // モーター停止
        STRAIGHT = 1,  // 直進 (台形速度プロファイル)
        PIVOT    = 2,  // 超信地旋回 (台形角速度プロファイル)
        STOP     = 3,  // 現在速度から減速停止
    };

    struct Command {
        MotionMode mode    = MotionMode::IDLE;
        float v_max        = 0.0f;   // 目標速度   [mm/s]
        float v_end        = 0.0f;   // 終端速度   [mm/s]
        float accl         = 0.0f;   // 加速度     [mm/s^2]
        float decel        = 0.0f;   // 減速度     [mm/s^2]
        float dist         = 0.0f;   // 目標距離   [mm]
        float w_max        = 0.0f;   // 目標角速度 [rad/s]
        float alpha        = 0.0f;   // 角加速度   [rad/s^2]
        float ang          = 0.0f;   // 目標角度   [rad] (PIVOT 時)
        float duty_suction = 0.0f;   // 吸引モーター duty [%]
        uint32_t timestamp = 0;
    };

    // IRQ が毎 tick 更新する状態。Core0 main ループから読み取り可能。
    struct State {
        float img_v        = 0.0f;   // 指令速度     [mm/s]
        float img_w        = 0.0f;   // 指令角速度   [rad/s]
        float img_dist     = 0.0f;   // 累積走行距離 [mm]
        float img_ang      = 0.0f;   // 累積旋回角度 [rad]
        float v_est        = 0.0f;   // 推定速度     [mm/s]
        float w_est        = 0.0f;   // 推定角速度   [rad/s]
        float duty_l       = 0.0f;   // 左モーター duty [%]
        float duty_r       = 0.0f;   // 右モーター duty [%]
        float duty_suction = 0.0f;   // 吸引モーター duty [%]
        MotionMode mode    = MotionMode::IDLE;
        uint32_t tick      = 0;

        State() = default;
        State(const volatile State &o)
            : img_v(o.img_v), img_w(o.img_w)
            , img_dist(o.img_dist), img_ang(o.img_ang)
            , v_est(o.v_est), w_est(o.w_est)
            , duty_l(o.duty_l), duty_r(o.duty_r)
            , duty_suction(o.duty_suction)
            , mode(o.mode), tick(o.tick) {}
    };

    static std::shared_ptr<PlanningTask> create();

    // Core0 の main() から呼ぶ。
    // モーター/吸引 PWM の GPIO・スライス設定と TIMER1 IRQ 登録を行う。
    void init(std::shared_ptr<SensingTask> sensing);

    // Core0 の main ループから呼ぶ。IRQ-safe (内部で割り込み禁止区間)。
    void send_command(const Command &cmd);

    volatile State state{};

private:
    PlanningTask() = default;

    static void timer_irq_handler();

    void tick(uint32_t dt_us);
    void update_trajectory(float dt);
    void update_control(float dt);
    void apply_motor();

    static std::shared_ptr<PlanningTask> s_instance;

    std::shared_ptr<SensingTask> sensing_;

    uint32_t interval_us_ = 1000;
    uint32_t next_alarm_  = 0;
    uint64_t prev_ts_     = 0;

    // Core0 main → Core0 IRQ コマンドバッファ
    Command       pending_cmd_{};
    volatile bool cmd_pending_ = false;

    Command active_cmd_{};

    // 軌道生成 (台形速度プロファイル)
    float img_v_    = 0.0f;
    float img_w_    = 0.0f;
    float img_dist_ = 0.0f;
    float img_ang_  = 0.0f;

    // 推定値 (エンコーダ差分から)
    float    v_est_      = 0.0f;
    float    w_est_      = 0.0f;
    uint16_t enc_r_prev_ = 0;
    uint16_t enc_l_prev_ = 0;
    bool     first_tick_ = true;

    // PID 積分項
    float vel_err_i_  = 0.0f;
    float gyro_err_i_ = 0.0f;

    // モーター・吸引 PWM
    uint     slice_L_      = 0;
    uint     slice_R_      = 0;
    uint     slice_S_      = 0;   // 吸引モーター
    uint32_t motor_wrap_   = 2999;
    float    duty_l_       = 0.0f;
    float    duty_r_       = 0.0f;
    float    duty_suction_ = 0.0f;
};
