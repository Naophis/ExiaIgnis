#pragma once
#include <stdint.h>
#include <memory>
#include "sensing_task.hpp"
#include "planning/astraea_types.hpp"

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

        Command() = default;
        // volatile Command からのコピー (クロスコア受け取り用)
        Command(const volatile Command &o)
            : mode(o.mode), v_max(o.v_max), v_end(o.v_end)
            , accl(o.accl), decel(o.decel), dist(o.dist)
            , w_max(o.w_max), alpha(o.alpha), ang(o.ang)
            , duty_suction(o.duty_suction), timestamp(o.timestamp) {}
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

    // Core0 の main() から呼ぶ。モーター/吸引 PWM の GPIO・スライス設定のみ。
    void init(std::shared_ptr<SensingTask> sensing);

    // Core1 エントリから呼ぶ。TIMER1 IRQ を Core1 に登録して計測ループを開始する。
    void start_irq();

    // Core0 の MainTask から呼ぶ (IRQ は Core1 側)。__dmb() で cross-core 安全。
    // void send_command(const Command &cmd);

    // Astraea 互換: motion_tgt_val_t ポインタを渡して次の IRQ tick で cp_request() を実行。
    // xTaskNotify(*th, (uint32_t)tgt_val.get(), ...) に相当。
    void send_command(std::shared_ptr<motion_tgt_val_t> tgt);

    volatile State state{};

    // ---- Astraea MotionPlanning 互換インターフェース ----
    float last_tgt_angle = 0.0f;

    void motor_enable()  {}  // TODO: モーター有効化
    void motor_disable() {   // IDLE コマンドでモーター停止
        Command cmd; cmd.mode = MotionMode::IDLE;
        // send_command(cmd);
    }
    void suction_enable(float duty, float /*duty_low*/) {
        Command cmd; cmd.mode = MotionMode::IDLE; cmd.duty_suction = duty;
        // send_command(cmd);
    }
    void suction_disable() {
        Command cmd; cmd.mode = MotionMode::IDLE; cmd.duty_suction = 0.0f;
        // send_command(cmd);
    }
    void reset_kf_state(bool /*full*/) {}  // TODO: カルマンフィルタリセット
    void reset_pos(float /*x*/, float /*y*/, float /*ang*/) {}  // TODO: 位置リセット

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

    // Core1 main_task → Core0 IRQ コマンドバッファ (Command 形式)
    // __dmb() + volatile でクロスコア可視性を保証
    volatile Command pending_cmd_{};
    volatile bool    cmd_pending_ = false;

    // Astraea 互換: motion_tgt_val_t ベースのコマンドバッファ
    std::shared_ptr<motion_tgt_val_t> pending_tgt_;
    volatile bool tgt_cmd_pending_ = false;
    int32_t last_nmr_timestamp_   = -1;

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
