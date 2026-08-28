#pragma once
#include "driver/dshot_bidir.hpp"
#include "hardware/pio.h"

// 吸引モーター用ESC(ESCape32想定)への bidirectional DShot600 出力。
// SuctionEscActuator(AM32 ESC向け、標準PWM)と同じ役割の代替実装だが、
// 現状どこからも呼ばれていない(ControlLaw/PlanningTaskへの結線は未実施)。
// AM32から実際にESCape32へ載せ替えるまではSuctionEscActuatorを使い続ける、
// という運用方針のため([[project_escape32_dshot_migration]]参照)。
//
// SuctionEscActuatorと異なり、DShotはハードウェアPWMのような自律出力を
// 持たないプロトコルのため、フレームを周期的に送信し続ける必要がある。
// tick()を1kHz程度の一定周期(例えばControlLawの制御周期と同じ)で
// 呼び出し続けることを前提とする。呼び出しが途切れるとESC側がフェイル
// セーフ(モーター停止)に入る。
class SuctionEscDshotActuator {
public:
    // pio: 専有するPIOブロック。gpio: ESCのS信号線につながるGPIO。
    // 戻り値false: PIO資源不足などでの初期化失敗。
    bool init(PIO pio, uint gpio);

    // duty_pct(0..100)をDShotフレームとして送出し、テレメトリ応答があれば
    // last_erpm_/last_telemetry_valid_を更新する。enable()前や失敗時は
    // モーター停止コマンド(duty=0)を送る。
    void tick(float duty_pct);

    void enable()  { enabled_ = true; }
    void disable() { enabled_ = false; }
    bool is_enabled() const { return enabled_; }

    // 直近のテレメトリ応答から得たeRPM(電気的RPM、極対数で割れば実RPM)。
    // last_telemetry_valid()がfalseの間は直近の有効値を保持したままにする
    // (呼び出し側が「更新が止まった」ことを検知できるよう、0クリアはしない)。
    float last_erpm() const { return last_erpm_; }
    bool  last_telemetry_valid() const { return last_telemetry_valid_; }

private:
    DshotBidir dshot_;
    bool  initialized_           = false;
    bool  enabled_                = false;
    float last_erpm_              = 0.0f;
    bool  last_telemetry_valid_   = false;
};
