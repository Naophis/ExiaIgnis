#include "planning/suction_esc_dshot_actuator.hpp"
#include "driver/dshot_gcr.hpp"

bool SuctionEscDshotActuator::init(PIO pio, uint gpio) {
    initialized_ = dshot_.init(pio, gpio, /*bitrate_hz=*/600000);
    return initialized_;
}

void SuctionEscDshotActuator::tick(float duty_pct) {
    if (!initialized_) return;

    const float pct = enabled_ ? duty_pct : 0.0f;
    // request_telemetry=true固定(bidirectional運用ではESC側の3Dモード等と
    // 無関係に毎フレームeRPM応答を要求する運用が一般的)。
    if (!dshot_.send_throttle(pct, /*request_telemetry=*/true)) {
        return;
    }

    uint32_t period_us = 0;
    if (dshot_.receive_telemetry(&period_us)) {
        last_erpm_ = dshot::erpm_from_period_us(period_us);
        last_telemetry_valid_ = true;
    } else {
        // 応答なし(ESC未接続/未対応/フレーム化け)。直近値は保持したまま
        // valid フラグだけ落とし、呼び出し側が「テレメトリ途絶」を検知できる
        // ようにする(値を握りつぶして0にしない)。
        last_telemetry_valid_ = false;
    }
}
