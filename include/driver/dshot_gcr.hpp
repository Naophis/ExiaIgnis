#pragma once
#include <stdint.h>

// Bidirectional DShot(inverted DShot)のフレーミング計算(CRC / GCR)を
// 担う、ハードウェア非依存の純粋関数群。
//
// [重要] このファイルの内容は公開されているDShot/bidirectional DShot仕様
// (KISS/BLHeli32/AM32/ESCape32等、主要ESCファームウェアが共通して実装している
// デファクトスタンダード)を基に実装したもので、実機(ロジックアナライザでの
// 波形確認)による検証はまだ行っていない。特に GCR テーブル・whitening順序・
// テレメトリ側のbit period比率(5/4倍)は、実機のESCape32を用いた波形キャプチャ
// で必ず突き合わせてから信頼すること。
//
// 参考にした仕様の要点:
//   - コマンドフレーム: 16bit = throttle(11bit) + telemetry要求bit(1bit) + CRC(4bit)、MSB first。
//     value(12bit) = (throttle<<1)|telem_bit
//     通常DShot:      crc =  (value ^ (value>>4) ^ (value>>8)) & 0xF
//     bidirectional:  crc = ~(value ^ (value>>4) ^ (value>>8)) & 0xF  (CRCを反転する)
//   - テレメトリ(eRPM)フレーム: 16bit値(period 12bit + crc 4bit、CRC計算式はコマンド側と同じ
//     bidirectional反転版)を4nibbleに分割し、各nibbleを直前nibbleとXORする(whitening、
//     先頭nibbleはそのまま)。whitening後の各nibbleを4bit->5bit GCRテーブルで符号化し、
//     20bit(5bit*4)として送出する。受信側はこの逆手順で復元する。
//   - period 12bit = exponent(上位3bit) : mantissa(下位9bit)。
//     period_base_us = mantissa << exponent  (基準単位は1us)。
//     eRPM(electrical RPM) = 60,000,000 / period_base_us (period=0は無効値として扱う)。
//     実RPM = eRPM / (モーター極対数)。極対数はモーター依存のためこのファイルでは扱わない。
namespace dshot {

// 1〜47はスロットル値ではなく特殊コマンド(DSHOT_CMD_*)用に予約されている。
constexpr uint16_t kThrottleMin = 48;    // 実スロットル0%
constexpr uint16_t kThrottleMax = 2047;  // 実スロットル100%
constexpr uint16_t kCmdMotorStop = 0;

// throttle: 0(=DSHOT_CMD_MOTOR_STOP)、または48〜2047。telemetry_bit: このフレームで
// ESCにテレメトリ応答を要求するか(bidirectionalでは通常常時1)。
// bidirectional: true の場合CRCを反転する(上記コメント参照)。
// 戻り値: MSB firstで送出すべき16bitフレーム(bit15が最初に出るビット)。
uint16_t build_command_frame(uint16_t throttle, bool telemetry_bit, bool bidirectional);

// throttle_pct: 0.0〜100.0 を kThrottleMin〜kThrottleMax へ線形変換する。
uint16_t throttle_pct_to_value(float throttle_pct);

// テレメトリ応答の生20bit(GCR符号化されたままの値、MSB firstでキャプチャした値を
// そのままuint32_tの下位20bitに詰めたもの)をデコードする。
// 戻り値: true=CRC OK(period_us_outに基準周期[us]を書く。period_us==0はモーター停止/無効を表す
// 特殊値なのでそのまま0を返す)、false=GCRシンボル不正 or CRC不一致。
bool decode_telemetry_gcr(uint32_t raw20, uint32_t *period_us_out);

// period_us(decode_telemetry_gcr()のperiod_us_out)からeRPMへ変換する。period_us==0は0を返す。
float erpm_from_period_us(uint32_t period_us);

} // namespace dshot
