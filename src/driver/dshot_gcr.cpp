#include "driver/dshot_gcr.hpp"

namespace dshot {

namespace {

// nibble(0..15) -> 5bit GCRコード。主要ESCファームウェア(BLHeli32/KISS/AM32系)の
// bidirectional DShotテレメトリ実装で共通して使われている固定テーブル
// (遷移回数が制限された16個の5bitパターンを選んだもの)。
constexpr uint8_t kGcrEncodeTable[16] = {
    0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
    0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
};

// 5bitコード(0..31) -> nibble の逆引きテーブル。該当なしは0xFFで無効を示す。
struct GcrDecodeTable {
  uint8_t table[32];
  constexpr GcrDecodeTable() : table{} {
    for (int i = 0; i < 32; i++) table[i] = 0xFF;
    for (int n = 0; n < 16; n++) table[kGcrEncodeTable[n]] = (uint8_t)n;
  }
};
constexpr GcrDecodeTable kGcrDecodeTable;

inline uint8_t crc4(uint16_t value, bool invert) {
  uint8_t crc = (uint8_t)((value ^ (value >> 4) ^ (value >> 8)) & 0x0F);
  if (invert) crc = (uint8_t)(~crc & 0x0F);
  return crc;
}

} // namespace

uint16_t throttle_pct_to_value(float throttle_pct) {
  if (throttle_pct <= 0.0f) return kCmdMotorStop;
  if (throttle_pct > 100.0f) throttle_pct = 100.0f;
  const float span = (float)(kThrottleMax - kThrottleMin);
  uint16_t v = kThrottleMin + (uint16_t)(span * throttle_pct / 100.0f + 0.5f);
  if (v > kThrottleMax) v = kThrottleMax;
  return v;
}

uint16_t build_command_frame(uint16_t throttle, bool telemetry_bit, bool bidirectional) {
  const uint16_t value = (uint16_t)((throttle << 1) | (telemetry_bit ? 1 : 0));
  const uint8_t crc = crc4(value, bidirectional);
  return (uint16_t)((value << 4) | crc);
}

bool decode_telemetry_gcr(uint32_t raw20, uint32_t *period_us_out) {
  // raw20: MSB firstで受信した20bit(5bitシンボル*4)。上位から順に4シンボルへ分割する。
  uint8_t nibble[4];
  for (int i = 0; i < 4; i++) {
    const uint8_t sym = (uint8_t)((raw20 >> (15 - i * 5)) & 0x1F);
    const uint8_t n = kGcrDecodeTable.table[sym];
    if (n == 0xFF) return false; // 不正なGCRシンボル
    nibble[i] = n;
  }
  // whiteningの逆変換: raw[0]=dec[0], raw[i]=dec[i]^raw[i-1]
  uint8_t raw_nibble[4];
  raw_nibble[0] = nibble[0];
  for (int i = 1; i < 4; i++) raw_nibble[i] = (uint8_t)(nibble[i] ^ raw_nibble[i - 1]);

  const uint16_t value =
      (uint16_t)((raw_nibble[0] << 12) | (raw_nibble[1] << 8) | (raw_nibble[2] << 4) | raw_nibble[3]);
  const uint16_t period12 = (uint16_t)(value >> 4);
  const uint8_t rx_crc = (uint8_t)(value & 0x0F);
  if (crc4(period12, /*invert=*/true) != rx_crc) return false;

  if (period12 == 0x0FFF) {
    // 全ビット1 = モーター停止/データ無効を表す予約値
    if (period_us_out) *period_us_out = 0;
    return true;
  }
  const uint8_t exponent = (uint8_t)(period12 >> 9);
  const uint32_t mantissa = period12 & 0x01FF;
  if (period_us_out) *period_us_out = mantissa << exponent;
  return true;
}

float erpm_from_period_us(uint32_t period_us) {
  if (period_us == 0) return 0.0f;
  return 60000000.0f / (float)period_us;
}

} // namespace dshot
