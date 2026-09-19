#pragma once
#include "pico/types.h"
#include <cstddef>
#include <cstdint>

// ============================================================
// PSRAM(APS6404L, QMI CS1) 起動時診断  (2026-09-19〜)
//
// sfe_setup_psram() は Read ID の KGD==0x5D 一発判定だけで、失敗の内訳を
// 一切残さない。またログが「形だけ正しいゴミ値」になっても気づけなかった
// (無応答チップは QSPI 線に残ったアドレスの残像を返す)。そこで
//   probe()   : Read ID の全バイトと CS 線の実レベル(間接証拠)
//   rw_test() : 実際に書いて読み戻す(直接証拠)
// の2段で、起動のたびに「本当に使えるか」を確かめる。
// ============================================================
namespace psram_check {

struct ProbeResult {
  uint8_t id[8];     // 0x9F 応答。期待: xx xx xx xx 0D 5D <EID> ..
  uint8_t cs_idle;   // QMI CS1n 非アサート時のパッドレベル (期待 1)
  uint8_t cs_assert; // QMI CS1n アサート時のパッドレベル   (期待 0)
  // QMI を介さず CS ピンを素の SIO として振った時の読み返し
  // (cs_assert の測り方自体の裏取り)。
  //   sio_low=1              → 線が物理的に Low へ落ちない(3V3短絡 or 出力段故障)
  //   sio_low=0, cs_assert=1 → 線は正常、QMI CS1 機能側の問題
  uint8_t sio_in;   // 入力(出力OFF)時。外付けプルアップがあれば 1
  uint8_t sio_low;  // SIO で Low 駆動中  (期待 0)
  uint8_t sio_high; // SIO で High 駆動中 (期待 1)
};

// cs_pin を一旦 SIO として振った後、XIP_CS1 機能へ切り替えて direct mode で
// Read ID する。戻った時点で cs_pin は XIP_CS1 機能(sfe_setup_psram と同じ)。
// CE# を意図的に Low へ振るため、その間 XIP(flash 実行)が一切起きない
// ことが前提: **Core1 起動前にのみ呼ぶこと**(詳細は .cpp の sio_toggle_ram)。
void probe(uint cs_pin, ProbeResult *out);
void print_probe(const char *tag, const ProbeResult &p);

// sfe_setup_psram() が not detected で return すると、QPI enable(0x35) も
// QMI M1 の rfmt/wfmt 設定も行われないため、その後の読み書きは「チップが
// 生きていても」必ず失敗する。rw_test() の結果を ID 判定から独立させるため、
// ID を見ずに RSTEN/RST/QPI enable と M1 設定を強制する。
void force_qpi_init(uint cs_pin);

struct RwResult {
  uint32_t words_tested = 0; // 比較した回数(32bit語 + 8bitアクセス分)
  uint32_t mismatches = 0;
  uint32_t err_mask = 0;     // (期待 ^ 実測) の全エラー OR
  // 実測値の全バイトが「同一ニブルの繰り返し」(0x00, 0x44, 0xCC …)だった回数。
  // チップがバスを駆動していない時に読めるアドレス残像の特徴
  // (ランダムパターンが偶然こうなる確率は 1/65536)。
  uint32_t residue_like = 0;
  // 実測値が「別の(2^k だけ離れた)番地へ書いたはずの値」だった回数。
  // 実容量が想定より小さく、上位アドレスが下位へ折り返している時の特徴。
  uint32_t alias_like = 0;
  static constexpr int kFirstMax = 4;
  int n_first = 0;
  uint32_t first_addr[kFirstMax] = {};
  uint32_t first_exp[kFirstMax] = {};
  uint32_t first_got[kFirstMax] = {};

  bool pass() const { return words_tested > 0 && mismatches == 0; }
};

// base〜base+size へ実際に書いて読み戻す。base はログが使うのと同じ
// 非キャッシュ窓(0x15000000)を渡すこと(キャッシュ窓だと XIP キャッシュ内で
// 完結してチップ無応答でも通ってしまう)。領域の内容は破壊される。
// 所要時間は数ms(64KB おきの 64byte ブロック + 2のべき乗オフセット + 終端)。
RwResult rw_test(uintptr_t base, size_t size);
void print_rw(const char *tag, const RwResult &r);

// main() の起動時チェック結果。MainTask が UI 初期化後に警告音を出すために
// 参照する(main() の時点ではブザー未初期化)。
void set_boot_result(bool ok);
bool boot_ok();

} // namespace psram_check
