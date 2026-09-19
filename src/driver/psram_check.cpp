#include "driver/psram_check.hpp"
#include "hardware/gpio.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/xip.h"
#include "hardware/sync.h"
#include "pico/platform.h"
extern "C" {
#include "sfe_psram.h"
}
#include <stdio.h>

namespace psram_check {

namespace {

bool g_boot_ok = true; // 未実施の間は警告を出さない

// direct mode 中は XIP(flash) へのアクセスがバスエラーになる。以下の
// *_direct() は RAM 常駐で、レジスタ直叩きのみ(flash 上の関数を呼ばない)。
// 待ちも busy_wait_us() ではなく空ループで行う。
#define PSRAM_CHECK_SPIN(n)                                                    \
  for (volatile int spin_i_ = 0; spin_i_ < (n); spin_i_ = spin_i_ + 1) {       \
  }

void __no_inline_not_in_flash_func(direct_begin)() {
  qmi_hw->direct_csr = 30 << QMI_DIRECT_CSR_CLKDIV_LSB | QMI_DIRECT_CSR_EN_BITS;
  // 直前の XIP 転送のクールダウン明けを待つ(sfe_psram.c と同じ)
  while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
  }
}

// CS1 を1回アサートして1バイト送る。quad=true は QPI 幅(Exit QPI 用)。
void __no_inline_not_in_flash_func(direct_cmd)(uint8_t cmd, bool quad) {
  qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
  qmi_hw->direct_tx =
      quad ? (QMI_DIRECT_TX_OE_BITS |
              QMI_DIRECT_TX_IWIDTH_VALUE_Q << QMI_DIRECT_TX_IWIDTH_LSB | cmd)
           : cmd;
  while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
  }
  (void)qmi_hw->direct_rx;
  qmi_hw->direct_csr &= ~QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
  PSRAM_CHECK_SPIN(50);
}

void __no_inline_not_in_flash_func(probe_direct)(uint cs_pin, ProbeResult *out) {
  const uint32_t intr = save_and_disable_interrupts();
  direct_begin();

  // CS 線の固着チェック(クロックなしの CS パルス。PSRAM には無害)
  out->cs_idle = (uint8_t)((sio_hw->gpio_in >> cs_pin) & 1u);
  qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
  PSRAM_CHECK_SPIN(50);
  out->cs_assert = (uint8_t)((sio_hw->gpio_in >> cs_pin) & 1u);
  qmi_hw->direct_csr &= ~QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
  PSRAM_CHECK_SPIN(50);

  direct_cmd(0xF5, true); // QPI モードに残っている場合に備えて Exit QPI

  // Read ID (0x9F) — SPI モード、全バイトを記録
  qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
  for (size_t i = 0; i < sizeof(out->id); i++) {
    qmi_hw->direct_tx = (i == 0) ? 0x9Fu : 0xFFu;
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_TXEMPTY_BITS) == 0) {
    }
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
    }
    out->id[i] = (uint8_t)qmi_hw->direct_rx;
  }
  qmi_hw->direct_csr &=
      ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS | QMI_DIRECT_CSR_EN_BITS);

  restore_interrupts(intr);
}

// CS ピンを素の SIO として 入力→Low→High と振って読み返す。
//
// [重要] CE# が Low の間に QSPI のクロックを1発も出してはならない。PSRAM は
// flash と SD0〜3/SCK を共有しており、CE# が Low なら flash 向けの XIP
// トラフィックを自分宛のコマンドとして解釈し、読み出し系に見えた時点で
// SD 線を駆動して flash の命令フェッチを壊す(→ HardFault で無言 hang)。
// 2026-09-20: この処理を flash 上の gpio_init()/busy_wait_us()/gpio_put()
// で行っていたため、CE# が正常に Low へ落ちる基板で step6a hang した
// (外付けプルアップが無い基板では、入力へ切り替えただけで内蔵プルダウンに
// より CE# が Low になる点にも注意: sio_in=0)。
// そのため RAM 常駐 + 割り込み禁止 + レジスタ直叩きで行い、CE# を High へ
// 戻してから抜ける。DMA(DShot) は SRAM しか読まないので XIP を起こさない。
// Core1 起動後は Core1 が flash を実行するため呼んではならない。
void __no_inline_not_in_flash_func(sio_toggle_ram)(uint cs_pin,
                                                   ProbeResult *out) {
  out->sio_in = out->sio_low = out->sio_high = 0xFF; // 未測定
  if (cs_pin >= 32u)
    return; // GPIO47 は gpio_hi_* 側。現状の基板では使わない
  const uint32_t mask = 1u << cs_pin;
  const uint32_t intr = save_and_disable_interrupts();

  sio_hw->gpio_oe_clr = mask;
  io_bank0_hw->io[cs_pin].ctrl = GPIO_FUNC_SIO
                                 << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
  PSRAM_CHECK_SPIN(400); // >=20us
  out->sio_in = (uint8_t)((sio_hw->gpio_in >> cs_pin) & 1u);

  sio_hw->gpio_clr = mask;
  sio_hw->gpio_oe_set = mask;
  PSRAM_CHECK_SPIN(400);
  out->sio_low = (uint8_t)((sio_hw->gpio_in >> cs_pin) & 1u);

  sio_hw->gpio_set = mask;
  PSRAM_CHECK_SPIN(400);
  out->sio_high = (uint8_t)((sio_hw->gpio_in >> cs_pin) & 1u);

  // SIO で High 駆動中に XIP_CS1(アイドル=High)へ戻すのでグリッチしない
  io_bank0_hw->io[cs_pin].ctrl = GPIO_FUNC_XIP_CS1
                                 << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
  sio_hw->gpio_oe_clr = mask;
  PSRAM_CHECK_SPIN(50);

  restore_interrupts(intr);
}

void __no_inline_not_in_flash_func(qpi_enable_direct)() {
  const uint32_t intr = save_and_disable_interrupts();
  direct_begin();
  direct_cmd(0xF5, true);  // Exit QPI(既に QPI なら以降の SPI 幅コマンドが通らない)
  direct_cmd(0x66, false); // RSTEN
  direct_cmd(0x99, false); // RST
  PSRAM_CHECK_SPIN(3000);  // リセット完了待ち(>=50us。150MHz で 100us 強)
  direct_cmd(0x35, false); // QPI enable
  qmi_hw->direct_csr &=
      ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS | QMI_DIRECT_CSR_EN_BITS);
  restore_interrupts(intr);
}

// offset だけで決まる擬似乱数パターン。ブロック同士が重なっても同じ offset
// には同じ値が入るので矛盾しない。inv パスで全ビットが 0/1 両方を取る。
inline uint32_t pattern(uint32_t off, bool inv) {
  uint32_t v = (off + 0x9E3779B9u) * 2654435761u;
  v ^= v >> 15;
  return inv ? ~v : v;
}

inline uint8_t byte_pattern(uint32_t i, bool inv) {
  const uint8_t v = (uint8_t)(i * 37u + 0x5Bu);
  return inv ? (uint8_t)~v : v;
}

// 戻り値: 不一致だったら true
bool record(RwResult &r, uint32_t addr, uint32_t exp, uint32_t got, int nbytes) {
  r.words_tested++;
  if (exp == got)
    return false;
  r.mismatches++;
  r.err_mask |= exp ^ got;
  bool residue = true;
  for (int b = 0; b < nbytes; b++) {
    const uint8_t x = (uint8_t)(got >> (8 * b));
    if ((x >> 4) != (x & 0x0Fu))
      residue = false;
  }
  if (residue)
    r.residue_like++;
  if (r.n_first < RwResult::kFirstMax) {
    r.first_addr[r.n_first] = addr;
    r.first_exp[r.n_first] = exp;
    r.first_got[r.n_first] = got;
    r.n_first++;
  }
  return true;
}

// got が「off と 2^k(1/2/4MB) の整数倍だけ離れた別番地」のパターンか。
bool is_alias_value(uint32_t off, uint32_t size, uint32_t got, bool inv) {
  for (uint32_t k = 20; k <= 22; k++) {
    const uint32_t span = 1u << k;
    for (uint32_t cand = off & (span - 1u); cand + 4u <= size; cand += span) {
      if (cand != off && pattern(cand, inv) == got)
        return true;
    }
  }
  return false;
}

constexpr uint32_t kBlockStride = 64u * 1024u;
constexpr uint32_t kBlockWords = 16u; // 64 byte
// 8bit アクセスの確認範囲(ログエントリは 250B で、語境界に揃わない
// バイト/ハーフワード書き込みが実際に発生する)。わざと奇数アドレスから。
constexpr uint32_t kByteOff = 0x21u;
constexpr uint32_t kByteLen = 61u;

// テスト対象の (offset, 語数) を列挙する。
//   - 64KB おきの 64byte ブロック: 全域に散らす(サイズ誤認/エイリアス検出)
//   - 2 のべき乗オフセット: アドレスビットごと
//   - 終端 64byte: 最上位アドレス
template <typename Fn> void for_each_region(uint32_t size, Fn fn) {
  for (uint32_t off = 0; off + kBlockWords * 4u <= size; off += kBlockStride)
    fn(off, kBlockWords);
  for (uint32_t off = 4u; off + 4u <= size; off <<= 1)
    fn(off, 1u);
  if (size >= kBlockWords * 4u)
    fn((size - kBlockWords * 4u) & ~3u, kBlockWords);
}

} // namespace

void probe(uint cs_pin, ProbeResult *out) {
  // パッド設定(IE 有効 / ISO 解除)はここで済ませ、CS アイドル=High 駆動に
  // しておく。以降の機能切り替えは RAM 側で funcsel だけを書き換える。
  gpio_set_function(cs_pin, GPIO_FUNC_XIP_CS1);
  sio_toggle_ram(cs_pin, out);
  probe_direct(cs_pin, out);
}

void print_probe(const char *tag, const ProbeResult &p) {
  printf("[psram-probe:%s] cs_idle=%u(exp 1) cs_assert=%u(exp 0) "
         "id=%02X %02X %02X %02X %02X %02X %02X %02X (exp .. .. .. .. 0D 5D)\n",
         tag, p.cs_idle, p.cs_assert, p.id[0], p.id[1], p.id[2], p.id[3],
         p.id[4], p.id[5], p.id[6], p.id[7]);
  printf("[psram-probe:%s] CS pin as SIO: in=%u low=%u(exp 0) high=%u(exp 1)\n",
         tag, p.sio_in, p.sio_low, p.sio_high);
}

void force_qpi_init(uint cs_pin) {
  gpio_set_function(cs_pin, GPIO_FUNC_XIP_CS1);
  qpi_enable_direct();
  sfe_psram_update_timing();

  // 以下は sfe_psram.c の setup_psram() / LoggingTask::init() と同じ設定。
  // rfmt: QPI Quad Read (0xEB), 24 ダミークロック / wfmt: QPI Quad Write (0x38)
  qmi_hw->m[1].rfmt =
      (QMI_M1_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M1_RFMT_PREFIX_WIDTH_LSB) |
      (QMI_M1_RFMT_ADDR_WIDTH_VALUE_Q << QMI_M1_RFMT_ADDR_WIDTH_LSB) |
      (QMI_M1_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M1_RFMT_SUFFIX_WIDTH_LSB) |
      (QMI_M1_RFMT_DUMMY_WIDTH_VALUE_Q << QMI_M1_RFMT_DUMMY_WIDTH_LSB) |
      (QMI_M1_RFMT_DUMMY_LEN_VALUE_24 << QMI_M1_RFMT_DUMMY_LEN_LSB) |
      (QMI_M1_RFMT_DATA_WIDTH_VALUE_Q << QMI_M1_RFMT_DATA_WIDTH_LSB) |
      (QMI_M1_RFMT_PREFIX_LEN_VALUE_8 << QMI_M1_RFMT_PREFIX_LEN_LSB) |
      (QMI_M1_RFMT_SUFFIX_LEN_VALUE_NONE << QMI_M1_RFMT_SUFFIX_LEN_LSB);
  qmi_hw->m[1].rcmd = 0xEBu << QMI_M1_RCMD_PREFIX_LSB;
  qmi_hw->m[1].wfmt =
      (QMI_M1_WFMT_PREFIX_WIDTH_VALUE_Q << QMI_M1_WFMT_PREFIX_WIDTH_LSB) |
      (QMI_M1_WFMT_ADDR_WIDTH_VALUE_Q << QMI_M1_WFMT_ADDR_WIDTH_LSB) |
      (QMI_M1_WFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M1_WFMT_SUFFIX_WIDTH_LSB) |
      (QMI_M1_WFMT_DUMMY_WIDTH_VALUE_Q << QMI_M1_WFMT_DUMMY_WIDTH_LSB) |
      (QMI_M1_WFMT_DUMMY_LEN_VALUE_NONE << QMI_M1_WFMT_DUMMY_LEN_LSB) |
      (QMI_M1_WFMT_DATA_WIDTH_VALUE_Q << QMI_M1_WFMT_DATA_WIDTH_LSB) |
      (QMI_M1_WFMT_PREFIX_LEN_VALUE_8 << QMI_M1_WFMT_PREFIX_LEN_LSB) |
      (QMI_M1_WFMT_SUFFIX_LEN_VALUE_NONE << QMI_M1_WFMT_SUFFIX_LEN_LSB);
  qmi_hw->m[1].wcmd = 0x38u << QMI_M1_WCMD_PREFIX_LSB;
  hw_set_bits(&xip_ctrl_hw->ctrl, XIP_CTRL_WRITABLE_M1_BITS);
}

RwResult rw_test(uintptr_t base, size_t size) {
  RwResult r;
  volatile uint32_t *const w = reinterpret_cast<volatile uint32_t *>(base);
  volatile uint8_t *const b = reinterpret_cast<volatile uint8_t *>(base);
  const uint32_t sz = (uint32_t)size;

  // 「全部書く → 全部読む」を分ける。書いた直後に同じ番地を読むと、無応答
  // チップでもバスに残った値で偶然一致しうる。分けておけばエイリアス
  // (容量誤認で上位アドレスが下位へ折り返す)も検出できる。
  for (int pass = 0; pass < 2; pass++) {
    const bool inv = (pass == 1);
    for_each_region(sz, [&](uint32_t off, uint32_t n) {
      for (uint32_t i = 0; i < n; i++)
        w[(off >> 2) + i] = pattern(off + i * 4u, inv);
    });
    for_each_region(sz, [&](uint32_t off, uint32_t n) {
      for (uint32_t i = 0; i < n; i++) {
        const uint32_t o = off + i * 4u;
        const uint32_t got = w[o >> 2];
        if (record(r, (uint32_t)base + o, pattern(o, inv), got, 4) &&
            is_alias_value(o, sz, got, inv))
          r.alias_like++;
      }
    });
  }

  if (sz >= kByteOff + kByteLen) {
    for (int pass = 0; pass < 2; pass++) {
      const bool inv = (pass == 1);
      for (uint32_t i = 0; i < kByteLen; i++)
        b[kByteOff + i] = byte_pattern(i, inv);
      for (uint32_t i = 0; i < kByteLen; i++)
        record(r, (uint32_t)base + kByteOff + i, byte_pattern(i, inv),
               b[kByteOff + i], 1);
    }
  }
  return r;
}

void print_rw(const char *tag, const RwResult &r) {
  if (r.pass()) {
    printf("[psram-rw:%s] PASS  %lu compares (write-all/read-all x2, "
           "pattern+inverse, 8bit access incl.)\n",
           tag, (unsigned long)r.words_tested);
    return;
  }
  printf("[psram-rw:%s] FAIL  mismatches=%lu/%lu err_mask=0x%08lX "
         "residue_like=%lu alias_like=%lu\n",
         tag, (unsigned long)r.mismatches, (unsigned long)r.words_tested,
         (unsigned long)r.err_mask, (unsigned long)r.residue_like,
         (unsigned long)r.alias_like);
  for (int i = 0; i < r.n_first; i++) {
    printf("[psram-rw:%s]   @0x%08lX exp=0x%08lX got=0x%08lX\n", tag,
           (unsigned long)r.first_addr[i], (unsigned long)r.first_exp[i],
           (unsigned long)r.first_got[i]);
  }

  // QPI では各バイトが 上位ニブル→下位ニブル の順で SD3..SD0 に載るので、
  // データビット (4k+n) は全て SDn を通る。エラービットを1ニブルへ畳めば
  // どの線が怪しいかが出る。
  uint32_t lines = 0;
  for (int i = 0; i < 8; i++)
    lines |= (r.err_mask >> (4 * i)) & 0xFu;

  if (r.residue_like * 10u >= r.mismatches * 9u &&
      r.mismatches * 10u >= r.words_tested * 9u) {
    printf("[psram-rw:%s]   verdict: chip is NOT driving the bus (reads are "
           "QSPI bus residue). CE#/power/QPI-mode, not data corruption\n",
           tag);
  } else if (r.alias_like * 10u >= r.mismatches * 8u) {
    printf("[psram-rw:%s]   verdict: chip responds, but addresses alias "
           "(actual capacity is smaller than the tested size)\n",
           tag);
  } else if (lines != 0 && (lines & (lines - 1u)) == 0) {
    int n = 0;
    while (!((lines >> n) & 1u))
      n++;
    printf("[psram-rw:%s]   verdict: chip responds, all errors are on SD%d "
           "(stuck/open/bridged data line)\n",
           tag, n);
  } else {
    printf("[psram-rw:%s]   verdict: chip responds but data is corrupt on "
           "SD lines mask=0x%lX (signal integrity / power / timing)\n",
           tag, (unsigned long)lines);
  }
}

void set_boot_result(bool ok) { g_boot_ok = ok; }
bool boot_ok() { return g_boot_ok; }

} // namespace psram_check
