#pragma once
#include <stdint.h>
#include <stddef.h>
#include "hardware/pio.h"
#include "driver/am32_protocol.hpp"

// AM32 ESC 設定構造体(EEPROM上のレイアウト)。
//
// 出典: AlkaMotors/AM32 (AM32-Firmware) Inc/eeprom.h の `EEprom_u` を
// フィールド名・順序・バイト幅とも1:1で移植したもの。offsetは
// am32-configurator/src/eeprom.ts の EepromLayout(全項目)と突き合わせて
// 一致を確認済み(調査レポート参照)。
//
// [重要] AM32公式の設定構造体には "Demag Compensation"(逆起電力補償)に
// 相当する独立した項目は存在しない(BLHeli_S等とは異なる)。近い概念として
// stuck_rotor_protection(ロック検出) と comp_pwm(complementary PWM、
// ローサイドの同期整流駆動有無)があるが、いずれもdemag compensationその
// ものではない。存在しない設定を捏造しないため、本実装ではdemag専用の
// フィールド/APIを提供しない。
#pragma pack(push, 1)
struct Am32EepromLayout {
    uint8_t reserved_0;             // 0  = BOOT_BYTE。bootloaderがjump可否判定に使う(0x01で有効)
    uint8_t eeprom_version;         // 1  出典: version.h EEPROM_VERSION (現行ファームウェアでは4)
    uint8_t reserved_1;             // 2  bootloaderが自身のBOOTLOADER_VERSIONで上書きするフィールド
    struct {
        uint8_t major;               // 3
        uint8_t minor;               // 4
    } version;
    uint8_t max_ramp;                        // 5   0.1%/ms 〜 25%/ms
    uint8_t minimum_duty_cycle;              // 6   0.2% 〜 51%
    uint8_t disable_stick_calibration;       // 7
    uint8_t absolute_voltage_cutoff;         // 8   1〜100 (0.5V刻み)
    uint8_t current_P;                       // 9
    uint8_t current_I;                       // 10
    uint8_t current_D;                       // 11
    uint8_t active_brake_power;              // 12  1〜5% duty
    uint8_t brake_on_zero_throttle;          // 13
    uint8_t reserved_eeprom_3[3];            // 14-16 (eeprom_version<3では旧ファーム名文字列だった領域)
    uint8_t dir_reversed;                    // 17
    uint8_t bi_direction;                    // 18  3Dモード
    uint8_t use_sine_start;                  // 19
    uint8_t comp_pwm;                        // 20  complementary PWM (demag compではない)
    uint8_t variable_pwm;                    // 21  fw<2.18: bool / fw>=2.18: 0=Fixed,1=Variable,2=byRPM
    uint8_t stuck_rotor_protection;          // 22
    uint8_t advance_level;                   // 23  timing。eeprom_version>=3: 10-42(スライダー表示は-10した0-32)
    uint8_t pwm_frequency;                   // 24  kHz直値
    uint8_t startup_power;                   // 25  50〜150 (%)
    uint8_t motor_kv;                        // 26  実KV = raw*40+20
    uint8_t motor_poles;                     // 27
    uint8_t brake_on_stop;                   // 28
    uint8_t stall_protection;                // 29
    uint8_t beep_volume;                     // 30  0〜11
    uint8_t telemetry_on_interval;           // 31
    struct {
        uint8_t low_threshold;   // 32  us = raw*2+750
        uint8_t high_threshold;  // 33  us = raw*2+1750
        uint8_t neutral;         // 34  us = raw+1374
        uint8_t dead_band;       // 35
    } servo;
    uint8_t low_voltage_cut_off;             // 36
    uint8_t low_cell_volt_cutoff;            // 37  V/cell = raw+250 (centivolt, 2.5〜3.5V)
    uint8_t rc_car_reverse;                  // 38  有効時、多数の項目を強制上書きする(ファームウェア側挙動)
    uint8_t use_hall_sensors;                // 39
    uint8_t sine_mode_changeover_thottle_level; // 40  5〜25%
    uint8_t drag_brake_strength;             // 41  1〜10
    uint8_t driving_brake_strength;          // 42  1〜10
    struct {
        uint8_t temperature; // 43  raw=℃直値。70〜140が有効、141以上は無効
        uint8_t current;     // 44  Amps = raw*2。202は「無効」sentinel
    } limits;
    uint8_t sine_mode_power;                 // 45  1〜10
    uint8_t input_type;                      // 46  AUTO=0,DSHOT=1,SERVO=2,SERIAL=3,EDTARM=4,DRONECAN=5
    uint8_t auto_advance;                    // 47  要fw>=2.16
    uint8_t tune[128];                       // 48-175 起動音(RTTTL/Bluejay形式)
    struct {
        uint8_t can_node;             // 176
        uint8_t esc_index;            // 177
        uint8_t require_arming;       // 178
        uint8_t telem_rate;           // 179
        uint8_t require_zero_throttle;// 180
        uint8_t filter_hz;            // 181
        uint8_t debug_rate;           // 182
        uint8_t term_enable;          // 183
        uint8_t reserved[8];          // 184-191 (Configuratorも読み書きしない領域)
    } can;
};
#pragma pack(pop)
static_assert(sizeof(Am32EepromLayout) == 192, "Am32EepromLayout must be exactly 192 bytes");

union Am32Eeprom {
    Am32EepromLayout f;
    uint8_t buffer[192];
};
static_assert(sizeof(Am32Eeprom) == 192, "Am32Eeprom union size mismatch");

namespace am32_detail {
inline int clamp_i(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace am32_detail

// AM32設定への高レベルアクセサ。生の192byte構造体(raw)を常に保持しつつ、
// 依頼にあった代表的な項目には人間の単位での get/set を用意する。
// raw.f.* で構造体の全フィールドに直接アクセス可能(未対応項目もここから読み書きできる)。
struct AM32Settings {
    // 出典: am32-configurator/src/mcu.ts `Mcu.LAYOUT_SIZE = 0xB8`。
    // 192byte構造体のうち、Configuratorが実際にESCと送受信するのは先頭184byteのみ
    // (末尾8byte = can.reserved[8] は対象外)。read/writeともこの長さを使う。
    static constexpr size_t LAYOUT_SIZE = 0xB8;  // 184

    Am32Eeprom raw{};

    uint8_t eeprom_version() const { return raw.f.eeprom_version; }
    uint8_t firmware_major() const { return raw.f.version.major; }
    uint8_t firmware_minor() const { return raw.f.version.minor; }

    uint16_t motor_kv() const { return (uint16_t)(raw.f.motor_kv * 40 + 20); }
    void set_motor_kv(uint16_t kv) {
        const int v = (kv >= 20) ? (int)((kv - 20) / 40) : 0;
        raw.f.motor_kv = (uint8_t)am32_detail::clamp_i(v, 0, 255);
    }

    uint8_t motor_poles() const { return raw.f.motor_poles; }
    void set_motor_poles(uint8_t poles) { raw.f.motor_poles = poles; }

    // timing: AM32 Configiratorのスライダー表示値と同じ0〜32単位(実機格納値=これ+10)。
    // eeprom_version<3の旧フォーマット(0-3, 7.5deg刻み)への変換は未対応(出典未確認のため非実装)。
    uint8_t timing() const { return raw.f.advance_level >= 10 ? (uint8_t)(raw.f.advance_level - 10) : 0; }
    void set_timing(uint8_t slider_0_32) {
        raw.f.advance_level = (uint8_t)(am32_detail::clamp_i(slider_0_32, 0, 32) + 10);
    }
    float timing_degrees() const { return timing() * 0.9375f; }

    uint8_t pwm_frequency_khz() const { return raw.f.pwm_frequency; }
    void set_pwm_frequency_khz(uint8_t khz) { raw.f.pwm_frequency = khz; }

    // fw<2.18ではbool、fw>=2.18では0/1/2のenum(0=Fixed,1=Variable,2=byRPM)。
    // ここでは「0=無効／0以外=有効」というbool的な入出力に統一する(両世代と互換)。
    bool variable_pwm_enabled() const { return raw.f.variable_pwm != 0; }
    void set_variable_pwm_enabled(bool en) { raw.f.variable_pwm = en ? 1 : 0; }

    uint8_t startup_power_percent() const { return raw.f.startup_power; }
    void set_startup_power_percent(uint8_t pct) {
        raw.f.startup_power = (uint8_t)am32_detail::clamp_i(pct, 50, 150);
    }

    bool sine_startup_enabled() const { return raw.f.use_sine_start != 0; }
    void set_sine_startup_enabled(bool en) { raw.f.use_sine_start = en ? 1 : 0; }

    bool bidirectional_enabled() const { return raw.f.bi_direction != 0; }
    void set_bidirectional_enabled(bool en) { raw.f.bi_direction = en ? 1 : 0; }

    bool complementary_pwm_enabled() const { return raw.f.comp_pwm != 0; }
    void set_complementary_pwm_enabled(bool en) { raw.f.comp_pwm = en ? 1 : 0; }

    bool stuck_rotor_protection_enabled() const { return raw.f.stuck_rotor_protection != 0; }
    void set_stuck_rotor_protection_enabled(bool en) { raw.f.stuck_rotor_protection = en ? 1 : 0; }

    uint16_t current_limit_amps() const { return (uint16_t)(raw.f.limits.current * 2); }
    bool current_limit_disabled() const { return raw.f.limits.current >= 202; }
    void set_current_limit_amps(uint16_t amps) {
        raw.f.limits.current = (uint8_t)am32_detail::clamp_i((int)(amps / 2), 0, 201);
    }
    void disable_current_limit() { raw.f.limits.current = 202; }

    uint8_t temperature_limit_c() const { return raw.f.limits.temperature; }
    bool temperature_limit_disabled() const { return raw.f.limits.temperature >= 141; }
    void set_temperature_limit_c(uint8_t c) {
        raw.f.limits.temperature = (uint8_t)am32_detail::clamp_i(c, 70, 140);
    }
    void disable_temperature_limit() { raw.f.limits.temperature = 255; }

    uint8_t brake_on_stop() const { return raw.f.brake_on_stop; }
    void set_brake_on_stop(uint8_t v) { raw.f.brake_on_stop = v; }

    uint8_t drag_brake_strength() const { return raw.f.drag_brake_strength; }
    void set_drag_brake_strength(uint8_t v) {
        raw.f.drag_brake_strength = (uint8_t)am32_detail::clamp_i(v, 1, 10);
    }

    uint8_t driving_brake_strength() const { return raw.f.driving_brake_strength; }
    void set_driving_brake_strength(uint8_t v) {
        raw.f.driving_brake_strength = (uint8_t)am32_detail::clamp_i(v, 1, 10);
    }

    uint8_t beep_volume() const { return raw.f.beep_volume; }
    void set_beep_volume(uint8_t v) { raw.f.beep_volume = (uint8_t)am32_detail::clamp_i(v, 0, 11); }
};

enum class Am32ConfigStatus : uint8_t {
    OK = 0,
    VALIDATION_ERROR,     // writeSettings()に渡した値が範囲外
    NOT_IN_CONFIG_MODE,   // enterConfigMode()に成功していない状態でread/write/save等を呼んだ
    PROTOCOL_ERROR,       // 詳細は AM32::last_protocol_status() を参照
    INIT_FAILED,          // PIO SM確保失敗等、致命的な初期化エラー
};
const char* am32_config_status_to_string(Am32ConfigStatus s);

// 依頼された値の妥当性を検証する。falseの場合 err に理由を書き込む(err_len込みで安全)。
bool am32_validate_settings(const AM32Settings& s, char* err, size_t err_len);

// AM32 ESCとの設定通信を統括する高レベルクラス。
//
// [書き込み/保存の設計について]
// AM32公式プロトコルには「一時書き込み」と「不揮発保存」を分離する概念が無く、
// SET_ADDRESS + SET_BUFFER(+payload) + PROG_FLASH の一連のコマンドがそのまま
// flashへの書き込み(=保存)そのものである(調査レポート§5参照)。
// 一方で依頼のAPI形状(readSettings→値変更→writeSettings→saveSettings)に
// 合わせるため、本実装では以下の2段構成にしている:
//   - writeSettings(): 値を検証しRAM上のstaging(cached_)を更新するだけ(ESCへは触れない)
//   - saveSettings() : cached_の内容を実際にESCのflashへ書き込む(=公式プロトコルのwrite)
// プロトコル自体を拡張・変更しているわけではない点に注意。
class AM32 {
public:
    enum class Mode : uint8_t { OFF, CONFIG, RUN };

    // gpio: ESCのS信号線に繋がるGPIO。pio: 専有するPIOブロック(既定pio0)。
    void init(uint gpio, PIO pio = pio0);

    // ESC電源をMOSFET等で制御する場合に呼ぶ(呼ばなければ電源は外部管理のまま)。
    // active_high=true: GPIO Hi でESC ON。信号線からの逆給電防止のため、
    // 呼び出し直後は必ずOFF状態にする。
    void set_power_gpio(uint power_gpio, bool active_high = true);
    bool has_power_control() const { return has_power_gpio_; }
    void power_off();
    void power_on();

    // 電源ON(制御していれば再投入)→静音待機→handshakeまで行う。
    //
    // has_power_control()==false の場合(ESC電源をMOSFET等で自動制御できない場合)、
    // ESCのbootloaderはこちらの呼び出しタイミングとは無関係に「電源投入直後の
    // 約50ms」しか設定通信を受け付けない(出典: 調査レポート§2.2 checkForSignal())。
    // そのため manual_retry_ms の間、probe送信→短い応答待ち→十分な無信号区間、を
    // 繰り返しpollingする(AM32公式Configuratorも「Connect後にバッテリを挿す」運用)。
    // 呼び出し側はこの関数を呼んだ直後にESCのバッテリを挿し直すこと。
    Am32ConfigStatus enterConfigMode(uint32_t manual_retry_ms = 10000);
    // CMD_RUNでESCをbootloaderからapplicationへ復帰させ、GPIOを解放する。
    // 呼び出し後、信号線を再度PWM/DShot等で使うのは呼び出し側の責務
    // (例: SuctionEscActuator::init() を呼び直す)。
    Am32ConfigStatus exitConfigMode();

    Am32ConfigStatus readSettings(AM32Settings& out);
    Am32ConfigStatus writeSettings(const AM32Settings& in);  // 検証してRAM staging更新のみ
    Am32ConfigStatus saveSettings();                          // stagingを実際にESC flashへ書き込む
    // save前の未保存変更(cached_)を、直近read成功時点(last_read_)まで巻き戻す。
    bool rollback();

    Mode mode() const { return mode_; }
    bool has_valid_devinfo() const { return devinfo_valid_; }
    const uint8_t* device_info() const { return devinfo_; }  // 9byte, 出典: 調査レポート§3.1
    const AM32Settings& cached_settings() const { return cached_; }

    Am32Protocol::Status last_protocol_status() const { return last_protocol_status_; }

private:
    Am32Protocol proto_;
    uint gpio_ = 0;
    PIO  pio_  = nullptr;

    bool has_power_gpio_ = false;
    uint power_gpio_ = 0;
    bool power_active_high_ = true;

    Mode mode_ = Mode::OFF;
    uint8_t devinfo_[9] = {};
    bool devinfo_valid_ = false;

    AM32Settings cached_;
    AM32Settings last_read_;
    bool has_read_ = false;

    Am32Protocol::Status last_protocol_status_ = Am32Protocol::Status::OK;
};

// --- デバッグ用CLIヘルパー ---
// "am32 <args>" の <args> 部分を渡して呼ぶ。結果は emit() へ1行ずつ渡される
// (printf配線やUSB CDC送信はemit側の責務。これによりバッファ管理を呼び出し側に委ねる)。
// 対応コマンド: read / dump / get <field> / set <field> <value> / save / rollback / reboot
using Am32CliEmit = void (*)(const char* line);
void am32_handle_cli(AM32& esc, const char* args, Am32CliEmit emit);
