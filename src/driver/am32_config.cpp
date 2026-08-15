#include "driver/am32_config.hpp"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "hardware/gpio.h"
#include "pico/time.h"

const char* am32_config_status_to_string(Am32ConfigStatus s) {
    switch (s) {
        case Am32ConfigStatus::OK: return "OK";
        case Am32ConfigStatus::VALIDATION_ERROR: return "VALIDATION_ERROR";
        case Am32ConfigStatus::NOT_IN_CONFIG_MODE: return "NOT_IN_CONFIG_MODE";
        case Am32ConfigStatus::PROTOCOL_ERROR: return "PROTOCOL_ERROR";
        case Am32ConfigStatus::INIT_FAILED: return "INIT_FAILED";
    }
    return "UNKNOWN";
}

bool am32_validate_settings(const AM32Settings& s, char* err, size_t err_len) {
    auto fail = [&](const char* msg) -> bool {
        if (err && err_len > 0) {
            snprintf(err, err_len, "%s", msg);
        }
        return false;
    };
    if (s.raw.f.motor_poles < 2 || s.raw.f.motor_poles > 36) {
        return fail("motor_poles out of range (2-36)");
    }
    if (s.raw.f.pwm_frequency < 8 || s.raw.f.pwm_frequency > 144) {
        return fail("pwm_frequency out of range (8-144 kHz)");
    }
    if (s.raw.f.startup_power < 50 || s.raw.f.startup_power > 150) {
        return fail("startup_power out of range (50-150 %)");
    }
    if (s.raw.f.advance_level > 42) {
        return fail("timing out of range (0-32)");
    }
    if (!s.temperature_limit_disabled() &&
        (s.raw.f.limits.temperature < 70 || s.raw.f.limits.temperature > 140)) {
        return fail("temperature_limit out of range (70-140 C, or disabled)");
    }
    if (s.raw.f.drag_brake_strength < 1 || s.raw.f.drag_brake_strength > 10) {
        return fail("drag_brake_strength out of range (1-10)");
    }
    if (s.raw.f.driving_brake_strength < 1 || s.raw.f.driving_brake_strength > 10) {
        return fail("driving_brake_strength out of range (1-10)");
    }
    if (s.raw.f.beep_volume > 11) {
        return fail("beep_volume out of range (0-11)");
    }
    return true;
}

void AM32::init(uint gpio, PIO pio) {
    gpio_ = gpio;
    pio_ = pio;
    mode_ = Mode::OFF;
    devinfo_valid_ = false;
    has_read_ = false;
}

void AM32::set_power_gpio(uint power_gpio, bool active_high) {
    has_power_gpio_ = true;
    power_gpio_ = power_gpio;
    power_active_high_ = active_high;
    gpio_init(power_gpio_);
    gpio_set_dir(power_gpio_, GPIO_OUT);
    power_off();  // 誤動作・逆給電防止のため既定はOFF
}

void AM32::power_off() {
    if (!has_power_gpio_) {
        return;
    }
    gpio_put(power_gpio_, power_active_high_ ? 0 : 1);
}

void AM32::power_on() {
    if (!has_power_gpio_) {
        return;
    }
    gpio_put(power_gpio_, power_active_high_ ? 1 : 0);
}

Am32ConfigStatus AM32::enterConfigMode(uint32_t manual_retry_ms) {
    if (mode_ == Mode::CONFIG) {
        return Am32ConfigStatus::OK;
    }
    devinfo_valid_ = false;

    if (has_power_gpio_) {
        // 電源投入タイミングをこちらで確実に把握できるため、1回のhandshakeで足りる。
        power_off();
        // ESC内部の平滑コンデンサ放電待ち。この待ち時間自体はAM32公式ソースに
        // 規定が無い一般的なハードウェア上の余裕であり(出典なし)、実機で
        // handshakeが安定しない場合は伸ばして調整すること。
        sleep_ms(300);
        if (!proto_.init(pio_, gpio_)) {
            return Am32ConfigStatus::INIT_FAILED;
        }
        power_on();
        // ESC bootloaderのcheckForSignal()が完了するまでラインを駆動せず待つ
        // (出典: 調査レポート§2.2、最大約55ms)。
        proto_.wait_powerup_silence();
        last_protocol_status_ = proto_.handshake(devinfo_, Am32Protocol::HANDSHAKE_TIMEOUT_US);
    } else {
        // 電源制御が無い場合、ESCの電源投入タイミングはこちらから制御できない。
        // AM32 bootloaderは電源投入直後の約50msしか設定通信を受け付けない
        // (出典: 調査レポート§2.2)ため、その窓を人手のバッテリ抜き差しで捉え
        // られるよう、probe送信の合間に十分な無信号区間を挟みながら
        // manual_retry_msの間pollingする(公式Configuratorも同様の運用)。
        if (!proto_.init(pio_, gpio_)) {
            return Am32ConfigStatus::INIT_FAILED;
        }
        printf("am32: no power control configured -- plug in the ESC battery now "
               "(polling for %lu ms)...\n",
               (unsigned long)manual_retry_ms);
        const uint64_t deadline = time_us_64() + (uint64_t)manual_retry_ms * 1000;
        int attempt = 0;
        do {
            attempt++;
            last_protocol_status_ = proto_.handshake(devinfo_, Am32Protocol::HANDSHAKE_TIMEOUT_US);
            if (last_protocol_status_ == Am32Protocol::Status::OK) {
                printf("am32: handshake OK (attempt %d)\n", attempt);
                break;
            }
            // 大半の時間を無信号(idle)のまま過ごし、ESC側checkForSignal()の
            // pull-down判定窓を自分のprobe送信で誤って埋めないようにする。
            sleep_ms(250);
        } while (time_us_64() < deadline);
    }

    if (last_protocol_status_ != Am32Protocol::Status::OK) {
        printf("am32: handshake failed -> %s\n",
               Am32Protocol::status_to_string(last_protocol_status_));
        proto_.deinit();
        mode_ = Mode::OFF;
        return Am32ConfigStatus::PROTOCOL_ERROR;
    }

    devinfo_valid_ = true;
    mode_ = Mode::CONFIG;
    return Am32ConfigStatus::OK;
}

Am32ConfigStatus AM32::exitConfigMode() {
    if (mode_ != Mode::CONFIG) {
        return Am32ConfigStatus::OK;
    }
    // 出典: bootloader/main.c CMD_RUN — ESCはbootloaderからapplicationへjumpし、応答は返さない。
    proto_.cmd_run();
    sleep_ms(2);
    proto_.deinit();
    mode_ = Mode::RUN;
    return Am32ConfigStatus::OK;
}

Am32ConfigStatus AM32::readSettings(AM32Settings& out) {
    if (mode_ != Mode::CONFIG) {
        return Am32ConfigStatus::NOT_IN_CONFIG_MODE;
    }
    last_protocol_status_ = proto_.set_address(Am32Protocol::ADDRESS_MAGIC_EEPROM);
    if (last_protocol_status_ != Am32Protocol::Status::OK) {
        return Am32ConfigStatus::PROTOCOL_ERROR;
    }
    last_protocol_status_ = proto_.read_flash(cached_.raw.buffer, (uint16_t)AM32Settings::LAYOUT_SIZE);
    if (last_protocol_status_ != Am32Protocol::Status::OK) {
        return Am32ConfigStatus::PROTOCOL_ERROR;
    }
    last_read_ = cached_;
    has_read_ = true;
    out = cached_;
    return Am32ConfigStatus::OK;
}

Am32ConfigStatus AM32::writeSettings(const AM32Settings& in) {
    char err[64];
    if (!am32_validate_settings(in, err, sizeof(err))) {
        return Am32ConfigStatus::VALIDATION_ERROR;
    }
    // ここではRAM上のstagingを更新するのみ。実際のESC flash書き込みはsaveSettings()で行う
    // (クラスコメント「書き込み/保存の設計について」参照)。
    cached_ = in;
    return Am32ConfigStatus::OK;
}

Am32ConfigStatus AM32::saveSettings() {
    if (mode_ != Mode::CONFIG) {
        return Am32ConfigStatus::NOT_IN_CONFIG_MODE;
    }
    last_protocol_status_ = proto_.set_address(Am32Protocol::ADDRESS_MAGIC_EEPROM);
    if (last_protocol_status_ != Am32Protocol::Status::OK) {
        return Am32ConfigStatus::PROTOCOL_ERROR;
    }
    last_protocol_status_ = proto_.write_buffer(cached_.raw.buffer, (uint16_t)AM32Settings::LAYOUT_SIZE);
    if (last_protocol_status_ != Am32Protocol::Status::OK) {
        return Am32ConfigStatus::PROTOCOL_ERROR;
    }
    // 出典: 調査レポート§5 — SET_BUFFERはESC側のRAMバッファへ積むだけで、実際にflashへ
    // 書き込むのはPROG_FLASHコマンド。
    last_protocol_status_ = proto_.prog_flash();
    if (last_protocol_status_ != Am32Protocol::Status::OK) {
        return Am32ConfigStatus::PROTOCOL_ERROR;
    }
    last_read_ = cached_;  // 保存成功。rollback()の基準点を更新する
    return Am32ConfigStatus::OK;
}

bool AM32::rollback() {
    if (!has_read_) {
        return false;
    }
    cached_ = last_read_;
    return true;
}

// ---------------- CLI ----------------

namespace {

void emit_line(Am32CliEmit emit, const char* fmt, ...) {
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    emit(buf);
}

void dump_settings(const AM32Settings& s, Am32CliEmit emit) {
    emit_line(emit, "eeprom_version   : %u", s.eeprom_version());
    emit_line(emit, "firmware         : %u.%u", s.firmware_major(), s.firmware_minor());
    emit_line(emit, "motor_kv         : %u", s.motor_kv());
    emit_line(emit, "motor_poles      : %u", s.motor_poles());
    emit_line(emit, "timing           : %u (%.2f deg)", s.timing(), (double)s.timing_degrees());
    emit_line(emit, "pwm_frequency    : %u kHz", s.pwm_frequency_khz());
    emit_line(emit, "variable_pwm     : %s", s.variable_pwm_enabled() ? "on" : "off");
    emit_line(emit, "startup_power    : %u %%", s.startup_power_percent());
    emit_line(emit, "sine_startup     : %s", s.sine_startup_enabled() ? "on" : "off");
    emit_line(emit, "bidirectional    : %s", s.bidirectional_enabled() ? "on" : "off");
    emit_line(emit, "complementary_pwm: %s", s.complementary_pwm_enabled() ? "on" : "off");
    emit_line(emit, "stuck_rotor_prot : %s", s.stuck_rotor_protection_enabled() ? "on" : "off");
    emit_line(emit, "brake_on_stop    : %u", s.brake_on_stop());
    emit_line(emit, "drag_brake       : %u", s.drag_brake_strength());
    emit_line(emit, "driving_brake    : %u", s.driving_brake_strength());
    if (s.current_limit_disabled()) {
        emit_line(emit, "current_limit    : disabled");
    } else {
        emit_line(emit, "current_limit    : %u A", s.current_limit_amps());
    }
    if (s.temperature_limit_disabled()) {
        emit_line(emit, "temp_limit       : disabled");
    } else {
        emit_line(emit, "temp_limit       : %u C", s.temperature_limit_c());
    }
    emit_line(emit, "beep_volume      : %u", s.beep_volume());
}

// "get"/"set" が受け付けるフィールド名。依頼にあった項目 + AM32Settings公開分。
bool cli_get(const AM32Settings& s, const char* field, char* out, size_t out_len) {
    if (!strcmp(field, "kv")) { snprintf(out, out_len, "%u", s.motor_kv()); return true; }
    if (!strcmp(field, "poles")) { snprintf(out, out_len, "%u", s.motor_poles()); return true; }
    if (!strcmp(field, "timing")) { snprintf(out, out_len, "%u", s.timing()); return true; }
    if (!strcmp(field, "pwm")) { snprintf(out, out_len, "%u", s.pwm_frequency_khz()); return true; }
    if (!strcmp(field, "variable_pwm")) { snprintf(out, out_len, "%u", s.variable_pwm_enabled() ? 1 : 0); return true; }
    if (!strcmp(field, "startup_power")) { snprintf(out, out_len, "%u", s.startup_power_percent()); return true; }
    if (!strcmp(field, "sine_start")) { snprintf(out, out_len, "%u", s.sine_startup_enabled() ? 1 : 0); return true; }
    if (!strcmp(field, "bidirectional")) { snprintf(out, out_len, "%u", s.bidirectional_enabled() ? 1 : 0); return true; }
    if (!strcmp(field, "current_limit")) { snprintf(out, out_len, "%u", s.current_limit_amps()); return true; }
    if (!strcmp(field, "temp_limit")) { snprintf(out, out_len, "%u", s.temperature_limit_c()); return true; }
    if (!strcmp(field, "brake_on_stop")) { snprintf(out, out_len, "%u", s.brake_on_stop()); return true; }
    if (!strcmp(field, "drag_brake")) { snprintf(out, out_len, "%u", s.drag_brake_strength()); return true; }
    if (!strcmp(field, "driving_brake")) { snprintf(out, out_len, "%u", s.driving_brake_strength()); return true; }
    if (!strcmp(field, "beep_volume")) { snprintf(out, out_len, "%u", s.beep_volume()); return true; }
    return false;
}

bool cli_set(AM32Settings& s, const char* field, long value) {
    if (!strcmp(field, "kv")) { s.set_motor_kv((uint16_t)value); return true; }
    if (!strcmp(field, "poles")) { s.set_motor_poles((uint8_t)value); return true; }
    if (!strcmp(field, "timing")) { s.set_timing((uint8_t)value); return true; }
    if (!strcmp(field, "pwm")) { s.set_pwm_frequency_khz((uint8_t)value); return true; }
    if (!strcmp(field, "variable_pwm")) { s.set_variable_pwm_enabled(value != 0); return true; }
    if (!strcmp(field, "startup_power")) { s.set_startup_power_percent((uint8_t)value); return true; }
    if (!strcmp(field, "sine_start")) { s.set_sine_startup_enabled(value != 0); return true; }
    if (!strcmp(field, "bidirectional")) { s.set_bidirectional_enabled(value != 0); return true; }
    if (!strcmp(field, "current_limit")) { s.set_current_limit_amps((uint16_t)value); return true; }
    if (!strcmp(field, "temp_limit")) { s.set_temperature_limit_c((uint8_t)value); return true; }
    if (!strcmp(field, "brake_on_stop")) { s.set_brake_on_stop((uint8_t)value); return true; }
    if (!strcmp(field, "drag_brake")) { s.set_drag_brake_strength((uint8_t)value); return true; }
    if (!strcmp(field, "driving_brake")) { s.set_driving_brake_strength((uint8_t)value); return true; }
    if (!strcmp(field, "beep_volume")) { s.set_beep_volume((uint8_t)value); return true; }
    return false;
}

}  // namespace

void am32_handle_cli(AM32& esc, const char* args, Am32CliEmit emit) {
    char buf[64];
    strncpy(buf, args, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* saveptr = nullptr;
    char* cmd = strtok_r(buf, " ", &saveptr);
    if (!cmd) {
        emit_line(emit, "usage: am32 read|dump|get <f>|set <f> <v>|save|rollback|reboot");
        return;
    }

    if (!strcmp(cmd, "read")) {
        AM32Settings s;
        Am32ConfigStatus st = esc.readSettings(s);
        if (st != Am32ConfigStatus::OK) {
            emit_line(emit, "read failed: %s (%s)", am32_config_status_to_string(st),
                       Am32Protocol::status_to_string(esc.last_protocol_status()));
            return;
        }
        emit_line(emit, "read OK");
        return;
    }
    if (!strcmp(cmd, "dump")) {
        dump_settings(esc.cached_settings(), emit);
        return;
    }
    if (!strcmp(cmd, "get")) {
        char* field = strtok_r(nullptr, " ", &saveptr);
        if (!field) {
            emit_line(emit, "usage: am32 get <field>");
            return;
        }
        char val[32];
        if (cli_get(esc.cached_settings(), field, val, sizeof(val))) {
            emit_line(emit, "%s = %s", field, val);
        } else {
            emit_line(emit, "unknown field: %s", field);
        }
        return;
    }
    if (!strcmp(cmd, "set")) {
        char* field = strtok_r(nullptr, " ", &saveptr);
        char* value = strtok_r(nullptr, " ", &saveptr);
        if (!field || !value) {
            emit_line(emit, "usage: am32 set <field> <value>");
            return;
        }
        AM32Settings s = esc.cached_settings();
        if (!cli_set(s, field, strtol(value, nullptr, 10))) {
            emit_line(emit, "unknown field: %s", field);
            return;
        }
        Am32ConfigStatus st = esc.writeSettings(s);
        if (st != Am32ConfigStatus::OK) {
            emit_line(emit, "set failed: %s", am32_config_status_to_string(st));
            return;
        }
        emit_line(emit, "%s staged (call 'am32 save' to write to ESC)", field);
        return;
    }
    if (!strcmp(cmd, "save")) {
        Am32ConfigStatus st = esc.saveSettings();
        if (st != Am32ConfigStatus::OK) {
            emit_line(emit, "save failed: %s (%s)", am32_config_status_to_string(st),
                       Am32Protocol::status_to_string(esc.last_protocol_status()));
            return;
        }
        emit_line(emit, "save OK");
        return;
    }
    if (!strcmp(cmd, "rollback")) {
        emit_line(emit, esc.rollback() ? "rolled back to last read" : "no prior read to roll back to");
        return;
    }
    if (!strcmp(cmd, "reboot")) {
        Am32ConfigStatus st = esc.exitConfigMode();
        emit_line(emit, "reboot: %s", am32_config_status_to_string(st));
        return;
    }
    emit_line(emit, "unknown command: %s", cmd);
}
