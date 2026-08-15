// AM32 ESC 設定read/write/save 使用例。
//
// このファイルはビルド対象(CMakeLists.txtのadd_executable)には含めていない
// 参考実装。src/main/main_task_usb.cpp 等、既存のUSBコマンドディスパッチャに
// am32_handle_cli() を配線する際の使い方の参考にすること。
//
// 前提: SUCTION_ESC_PWM(GPIO11, define.hpp)に接続された吸引用AM32 ESC
// (FlyingRC Mini ESC 40A / AM32ファームウェア)を対象とする。
// このGPIOは通常時 SuctionEscActuator がRC標準PWM(1000-2000us)で駆動して
// おり、AM32設定通信中はPIOがGPIO機能を横取りするため、呼び出し側は
// 設定モードに入る前にSuctionEscActuatorの出力を止めておくこと
// (PWMスライスが動いたままでも実害はないが、モーターへ意図しない指令が
// 残らないよう安全のため)。

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "driver/am32_config.hpp"
#include "define.hpp"

static void print_line(const char* line) { printf("%s\n", line); }

static void am32_read_modify_save_example() {
    AM32 esc;
    esc.init(SUCTION_ESC_PWM, pio0);

    // ESC電源をMOSFET等で制御している場合のみ配線する。していない場合は
    // set_power_gpio()を呼ばず、外部で電源が既に入っている前提で進めてよい。
    // esc.set_power_gpio(ESC_POWER_MOSFET_GPIO, /*active_high=*/true);

    printf("entering AM32 config mode...\n");
    Am32ConfigStatus st = esc.enterConfigMode();
    if (st != Am32ConfigStatus::OK) {
        printf("enterConfigMode failed: %s (%s)\n", am32_config_status_to_string(st),
               Am32Protocol::status_to_string(esc.last_protocol_status()));
        return;
    }
    printf("handshake OK. devinfo:");
    for (int i = 0; i < 9; i++) {
        printf(" %02X", esc.device_info()[i]);
    }
    printf("\n");

    AM32Settings settings;
    st = esc.readSettings(settings);
    if (st != Am32ConfigStatus::OK) {
        printf("readSettings failed: %s (%s)\n", am32_config_status_to_string(st),
               Am32Protocol::status_to_string(esc.last_protocol_status()));
        esc.exitConfigMode();
        return;
    }
    printf("KV = %u\n", settings.motor_kv());
    printf("Poles = %u\n", settings.motor_poles());
    printf("Timing = %u\n", settings.timing());
    printf("PWM freq = %u kHz\n", settings.pwm_frequency_khz());

    // --- パラメータ変更 ---
    settings.set_motor_kv(8500);
    settings.set_motor_poles(12);
    settings.set_timing(25);

    st = esc.writeSettings(settings);  // 範囲チェック + RAM staging更新のみ(まだESCへは書かない)
    if (st != Am32ConfigStatus::OK) {
        printf("writeSettings failed: %s\n", am32_config_status_to_string(st));
        esc.exitConfigMode();
        return;
    }

    st = esc.saveSettings();  // ここで実際にESCのflashへ書き込む
    if (st != Am32ConfigStatus::OK) {
        printf("saveSettings failed: %s (%s) -- rolling back staged value\n",
               am32_config_status_to_string(st), Am32Protocol::status_to_string(esc.last_protocol_status()));
        esc.rollback();
    } else {
        printf("save OK\n");
    }

    // 保存内容を読み直して検証する(ESC/EEPROM破損やversion不一致の検出も兼ねる)。
    AM32Settings verify;
    st = esc.readSettings(verify);
    if (st == Am32ConfigStatus::OK) {
        printf("verify: KV=%u Poles=%u Timing=%u\n", verify.motor_kv(), verify.motor_poles(),
               verify.timing());
    }

    esc.exitConfigMode();  // ESCをapplication(通常のPWM/DShot受付状態)へ戻す
    printf("exited config mode\n");

    // 呼び出し側で通常動作用のPWM出力を再開する(SuctionEscActuator::init()等)。
}

// USB CDC等から1行コマンドを受け取り、"am32 " prefixを剥がしてam32_handle_cli()へ渡す例。
static AM32 g_esc;
static bool g_am32_ready = false;

static void handle_serial_line(const char* line) {
    const char* prefix = "am32 ";
    const size_t prefix_len = 5;
    if (strncmp(line, prefix, prefix_len) != 0) {
        return;
    }
    if (!g_am32_ready) {
        g_esc.init(SUCTION_ESC_PWM, pio0);
        if (g_esc.enterConfigMode() != Am32ConfigStatus::OK) {
            printf("am32: enterConfigMode failed\n");
            return;
        }
        g_am32_ready = true;
    }
    am32_handle_cli(g_esc, line + prefix_len, print_line);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    am32_read_modify_save_example();

    // handle_serial_line()は、実運用では main_task_usb.cpp の
    // USBコマンドディスパッチループから1行ずつ呼び出す想定(参考のため未使用警告を抑制)。
    (void)&handle_serial_line;

    while (true) {
        tight_loop_contents();
    }
}
