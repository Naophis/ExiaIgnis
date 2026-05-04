#pragma once
#include <stdint.h>
#include <ArduinoJson.h>

// ConfigLoader — LittleFS on flash + ArduinoJson による設定ファイル読み書き
//
// 使い方:
//   1. main() の先頭 (multicore_launch_core1 より前) で ConfigLoader::init() を呼ぶ
//   2. 各モジュールで ConfigLoader::doc()["sensing"]["led_settle_us"] | 13 のように取得
//   3. /config.json が存在しない場合は自動的にデフォルト値で生成される
//
// Flash レイアウト: 末尾 256KB をファイルシステム領域として使用

// --------------------------------------------------------
// [使用例] 複雑な構造体のパラメータを一括で読み込む方法
// --------------------------------------------------------
// ArduinoJsonでは、以下の関数を同一ネームスペースに定義することで
// JSONから構造体へのマッピング（doc["pid"].as<PidParam>()）が可能です。
//
// 例:
// struct PidParam { float p, i, d; };
// void convertFromJson(JsonVariantConst src, PidParam& dst) {
//     dst.p = src["p"] | 0.0f;
//     dst.i = src["i"] | 0.0f;
//     dst.d = src["d"] | 0.0f;
// }
// 
// 利用側 (main_task.cppなど):
// param->motor_pid = ConfigLoader::doc()["motor_pid"].as<PidParam>();
// --------------------------------------------------------

class ConfigLoader {
public:
    // LittleFS マウント + config.json 読み込み
    // 初回起動時はフォーマットしてデフォルト config.json を生成
    static bool init();

    static void deinit();

    // ドット区切りパスで値を取得 (例: "sensing.led_settle_us")
    // ※ArduinoJson を直接使うこともできますが、互換性のため残します
    static int   get_int  (const char *path, int   default_val);
    static float get_float(const char *path, float default_val);
    static bool  get_bool (const char *path, bool  default_val);

    // config.json を現在の doc_ の内容で上書き保存
    static bool save();

    // JSON ルートドキュメントへの直接アクセス
    static JsonDocument& doc() { return doc_; }

private:
    static JsonDocument doc_;

    static bool mount_or_format();
    static bool read_json();
    static bool write_default_json();
};
