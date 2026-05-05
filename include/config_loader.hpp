#pragma once
#include <stdint.h>
#include <ArduinoJson.h>

// ConfigLoader — LittleFS on flash + ArduinoJson による設定ファイル読み書き
//
// 基本の使い方:
//   ConfigLoader::init();                              // 起動時に一度だけ
//   int v = ConfigLoader::get_int("sensing.interval_us", 1000);
//
// 構造体への一括ロード (config_mapping.hpp の convertFromJson が必要):
//   input_param_t param;
//   ConfigLoader::load_as("/hardware.json", param);   // ファイル → 構造体
//   // または既存の doc() から:
//   param = ConfigLoader::doc().as<input_param_t>();  // /config.json から
//
// Flash レイアウト: 末尾 256KB をファイルシステム領域として使用

class ConfigLoader {
public:
    // LittleFS マウント + /config.json 読み込み
    // 初回起動時はフォーマットしてデフォルト config.json を生成
    static bool init();

    static void deinit();

    // ドット区切りパスで値を取得 (例: "sensing.led_settle_us")
    static int   get_int  (const char *path, int   default_val);
    static float get_float(const char *path, float default_val);
    static bool  get_bool (const char *path, bool  default_val);

    // /config.json を現在の doc_ の内容で上書き保存
    static bool save();

    // JSON ルートドキュメントへの直接アクセス (/config.json の内容)
    static JsonDocument& doc() { return doc_; }

    // LittleFS 上の任意の JSON ファイルを dst に読み込む
    // 戻り値: true=成功, false=ファイル不在 or パースエラー
    static bool load_file(const char* path, JsonDocument& dst);

    // LittleFS 上の JSON ファイルを読み込んで型 T に変換する
    // config_mapping.hpp の convertFromJson(JsonVariantConst, T&) が定義されている型で使用可能
    //
    // 使用例:
    //   input_param_t param;
    //   if (!ConfigLoader::load_as("/hardware.json", param)) { /* エラー処理 */ }
    template<typename T>
    static bool load_as(const char* path, T& dst) {
        JsonDocument tmp;
        if (!load_file(path, tmp)) return false;
        convertFromJson(tmp.as<JsonVariantConst>(), dst);
        return true;
    }

private:
    static JsonDocument doc_;

    static bool mount_or_format();
    static bool read_json();
    static bool write_default_json();
};
