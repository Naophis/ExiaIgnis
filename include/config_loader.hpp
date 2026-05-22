#pragma once
#include <stdint.h>
#include <ArduinoJson.h>
#include "config_mapping.hpp"

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
        bind_log::file() = path;
        convertFromJson(tmp.as<JsonVariantConst>(), dst);
        bind_log::file() = "";
        return true;
    }

    // ---- ファイル転送用 低レベル API ----------------------------------------

    // rawバイト列をファイルに書き込む (作成 or 上書き)
    static bool write_file(const char* path, const uint8_t* data, size_t size);

    // rawバイト列としてファイルを読み出す。buf_size より大きければ失敗
    static bool read_file_raw(const char* path,
                              uint8_t* buf, size_t buf_size, size_t& out_size);

    // ファイルサイズを返す。ファイルが存在しなければ -1
    static int32_t file_size(const char* path);

    // ファイルを削除する
    static bool delete_file(const char* path);

    // LittleFS を再フォーマットして全ファイルを消去し、マウントし直す
    static bool format_all();

    // 起動時に破損が検出された場合、Core1 起動後にここで実際のフォーマットを行う
    // 破損があれば true を返す (フォーマット実行)、正常なら false
    static bool check_and_reformat();

    // ルートディレクトリのファイル一覧を列挙する
    // cb(ctx, name, size) がファイルごとに呼ばれる
    static void list_files(void (*cb)(void* ctx, const char* name, int32_t size),
                           void* ctx);

    // ストレージ使用量を返す (bytes)。used / total に書き込む。
    static void storage_info(uint32_t &used, uint32_t &total);

private:
    static JsonDocument doc_;
    static bool needs_reformat_;  // 起動時に破損を検出、Core1 起動後に reformat する

    static bool mount_or_format();
    static bool read_json();
    static bool write_default_json();
};
