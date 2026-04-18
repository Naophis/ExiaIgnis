#pragma once
#include <stdint.h>

struct cJSON;  // 前方宣言 — cJSON.h は config_loader.cpp 内でのみ include

// ConfigLoader — LittleFS on flash + cJSON による設定ファイル読み書き
//
// 使い方:
//   1. main() の先頭 (multicore_launch_core1 より前) で ConfigLoader::init() を呼ぶ
//   2. 各モジュールで ConfigLoader::get_int("sensing.led_settle_us", 13) のように取得
//   3. /config.json が存在しない場合は自動的にデフォルト値で生成される
//
// Flash レイアウト: 末尾 256KB をファイルシステム領域として使用

class ConfigLoader {
public:
    // LittleFS マウント + config.json 読み込み
    // 初回起動時はフォーマットしてデフォルト config.json を生成
    static bool init();

    static void deinit();

    // ドット区切りパスで値を取得 (例: "sensing.led_settle_us")
    // キーが存在しない場合は default_val を返す
    static int   get_int  (const char *path, int   default_val);
    static float get_float(const char *path, float default_val);
    static bool  get_bool (const char *path, bool  default_val);

    // config.json を現在の root_ の内容で上書き保存
    static bool save();

    // JSON ルートノードへの直接アクセス (高度な使用向け)
    static cJSON *root() { return root_; }

private:
    static cJSON *root_;

    static bool mount_or_format();
    static bool read_json();
    static bool write_default_json();
    static cJSON *resolve_path(const char *path);
};
