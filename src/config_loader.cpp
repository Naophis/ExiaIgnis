#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "lfs.h"
#include <string>
#include "config_loader.hpp"
#include "config_mapping.hpp"

// ============================================================
// Flash ブロックデバイス設定 (末尾 256KB をFS領域に使用)
// ============================================================
#define LFS_FLASH_SIZE    (256 * 1024)
#define LFS_FLASH_OFFSET  (PICO_FLASH_SIZE_BYTES - LFS_FLASH_SIZE)
#define LFS_BLOCK_SIZE    FLASH_SECTOR_SIZE   // 4096 bytes
#define LFS_BLOCK_COUNT   (LFS_FLASH_SIZE / LFS_BLOCK_SIZE)  // 64 blocks

static int flash_read(const lfs_config *c, lfs_block_t block,
                      lfs_off_t off, void *buf, lfs_size_t size) {
    uint32_t addr = LFS_FLASH_OFFSET + block * c->block_size + off;
    memcpy(buf, reinterpret_cast<const void*>(XIP_BASE + addr), size);
    return LFS_ERR_OK;
}

// flash_safe_execute コールバック: Core1 を安全に停止してから実行される
struct lfs_erase_args { uint32_t addr; uint32_t size; };
struct lfs_prog_args  { uint32_t addr; const uint8_t *buf; uint32_t size; };

static void __no_inline_not_in_flash_func(do_lfs_erase)(void *varg) {
    auto *a = static_cast<lfs_erase_args *>(varg);
    flash_range_erase(a->addr, a->size);
}

static void __no_inline_not_in_flash_func(do_lfs_prog)(void *varg) {
    auto *a = static_cast<lfs_prog_args *>(varg);
    flash_range_program(a->addr, a->buf, a->size);
}

static int flash_prog(const lfs_config *c, lfs_block_t block,
                      lfs_off_t off, const void *buf, lfs_size_t size) {
    lfs_prog_args args = {
        LFS_FLASH_OFFSET + (uint32_t)(block * c->block_size + off),
        static_cast<const uint8_t *>(buf),
        (uint32_t)size,
    };
    flash_safe_execute(do_lfs_prog, &args, UINT32_MAX);
    return LFS_ERR_OK;
}

static int flash_erase(const lfs_config *c, lfs_block_t block) {
    lfs_erase_args args = {
        LFS_FLASH_OFFSET + (uint32_t)(block * c->block_size),
        (uint32_t)c->block_size,
    };
    flash_safe_execute(do_lfs_erase, &args, UINT32_MAX);
    return LFS_ERR_OK;
}

static int flash_sync(const lfs_config *) { return LFS_ERR_OK; }

static uint8_t lfs_read_buf [FLASH_PAGE_SIZE];
static uint8_t lfs_prog_buf [FLASH_PAGE_SIZE];
static uint8_t lfs_lookahead[16];

static const lfs_config lfs_cfg = {
    .read  = flash_read,
    .prog  = flash_prog,
    .erase = flash_erase,
    .sync  = flash_sync,
    .read_size      = FLASH_PAGE_SIZE,
    .prog_size      = FLASH_PAGE_SIZE,
    .block_size     = LFS_BLOCK_SIZE,
    .block_count    = LFS_BLOCK_COUNT,
    .block_cycles   = 500,
    .cache_size     = FLASH_PAGE_SIZE,
    .lookahead_size = sizeof(lfs_lookahead),
    .read_buffer    = lfs_read_buf,
    .prog_buffer    = lfs_prog_buf,
    .lookahead_buffer = lfs_lookahead,
};

static lfs_t lfs;

// ============================================================
// デフォルト設定 JSON
// ============================================================
static const char DEFAULT_CONFIG[] =
"{\n"
"  \"sensing\": {\n"
"    \"led_settle_us\": 13,\n"
"    \"interval_us\": 1000\n"
"  }\n"
"}\n";

// ============================================================
// ConfigLoader 実装
// ============================================================
// ============================================================
// ConfigLoader 実装
// ============================================================
JsonDocument ConfigLoader::doc_;

bool ConfigLoader::init() {
    if (!mount_or_format()) {
        printf("[config] LittleFS mount failed\n");
        return false;
    }

    // config.json が存在しなければデフォルトを書く
    lfs_file_t f;
    if (lfs_file_open(&lfs, &f, "/config.json", LFS_O_RDONLY) < 0) {
        printf("[config] config.json not found, writing default\n");
        if (!write_default_json()) return false;
    } else {
        lfs_file_close(&lfs, &f);
    }

    return read_json();
}

void ConfigLoader::deinit() {
    doc_.clear();
    lfs_unmount(&lfs);
}

bool ConfigLoader::save() {
    lfs_file_t f;
    bool ok = false;
    if (lfs_file_open(&lfs, &f, "/config.json",
                      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) >= 0) {
        std::string output;
        serializeJson(doc_, output);
        lfs_file_write(&lfs, &f, output.c_str(), output.length());
        lfs_file_close(&lfs, &f);
        ok = true;
    }
    return ok;
}

static JsonVariant resolve_path(JsonDocument& doc, const char *path) {
    if (!path) return JsonVariant();
    
    char buf[64];
    strncpy(buf, path, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    JsonVariant node = doc.as<JsonVariant>();
    char *token = strtok(buf, ".");
    while (token && !node.isNull()) {
        node = node[token];
        token = strtok(nullptr, ".");
    }
    return node;
}

int ConfigLoader::get_int(const char *path, int default_val) {
    JsonVariant node = resolve_path(doc_, path);
    return node.is<int>() ? node.as<int>() : default_val;
}

float ConfigLoader::get_float(const char *path, float default_val) {
    JsonVariant node = resolve_path(doc_, path);
    return node.is<float>() ? node.as<float>() : default_val;
}

bool ConfigLoader::get_bool(const char *path, bool default_val) {
    JsonVariant node = resolve_path(doc_, path);
    return node.is<bool>() ? node.as<bool>() : default_val;
}

// ---- private ----

bool ConfigLoader::mount_or_format() {
    if (lfs_mount(&lfs, &lfs_cfg) == 0) return true;
    printf("[config] formatting LittleFS...\n");
    if (lfs_format(&lfs, &lfs_cfg) < 0) return false;
    return lfs_mount(&lfs, &lfs_cfg) == 0;
}

bool ConfigLoader::load_file(const char* path, JsonDocument& dst) {
    lfs_file_t f;
    if (lfs_file_open(&lfs, &f, path, LFS_O_RDONLY) < 0) {
        printf("[config] cannot open %s\n", path);
        return false;
    }

    lfs_ssize_t size = lfs_file_size(&lfs, &f);
    char *buf = static_cast<char*>(malloc(size + 1));
    if (!buf) { lfs_file_close(&lfs, &f); return false; }

    lfs_file_read(&lfs, &f, buf, size);
    lfs_file_close(&lfs, &f);
    buf[size] = '\0';

    DeserializationError error = deserializeJson(dst, buf);
    free(buf);

    if (error) {
        printf("[config] JSON parse error in %s: %s\n", path, error.c_str());
        return false;
    }
    printf("[config] loaded %s (%d bytes)\n", path, (int)size);
    return true;
}

bool ConfigLoader::read_json() {
    return load_file("/config.json", doc_);
}

bool ConfigLoader::write_file(const char* path, const uint8_t* data, size_t size) {
    lfs_file_t f;
    if (lfs_file_open(&lfs, &f, path,
                      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) < 0)
        return false;
    lfs_ssize_t n = lfs_file_write(&lfs, &f, data, size);
    lfs_file_close(&lfs, &f);
    return n == (lfs_ssize_t)size;
}

bool ConfigLoader::read_file_raw(const char* path,
                                  uint8_t* buf, size_t buf_size,
                                  size_t& out_size) {
    lfs_file_t f;
    if (lfs_file_open(&lfs, &f, path, LFS_O_RDONLY) < 0) return false;
    lfs_ssize_t size = lfs_file_size(&lfs, &f);
    if (size < 0 || (size_t)size > buf_size) {
        lfs_file_close(&lfs, &f);
        return false;
    }
    lfs_ssize_t n = lfs_file_read(&lfs, &f, buf, size);
    lfs_file_close(&lfs, &f);
    if (n < 0) return false;
    out_size = (size_t)n;
    return true;
}

int32_t ConfigLoader::file_size(const char* path) {
    lfs_file_t f;
    if (lfs_file_open(&lfs, &f, path, LFS_O_RDONLY) < 0) return -1;
    lfs_soff_t size = lfs_file_size(&lfs, &f);
    lfs_file_close(&lfs, &f);
    return (int32_t)size;
}

bool ConfigLoader::delete_file(const char* path) {
    return lfs_remove(&lfs, path) == LFS_ERR_OK;
}

bool ConfigLoader::format_all() {
    lfs_unmount(&lfs);
    printf("[config] formatting LittleFS...\n");
    if (lfs_format(&lfs, &lfs_cfg) < 0) {
        printf("[config] format failed\n");
        return false;
    }
    if (lfs_mount(&lfs, &lfs_cfg) != 0) {
        printf("[config] remount failed\n");
        return false;
    }
    printf("[config] format OK\n");
    return true;
}

void ConfigLoader::list_files(void (*cb)(void* ctx, const char* name, int32_t size),
                               void* ctx) {
    lfs_dir_t dir;
    if (lfs_dir_open(&lfs, &dir, "/") < 0) return;
    struct lfs_info info;
    while (lfs_dir_read(&lfs, &dir, &info) > 0) {
        if (info.type == LFS_TYPE_REG) {
            cb(ctx, info.name, (int32_t)info.size);
        }
    }
    lfs_dir_close(&lfs, &dir);
}

bool ConfigLoader::write_default_json() {
    lfs_file_t f;
    if (lfs_file_open(&lfs, &f, "/config.json",
                      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) < 0) return false;
    lfs_file_write(&lfs, &f, DEFAULT_CONFIG, strlen(DEFAULT_CONFIG));
    lfs_file_close(&lfs, &f);
    return true;
}
