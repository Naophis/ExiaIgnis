#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "lfs.h"
#include "cJSON.h"
#include "config_loader.hpp"

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

static int flash_prog(const lfs_config *c, lfs_block_t block,
                      lfs_off_t off, const void *buf, lfs_size_t size) {
    uint32_t addr = LFS_FLASH_OFFSET + block * c->block_size + off;
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(addr, static_cast<const uint8_t*>(buf), size);
    restore_interrupts(ints);
    return LFS_ERR_OK;
}

static int flash_erase(const lfs_config *c, lfs_block_t block) {
    uint32_t addr = LFS_FLASH_OFFSET + block * c->block_size;
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(addr, c->block_size);
    restore_interrupts(ints);
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
cJSON *ConfigLoader::root_ = nullptr;

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
    if (root_) { cJSON_Delete(root_); root_ = nullptr; }
    lfs_unmount(&lfs);
}

bool ConfigLoader::save() {
    if (!root_) return false;
    char *text = cJSON_Print(root_);
    if (!text) return false;

    lfs_file_t f;
    bool ok = false;
    if (lfs_file_open(&lfs, &f, "/config.json",
                      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) >= 0) {
        lfs_file_write(&lfs, &f, text, strlen(text));
        lfs_file_close(&lfs, &f);
        ok = true;
    }
    free(text);
    return ok;
}

int ConfigLoader::get_int(const char *path, int default_val) {
    cJSON *node = resolve_path(path);
    return (node && cJSON_IsNumber(node)) ? (int)node->valuedouble : default_val;
}

float ConfigLoader::get_float(const char *path, float default_val) {
    cJSON *node = resolve_path(path);
    return (node && cJSON_IsNumber(node)) ? (float)node->valuedouble : default_val;
}

bool ConfigLoader::get_bool(const char *path, bool default_val) {
    cJSON *node = resolve_path(path);
    if (!node) return default_val;
    if (cJSON_IsTrue(node))  return true;
    if (cJSON_IsFalse(node)) return false;
    return default_val;
}

// ---- private ----

bool ConfigLoader::mount_or_format() {
    if (lfs_mount(&lfs, &lfs_cfg) == 0) return true;
    printf("[config] formatting LittleFS...\n");
    if (lfs_format(&lfs, &lfs_cfg) < 0) return false;
    return lfs_mount(&lfs, &lfs_cfg) == 0;
}

bool ConfigLoader::read_json() {
    lfs_file_t f;
    if (lfs_file_open(&lfs, &f, "/config.json", LFS_O_RDONLY) < 0) return false;

    lfs_ssize_t size = lfs_file_size(&lfs, &f);
    char *buf = static_cast<char*>(malloc(size + 1));
    if (!buf) { lfs_file_close(&lfs, &f); return false; }

    lfs_file_read(&lfs, &f, buf, size);
    lfs_file_close(&lfs, &f);
    buf[size] = '\0';

    if (root_) cJSON_Delete(root_);
    root_ = cJSON_Parse(buf);
    free(buf);

    if (!root_) {
        printf("[config] JSON parse error\n");
        return false;
    }
    printf("[config] loaded config.json (%d bytes)\n", (int)size);
    return true;
}

bool ConfigLoader::write_default_json() {
    lfs_file_t f;
    if (lfs_file_open(&lfs, &f, "/config.json",
                      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) < 0) return false;
    lfs_file_write(&lfs, &f, DEFAULT_CONFIG, strlen(DEFAULT_CONFIG));
    lfs_file_close(&lfs, &f);
    return true;
}

cJSON *ConfigLoader::resolve_path(const char *path) {
    if (!root_ || !path) return nullptr;

    // ドット区切りを分解して順にたどる
    char buf[64];
    strncpy(buf, path, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    cJSON *node = root_;
    char *token = strtok(buf, ".");
    while (token && node) {
        node = cJSON_GetObjectItemCaseSensitive(node, token);
        token = strtok(nullptr, ".");
    }
    return node;
}
