#include "config_loader.hpp"
#include "pico/error.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <cstdlib>
#include <cstring>
#include <stdio.h>

// ─── USB シリアル受信 ─────────────────────────────────────────────────────
// idle_ms の無通信が続いたら返す。\n で終端された行を1つ読み込む。
int usb_read_with_timeout(char *buf, size_t max_size, uint32_t idle_ms) {
  size_t len = 0;
  absolute_time_t deadline = make_timeout_time_ms(idle_ms);
  while (len < max_size - 1) {
    if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0)
      break;
    int c = getchar_timeout_us(1000);
    if (c == PICO_ERROR_TIMEOUT)
      continue;
    buf[len++] = (char)c;
    deadline = make_timeout_time_ms(idle_ms); // 文字受信ごとにリセット
    if ((char)c == '\n')
      break;
  }
  buf[len] = '\0';
  return (int)len;
}

// ─── USB コマンド処理 ─────────────────────────────────────────────────────
// "filename@content" → ファイルを保存して true を返す (再ロード要)。
// "LIST" / "SHOW:name" / "READ:name" / "DELETE:name" / "DELETEALL" を処理する。
bool rx_usb_cmd(char *buf, int len) {
  while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r'))
    buf[--len] = '\0';
  if (len == 0)
    return false;

  // ── filename@content ─────────────────────────────────────────────────
  char *at = strchr(buf, '@');
  if (at) {
    *at = '\0';
    const char *name = buf;
    const char *content = at + 1;

    char path[68];
    path[0] = '/';
    strncpy(path + 1, name, sizeof(path) - 2);
    path[sizeof(path) - 1] = '\0';

    // flash_range_erase/prog は割り込みを ~100ms 禁止するため USB CDC
    // が切断される。 OK を先に送信し、USB が届けてから書き込む。
    printf("OK\n");
    fflush(stdout);
    sleep_ms(80); // USB が OK を転送するまで待機

    ConfigLoader::write_file(path, reinterpret_cast<const uint8_t *>(content),
                             strlen(content));
    return true;
  }

  // ── LIST ──────────────────────────────────────────────────────────────
  if (strcmp(buf, "LIST") == 0) {
    ConfigLoader::list_files(
        [](void *, const char *name, int32_t size) {
          printf("%s:%d\n", name, (int)size);
        },
        nullptr);
    printf("OK\n");
    fflush(stdout);
    return false;
  }

  // ── DELETEALL ─────────────────────────────────────────────────────────
  if (strcmp(buf, "DELETEALL") == 0) {
    printf("OK\n");
    fflush(stdout);
    sleep_ms(80);
    ConfigLoader::format_all();
    return false;
  }

  // ── DELETE:name ───────────────────────────────────────────────────────
  if (strncmp(buf, "DELETE:", 7) == 0) {
    char path[68];
    path[0] = '/';
    strncpy(path + 1, buf + 7, sizeof(path) - 2);
    path[sizeof(path) - 1] = '\0';
    printf(ConfigLoader::delete_file(path) ? "OK\n" : "ERR:not found\n");
    fflush(stdout);
    return false;
  }

  // ── SHOW:name ─────────────────────────────────────────────────────────
  if (strncmp(buf, "SHOW:", 5) == 0) {
    char path[68];
    path[0] = '/';
    strncpy(path + 1, buf + 5, sizeof(path) - 2);
    path[sizeof(path) - 1] = '\0';

    int32_t fsize = ConfigLoader::file_size(path);
    if (fsize < 0) {
      printf("ERR:not found\n");
      fflush(stdout);
      return false;
    }

    uint8_t *fbuf = (uint8_t *)malloc((size_t)fsize + 1);
    if (!fbuf) {
      printf("ERR:no memory\n");
      fflush(stdout);
      return false;
    }

    size_t out_size = 0;
    if (ConfigLoader::read_file_raw(path, fbuf, (size_t)fsize, out_size)) {
      fbuf[out_size] = '\0';
      printf("=== %s (%u bytes) ===\n", path, (unsigned)out_size);
      fflush(stdout);
      // USB CDC バッファ溢れ防止のため 256 バイトずつ出力
      for (size_t i = 0; i < out_size; i += 256) {
        size_t chunk = (out_size - i) < 256 ? (out_size - i) : 256;
        fwrite(fbuf + i, 1, chunk, stdout);
        fflush(stdout);
        sleep_ms(10);
      }
      printf("\n===\n");
      fflush(stdout);
    } else {
      printf("ERR:read failed\n");
      fflush(stdout);
    }
    free(fbuf);
    return false;
  }

  // ── READ:name ─────────────────────────────────────────────────────────
  if (strncmp(buf, "READ:", 5) == 0) {
    char path[68];
    path[0] = '/';
    strncpy(path + 1, buf + 5, sizeof(path) - 2);
    path[sizeof(path) - 1] = '\0';

    int32_t fsize = ConfigLoader::file_size(path);
    if (fsize < 0) {
      printf("ERR:not found\n");
      fflush(stdout);
      return false;
    }

    uint8_t *fbuf = (uint8_t *)malloc((size_t)fsize);
    if (!fbuf) {
      printf("ERR:no memory\n");
      fflush(stdout);
      return false;
    }

    size_t out_size = 0;
    if (ConfigLoader::read_file_raw(path, fbuf, (size_t)fsize, out_size)) {
      printf("%u\n", (unsigned)out_size);
      fflush(stdout);
      fwrite(fbuf, 1, out_size, stdout);
      fflush(stdout);
      printf("OK\n");
      fflush(stdout);
    } else {
      printf("ERR:read failed\n");
      fflush(stdout);
    }
    free(fbuf);
    return false;
  }

  return false;
}
