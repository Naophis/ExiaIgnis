#include "host_common.hpp"

#include <iostream>
#include <iterator>
#include <unistd.h>

static std::unordered_map<std::string, std::string> g_files;

bool ConfigLoader::load_file(const char *path, JsonDocument &dst) {
  std::string name = path;
  if (!name.empty() && name[0] == '/')
    name.erase(0, 1);
  const auto it = g_files.find(name);
  if (it == g_files.end())
    return false;
  const auto err = deserializeJson(dst, it->second);
  if (err) {
    printf("[host] %s: JSON parse error: %s\n", path, err.c_str());
    return false;
  }
  return true;
}

namespace host {

std::vector<uint8_t> embed_maze(const std::vector<uint8_t> &src, int maze_size, uint8_t fill) {
  int n = 0;
  while (n * n < (int)src.size())
    n++;
  if (n * n != (int)src.size() || n > maze_size)
    return {};
  std::vector<uint8_t> out(maze_size * maze_size, fill);
  for (int y = 0; y < n; y++)
    for (int x = 0; x < n; x++)
      out[x + y * maze_size] = src[x + y * n];
  return out;
}

bool read_input(JsonDocument &in, JsonDocument &out) {
  const std::string input((std::istreambuf_iterator<char>(std::cin)), std::istreambuf_iterator<char>());
  if (const auto err = deserializeJson(in, input)) {
    out["ok"] = false;
    out["error"] = std::string("入力 JSON を読めません: ") + err.c_str();
    return false;
  }
  for (JsonPairConst kv : in["files"].as<JsonObjectConst>()) {
    if (kv.value().is<const char *>())
      g_files[kv.key().c_str()] = kv.value().as<const char *>();
    else
      serializeJson(kv.value(), g_files[kv.key().c_str()]);
  }
  return true;
}

JsonOut::JsonOut() {
  fflush(stdout);
  fd_ = dup(STDOUT_FILENO);
  dup2(STDERR_FILENO, STDOUT_FILENO);
}

int JsonOut::finish(int code) {
  fflush(stdout);
  std::string text;
  serializeJson(doc, text);
  text += "\n";
  if (write(fd_, text.data(), text.size()) < 0)
    return 2;
  return code;
}

} // namespace host
