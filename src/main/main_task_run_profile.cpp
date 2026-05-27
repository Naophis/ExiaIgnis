#include "config_loader.hpp"
#include "main/main_task.hpp"
#include <ArduinoJson.h>
#include <stdlib.h>
#include <string.h>

void MainTask::load_circuit_path() {
  const char *fileName = (sys_.hf_cl == 0) ? "/circuit.hf" : "/circuit.cl";

  JsonDocument doc;
  if (!ConfigLoader::load_file(fileName, doc)) {
    printf("not found\n");
    return;
  }

  JsonArrayConst path_str  = doc["path_str"].as<JsonArrayConst>();
  JsonArrayConst path_turn = doc["path_turn"].as<JsonArrayConst>();
  int path_size = (int)path_str.size();
  printf("path_size: %d\n", path_size);

  pc->other_route_map.clear();
  pc->path_s.clear();
  pc->path_t.clear();
  for (int i = 0; i < path_size; i++) {
    pc->path_s.emplace_back(path_str[i].as<float>());
    pc->path_t.emplace_back(static_cast<unsigned char>(path_turn[i].as<int>()));
  }

  param_->sen_ref_p.normal.exist.left45 =
      doc["sensor_exist_left45"] | param_->sen_ref_p.normal.exist.left45;
  param_->wall_off_wait_dist =
      doc["wall_off_wait_dist"] | param_->wall_off_wait_dist;
  param_->front_dist_offset2 =
      doc["front_dist_offset2"] | param_->front_dist_offset2;
  param_->front_dist_offset3 =
      doc["front_dist_offset3"] | param_->front_dist_offset3;
  param_->front_dist_offset4 =
      doc["front_dist_offset4"] | param_->front_dist_offset4;
  param_->wall_off_dist.div_th_r =
      param_->wall_off_dist.div_th_r2 =
          param_->wall_off_dist.div_th_r3 =
              doc["wall_off_hold_div_th_r"] | param_->wall_off_dist.div_th_r;
  param_->wall_off_diff_ref_front_th =
      doc["wall_off_diff_ref_front_th"] | param_->wall_off_diff_ref_front_th;
  param_->wall_off_front_move_dist_th =
      doc["wall_off_front_move_dist_th"] | param_->wall_off_front_move_dist_th;
  param_->wall_off_dist.diff_check_dist =
      doc["wall_off_diff_check_dist"] | param_->wall_off_dist.diff_check_dist;
  param_->wall_off_dist.right_diff_th =
      doc["wall_off_noexist_diff_r"] | param_->wall_off_dist.right_diff_th;
  param_->wall_off_dist.noexist_th_r =
      param_->wall_off_dist.noexist_th_r2 =
          doc["wall_off_hold_noexist_th_r"] | param_->wall_off_dist.noexist_th_r;
}

void MainTask::exec_param_prof() {
  const char *fileName = (sys_.hf_cl == 0) ? "/run_prf.hf" : "/run_prf.cl";

  JsonDocument doc;
  if (!ConfigLoader::load_file(fileName, doc)) {
    printf("not found\n");
    return;
  }

  exec_param_list.clear();
  JsonArrayConst exec_prof = doc["exec_prof"].as<JsonArrayConst>();
  printf("exec_prof_size: %d\n", (int)exec_prof.size());

  for (JsonVariantConst item : exec_prof) {
    exec_pram_t ep{};
    ep.fast_idx   = item["fast"]   | 0;
    ep.normal_idx = item["normal"] | 0;
    ep.slow_idx   = item["slow"]   | 0;
    exec_param_list.emplace_back(ep);
  }
  printf("exec_prof_size2: %d\n", (int)exec_param_list.size());
}

static void write_maze_file(const char *path, const std::vector<uint8_t> &map,
                            bool write) {
  std::string data;
  if (write) {
    for (const auto d : map) {
      data += std::to_string(d);
      data += ',';
    }
  } else {
    printf("delete maze data\n");
    data = "null";
  }
  if (!ConfigLoader::write_file(path,
                                reinterpret_cast<const uint8_t *>(data.c_str()),
                                data.size())) {
    printf("[main] write_maze_file: write failed (%s)\n", path);
  }
}

void MainTask::save_maze_data(bool write) {
  write_maze_file(maze_log_file.c_str(), lgc->map, write);

  printf("wall: [");
  for (int x = 0; x < sys_.maze_size; x++) {
    for (int y = 0; y < sys_.maze_size; y++) {
      auto d = lgc->map[x + y * sys_.maze_size];
      printf("%d,", (d & 0x0f));
    }
  }
  printf("]\n");

  printf("wall2: [");
  for (int x = 0; x < sys_.maze_size; x++) {
    for (int y = 0; y < sys_.maze_size; y++) {
      auto d = lgc->map[x + y * sys_.maze_size];
      printf("%d,", (d & 0xff));
    }
    printf("\n");
  }
  printf("]\n");
}

void MainTask::save_maze_kata_data(bool write) {
  write_maze_file(maze_log_kata_file.c_str(), lgc->map, write);
}

void MainTask::save_maze_return_data(bool write) {
  write_maze_file(maze_log_return_file.c_str(), lgc->map, write);
}

void MainTask::read_maze_data() {
  int32_t sz = ConfigLoader::file_size(maze_log_file.c_str());
  if (sz < 0) return;

  char *raw = static_cast<char *>(malloc(sz + 1));
  if (!raw) return;

  size_t out_size = 0;
  if (!ConfigLoader::read_file_raw(maze_log_file.c_str(),
                                   reinterpret_cast<uint8_t *>(raw), sz,
                                   out_size)) {
    free(raw);
    return;
  }
  raw[out_size] = '\0';
  printf("%s\n", raw);

  if (strncmp(raw, "null", 4) == 0) {
    free(raw);
    return;
  }

  std::vector<int> vals;
  char *tok = strtok(raw, ",");
  while (tok) {
    vals.push_back(atoi(tok));
    tok = strtok(nullptr, ",");
  }
  free(raw);

  printf("map_list.size = %d\n", (int)vals.size());
  for (int i = 0; i < (int)vals.size(); i++) {
    lgc->set_native_wall_data(i, vals[i]);
  }
  printf("read maze data!!!\n");

  printf("wall: [");
  for (int x = 0; x < sys_.maze_size; x++) {
    for (int y = 0; y < sys_.maze_size; y++) {
      auto d = lgc->map[x + y * sys_.maze_size];
      printf("%d,", (d & 0x0f));
    }
  }
  printf("]\n");

  printf("wall2: [");
  for (int x = 0; x < sys_.maze_size; x++) {
    for (int y = 0; y < sys_.maze_size; y++) {
      auto d = lgc->map[x + y * sys_.maze_size];
      printf("%d,", (d & 0xff));
    }
    printf("\n");
  }
  printf("]\n");

  printf("map___\n");
  sleep_ms(1);
  for (int y = 0; y < sys_.maze_size; y++) {
    for (int x = 0; x < sys_.maze_size; x++) {
      auto d = lgc->map[x + y * sys_.maze_size];
      if (x == (sys_.maze_size - 1) && y == (sys_.maze_size - 1)) {
        printf("%d", (d & 0x0f));
      } else {
        printf("%d,", (d & 0x0f));
      }
    }
    printf("\n");
  }
  printf("\n");
  printf("end___\n");
}
