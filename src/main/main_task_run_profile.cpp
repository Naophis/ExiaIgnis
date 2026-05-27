#include "main/main_task.hpp"

void MainTask::load_circuit_path() {
//   mount();
//   string fileName = "/spiflash/circuit.hf";

//   if (sys_.hf_cl == 0) {
//     fileName = "/spiflash/circuit.hf";
//   } else {
//     fileName = "/spiflash/circuit.cl";
//   }

//   std::ifstream ifs(fileName);
//   if (!ifs) {
//     printf("not found\n");
//     return;
//   }
//   std::string str;
//   std::string buf;
//   while (!ifs.eof()) {
//     std::getline(ifs, buf);
//     str += buf;
//   }
//   ifs.close();
//   cJSON *root = cJSON_CreateObject(), *path_str, *path_turn;
//   root = cJSON_Parse(str.c_str());
//   path_str = getItem(root, "path_str");
//   path_turn = getItem(root, "path_turn");
//   int path_size = cJSON_GetArraySize(path_str);
//   printf("path_size: %d\n", path_size);
//   pc->other_route_map.clear();
//   pc->path_s.clear();
//   pc->path_t.clear();
//   for (int i = 0; i < path_size; i++) {
//     float str = cJSON_GetArrayItem(path_str, i)->valuedouble;
//     int turn = cJSON_GetArrayItem(path_turn, i)->valueint;
//     pc->path_s.emplace_back(str);
//     pc->path_t.emplace_back(turn);
//   }

//   param->sen_ref_p.normal.exist.left45        //
//       = param->sen_ref_p.normal.expand.left45 //
//       = getItem(root, "sensor_exist_left45")->valuedouble;
//   param->wall_off_wait_dist = getItem(root, "wall_off_wait_dist")->valuedouble;
//   param->front_dist_offset2 = getItem(root, "front_dist_offset2")->valuedouble;
//   param->front_dist_offset3 = getItem(root, "front_dist_offset3")->valuedouble;
//   param->front_dist_offset4 = getItem(root, "front_dist_offset4")->valuedouble;
//   param->wall_off_dist.div_th_r =      //
//       param->wall_off_dist.div_th_r2 = //
//       param->wall_off_dist.div_th_r3 = //
//       getItem(root, "wall_off_hold_div_th_r")->valuedouble;
//   param->wall_off_diff_ref_front_th =
//       getItem(root, "wall_off_diff_ref_front_th")->valuedouble;
//   param->wall_off_front_move_dist_th =
//       getItem(root, "wall_off_front_move_dist_th")->valuedouble;
//   param->wall_off_dist.diff_check_dist =
//       getItem(root, "wall_off_diff_check_dist")->valuedouble;
//   param->wall_off_dist.right_diff_th =
//       getItem(root, "wall_off_noexist_diff_r")->valuedouble;
//   param->wall_off_dist.noexist_th_r =      //
//       param->wall_off_dist.noexist_th_r2 = //
//       getItem(root, "wall_off_hold_noexist_th_r")->valuedouble;

//   cJSON_Delete(root);
//   umount();
}

void MainTask::exec_param_prof() {
  mount();
  string fileName = "/spiflash/run_prf.txt";
  if (sys_.hf_cl == 0) {
    fileName = "/spiflash/run_prf.hf";
  } else {
    fileName = "/spiflash/run_prf.cl";
  }
  std::ifstream ifs(fileName);
  if (!ifs) {
    printf("not found\n");
    return;
  }
  std::string str;
  std::string buf;
  while (!ifs.eof()) {
    std::getline(ifs, buf);
    str += buf;
  }
  ifs.close();
  exec_param_list.clear();
  cJSON *root = cJSON_CreateObject(), *exec_prof;
  root = cJSON_Parse(str.c_str());
  exec_prof = getItem(root, "exec_prof");
  int exec_prof_size = cJSON_GetArraySize(exec_prof);
  printf("exec_prof_size: %d\n", exec_prof_size);

  for (int i = 0; i < exec_prof_size; i++) {
    const auto prof = cJSON_GetArrayItem(exec_prof, i);
    exec_pram_t ep;
    ep.fast_idx = getItem(prof, "fast")->valueint;     // 0
    ep.normal_idx = getItem(prof, "normal")->valueint; // 1
    ep.slow_idx = getItem(prof, "slow")->valueint;     // 2
    exec_param_list.emplace_back(ep);
  }
  printf("exec_prof_size2: %d\n", exec_param_list.size());
  cJSON_Delete(root);
  umount();
}


void MainTask::save_maze_data(bool write) {
  mount();
  auto *f = fopen(maze_log_file.c_str(), "wb");
  if (f == NULL) {
    umount();
    return;
  }
  if (write) {
    for (const auto d : lgc->map) {
      fprintf(f, "%d,", d);
    }
  } else {
    printf("delete maze data\n");
    fprintf(f, "null");
  }
  fflush(f);
  fclose(f);

  printf("wall: [");
  // for (const auto d : lgc->map) {
  for (int x = 0; x < sys_.maze_size; x++) {
    for (int y = 0; y < sys_.maze_size; y++) {
      auto d = lgc->map[x + y * sys_.maze_size];
      printf("%d,", (d & 0x0f));
    }
  }
  printf("]\n");

  printf("wall2: [");
  // for (const auto d : lgc->map) {
  for (int x = 0; x < sys_.maze_size; x++) {
    for (int y = 0; y < sys_.maze_size; y++) {
      auto d = lgc->map[x + y * sys_.maze_size];
      printf("%d,", (d & 0xff));
    }
    printf("\n");
  }
  printf("]\n");
  umount();
}
void MainTask::save_maze_kata_data(bool write) {
  mount();
  auto *f = fopen(maze_log_kata_file.c_str(), "wb");
  if (f == NULL) {
    umount();
    return;
  }
  if (write) {
    for (const auto d : lgc->map) {
      fprintf(f, "%d,", d);
    }
  } else {
    printf("delete maze data\n");
    fprintf(f, "null");
  }
  fflush(f);
  fclose(f);
  umount();
}
void MainTask::save_maze_return_data(bool write) {
  mount();
  auto *f = fopen(maze_log_return_file.c_str(), "wb");
  if (f == NULL) {
    umount();
    return;
  }
  if (write) {
    for (const auto d : lgc->map) {
      fprintf(f, "%d,", d);
    }
  } else {
    printf("delete maze data\n");
    fprintf(f, "null");
  }
  fflush(f);
  fclose(f);
  umount();
}

void MainTask::read_maze_data() {
  mount();
  auto *f = fopen(maze_log_file.c_str(), "rb");
  if (f == NULL) {
    umount();
    return;
  }
  // char line_buf[LINE_BUF_SIZE];
  std::string str = "";
  while (fgets(line_buf, sizeof(line_buf), f) != NULL) {
    printf("%s", line_buf);
    // printf("_______\n");
    str += std::string(line_buf);
  }
  printf("\n");
  fflush(f);
  fclose(f);
  // std::string str = std::string(line_buf);
  if (str == "null")
    return;
  auto map_list = split(str, ',');
  printf("map_list.size = %d\n", map_list.size());
  for (int i = 0; i < map_list.size(); i++) {
    lgc->set_native_wall_data(i, stoi(map_list[i]));
  }
  printf("read maze data!!!\n");

  printf("wall: [");
  // for (const auto d : lgc->map) {
  for (int x = 0; x < sys_.maze_size; x++) {
    for (int y = 0; y < sys_.maze_size; y++) {
      auto d = lgc->map[x + y * sys_.maze_size];
      printf("%d,", (d & 0x0f));
    }
  }
  printf("]\n");

  printf("wall2: [");
  // for (const auto d : lgc->map) {
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
  printf("end___\n"); // ファイル追記終了トリガー
  umount();
}