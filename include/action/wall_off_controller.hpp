#pragma once
// Astraea WallOffController から移植 (FreeRTOS/ESP-IDF 依存・TaskHandle_t を除去)

#include "include/structs.hpp"
#include "planning/astraea_types.hpp"
#include <memory>

class WallOffController {
public:
    WallOffController() = default;

    bool execute_wall_off(TurnDirection td, param_straight_t &ps_front);
    bool execute_wall_off_dia(TurnDirection td, param_straight_t &ps_front,
                              bool &use_oppo_wall, bool &exist_wall);
    MotionResult execute_search_wall_off(param_straight_t &p, bool search_mode);
    float calculate_dia_wall_off_distance(TurnDirection td, TurnType turn_type,
                                          bool &exist_wall);

    void set_input_param_entity(std::shared_ptr<input_param_t> &_param) {
        param = _param;
    }
    void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
        tgt_val = _tgt_val;
    }
    void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity) {
        sensing_result = _entity;
    }

    bool continuous_turn_flag = false;

private:
    std::shared_ptr<input_param_t>             param;
    std::shared_ptr<motion_tgt_val_t>          tgt_val;
    std::shared_ptr<sensing_result_entity_t>   sensing_result;
};
