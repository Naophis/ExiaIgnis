#include "action/wall_off_controller.hpp"

// TODO: Astraea wall_off_controller.cpp から移植
bool WallOffController::execute_wall_off(TurnDirection td,
                                         param_straight_t &ps_front) {
    (void)td; (void)ps_front;
    return true;
}

bool WallOffController::execute_wall_off_dia(TurnDirection td,
                                              param_straight_t &ps_front,
                                              bool &use_oppo_wall,
                                              bool &exist_wall) {
    (void)td; (void)ps_front;
    use_oppo_wall = false;
    exist_wall    = false;
    return true;
}

MotionResult WallOffController::execute_search_wall_off(param_straight_t &p,
                                                         bool search_mode) {
    (void)p; (void)search_mode;
    return MotionResult::NONE;
}

float WallOffController::calculate_dia_wall_off_distance(TurnDirection td,
                                                          TurnType turn_type,
                                                          bool &exist_wall) {
    (void)td; (void)turn_type; (void)exist_wall;
    return 0.0f;
}
