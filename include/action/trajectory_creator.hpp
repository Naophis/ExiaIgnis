#pragma once
// Astraea TrajectoryCreator から移植 (FreeRTOS/ESP-IDF 依存を除去)
// 使用メソッドのみ実装: get_turn_dir / get_turn_type / get_next_dir

#include "planning/astraea_types.hpp"

class TrajectoryCreator {
public:
    TurnDirection get_turn_dir(int turn_num) {
        if (turn_num == 255 || turn_num == 0)
            return TurnDirection::End;
        return (turn_num & 0x01) == 0x01 ? TurnDirection::Right : TurnDirection::Left;
    }

    TurnType get_turn_type(int turn_num) {
        if (turn_num == 1  || turn_num == 2)  return TurnType::Normal;
        if (turn_num == 3  || turn_num == 4)  return TurnType::Orval;
        if (turn_num == 5  || turn_num == 6)  return TurnType::Large;
        if (turn_num == 7  || turn_num == 8)  return TurnType::Dia45;
        if (turn_num == 9  || turn_num == 10) return TurnType::Dia135;
        if (turn_num == 11 || turn_num == 12) return TurnType::Dia90;
        if (turn_num == 13 || turn_num == 14) return TurnType::Kojima;
        if (turn_num == 255)                   return TurnType::Finish;
        return TurnType::Finish;
    }

    TurnType get_turn_type(int turn_num, bool dia) {
        if (turn_num == 1  || turn_num == 2)  return TurnType::Normal;
        if (turn_num == 3  || turn_num == 4)  return TurnType::Orval;
        if (turn_num == 5  || turn_num == 6)  return TurnType::Large;
        if (turn_num == 7  || turn_num == 8)  return !dia ? TurnType::Dia45   : TurnType::Dia45_2;
        if (turn_num == 9  || turn_num == 10) return !dia ? TurnType::Dia135  : TurnType::Dia135_2;
        if (turn_num == 11 || turn_num == 12) return TurnType::Dia90;
        if (turn_num == 13 || turn_num == 14) return TurnType::Kojima;
        if (turn_num == 255)                   return TurnType::Finish;
        return TurnType::Finish;
    }

    Direction get_next_dir(Direction dir, TurnType type, TurnDirection turn_dir) {
        if (type == TurnType::Normal || type == TurnType::Large) {
            if (dir == Direction::North)
                return turn_dir == TurnDirection::Right ? Direction::East  : Direction::West;
            if (dir == Direction::East)
                return turn_dir == TurnDirection::Right ? Direction::South : Direction::North;
            if (dir == Direction::West)
                return turn_dir == TurnDirection::Right ? Direction::North : Direction::South;
            if (dir == Direction::South)
                return turn_dir == TurnDirection::Right ? Direction::West  : Direction::East;
        } else if (type == TurnType::Orval) {
            if (dir == Direction::North) return Direction::South;
            if (dir == Direction::East)  return Direction::West;
            if (dir == Direction::West)  return Direction::East;
            if (dir == Direction::South) return Direction::North;
        } else if (type == TurnType::Dia45) {
            if (dir == Direction::North)
                return turn_dir == TurnDirection::Right ? Direction::NorthEast : Direction::NorthWest;
            if (dir == Direction::East)
                return turn_dir == TurnDirection::Right ? Direction::SouthEast : Direction::NorthEast;
            if (dir == Direction::West)
                return turn_dir == TurnDirection::Right ? Direction::NorthWest : Direction::SouthWest;
            if (dir == Direction::South)
                return turn_dir == TurnDirection::Right ? Direction::SouthWest : Direction::SouthEast;
        } else if (type == TurnType::Dia45_2) {
            if (dir == Direction::NorthEast)
                return turn_dir == TurnDirection::Right ? Direction::East  : Direction::North;
            if (dir == Direction::NorthWest)
                return turn_dir == TurnDirection::Right ? Direction::North : Direction::West;
            if (dir == Direction::SouthEast)
                return turn_dir == TurnDirection::Right ? Direction::South : Direction::East;
            if (dir == Direction::SouthWest)
                return turn_dir == TurnDirection::Right ? Direction::West  : Direction::South;
        } else if (type == TurnType::Dia135) {
            if (dir == Direction::North)
                return turn_dir == TurnDirection::Right ? Direction::SouthEast : Direction::SouthWest;
            if (dir == Direction::East)
                return turn_dir == TurnDirection::Right ? Direction::SouthWest : Direction::NorthWest;
            if (dir == Direction::West)
                return turn_dir == TurnDirection::Right ? Direction::NorthEast : Direction::SouthEast;
            if (dir == Direction::South)
                return turn_dir == TurnDirection::Right ? Direction::NorthWest : Direction::NorthEast;
        } else if (type == TurnType::Dia135_2) {
            if (dir == Direction::NorthEast)
                return turn_dir == TurnDirection::Right ? Direction::South : Direction::West;
            if (dir == Direction::NorthWest)
                return turn_dir == TurnDirection::Right ? Direction::East  : Direction::South;
            if (dir == Direction::SouthEast)
                return turn_dir == TurnDirection::Right ? Direction::West  : Direction::North;
            if (dir == Direction::SouthWest)
                return turn_dir == TurnDirection::Right ? Direction::North : Direction::East;
        } else if (type == TurnType::Dia90) {
            if (dir == Direction::NorthEast)
                return turn_dir == TurnDirection::Right ? Direction::SouthEast : Direction::NorthWest;
            if (dir == Direction::NorthWest)
                return turn_dir == TurnDirection::Right ? Direction::NorthEast : Direction::SouthWest;
            if (dir == Direction::SouthEast)
                return turn_dir == TurnDirection::Right ? Direction::SouthWest : Direction::NorthEast;
            if (dir == Direction::SouthWest)
                return turn_dir == TurnDirection::Right ? Direction::NorthWest : Direction::SouthEast;
        }
        return dir;
    }
};
