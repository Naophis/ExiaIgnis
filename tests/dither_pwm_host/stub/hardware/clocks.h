#pragma once
#include "pico/types.h"
enum { clk_sys = 5 };
static inline uint32_t clock_get_hz(int) { return 150000000u; }
