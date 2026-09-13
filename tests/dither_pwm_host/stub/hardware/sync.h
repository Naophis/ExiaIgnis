#pragma once
#include "pico/types.h"
static inline void __dmb() {}
static inline void hw_set_bits(io_rw_32* r, uint32_t m) { *r |= m; }
