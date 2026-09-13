#pragma once
#include "pico/types.h"
struct dma_debug_ch_t { io_rw_32 dbg_ctdreq; io_rw_32 dbg_tcr; };
struct dma_debug_hw_t { dma_debug_ch_t ch[16]; };
extern dma_debug_hw_t* dma_debug_hw;
