#pragma once
#include "pico/types.h"
struct dma_channel_hw_t { volatile uintptr_t read_addr, write_addr; io_rw_32 transfer_count, ctrl_trig; uint32_t dreq; bool started; };
struct dma_hw_t { dma_channel_hw_t ch[16]; };
extern dma_hw_t* dma_hw;
enum { DMA_SIZE_8, DMA_SIZE_16, DMA_SIZE_32 };
struct dma_channel_config { uint32_t ctrl; uint dreq; uint ring_bits; bool ring_write; bool hp; };
static int g_next_ch = 0;
static inline int dma_claim_unused_channel(bool) { return g_next_ch++; }
static inline dma_channel_config dma_channel_get_default_config(uint) { return dma_channel_config{}; }
static inline void channel_config_set_transfer_data_size(dma_channel_config*, int) {}
static inline void channel_config_set_read_increment(dma_channel_config*, bool) {}
static inline void channel_config_set_write_increment(dma_channel_config*, bool) {}
static inline void channel_config_set_ring(dma_channel_config* c, bool w, uint b) { c->ring_write=w; c->ring_bits=b; }
static inline void channel_config_set_dreq(dma_channel_config* c, uint d) { c->dreq=d; }
static inline void channel_config_set_high_priority(dma_channel_config* c, bool h) { c->hp=h; }
static inline void channel_config_set_irq_quiet(dma_channel_config*, bool) {}
static inline void channel_config_set_chain_to(dma_channel_config*, uint) {}
static inline uint32_t dma_encode_endless_transfer_count() { return 0xffffffffu; }
static inline void dma_channel_configure(uint ch, const dma_channel_config* c, volatile void* w, const volatile void* r, uint32_t cnt, bool trig) {
  dma_hw->ch[ch].write_addr=(uintptr_t)w; dma_hw->ch[ch].read_addr=(uintptr_t)r; dma_hw->ch[ch].transfer_count=cnt; dma_hw->ch[ch].dreq=c->dreq; dma_hw->ch[ch].started=trig; }
static inline void dma_channel_set_read_addr(uint ch, const volatile void* r, bool) { dma_hw->ch[ch].read_addr=(uintptr_t)r; }
static inline void dma_start_channel_mask(uint32_t m) { for (int i=0;i<16;i++) if (m&(1u<<i)) dma_hw->ch[i].started=true; }
static inline void dma_channel_abort(uint ch) { dma_hw->ch[ch].started=false; }
