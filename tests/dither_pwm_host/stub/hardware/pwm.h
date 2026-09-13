#pragma once
#include "pico/types.h"
struct pwm_slice_hw_t { io_rw_32 csr, div, ctr, cc, top; };
struct pwm_hw_t { pwm_slice_hw_t slice[12]; io_rw_32 en; };
extern pwm_hw_t* pwm_hw;
struct pwm_config { uint32_t csr, div, top; };
static inline pwm_config pwm_get_default_config() { return pwm_config{0,16,0xffff}; }
static inline void pwm_config_set_clkdiv_int_frac4(pwm_config* c, uint8_t i, uint8_t f) { c->div = (i<<4)|f; }
static inline void pwm_config_set_wrap(pwm_config* c, uint16_t w) { c->top = w; }
static inline void pwm_config_set_phase_correct(pwm_config*, bool) {}
static inline void pwm_init(uint s, pwm_config* c, bool start) { pwm_hw->slice[s].csr = start?1:0; pwm_hw->slice[s].div=c->div; pwm_hw->slice[s].top=c->top; pwm_hw->slice[s].ctr=0; pwm_hw->slice[s].cc=0; }
static inline void pwm_set_counter(uint s, uint16_t c) { pwm_hw->slice[s].ctr = c; }
static inline void pwm_set_enabled(uint s, bool e) { if (e) pwm_hw->en |= 1u<<s; else pwm_hw->en &= ~(1u<<s); }
static inline uint pwm_get_dreq(uint s) { return 32 + s; }
