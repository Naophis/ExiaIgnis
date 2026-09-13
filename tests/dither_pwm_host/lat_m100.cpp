// 実ファーム構成 (top 1499, M=100, lead 4, guard 100, 1kHz tick, update は tick 開始 50us 後) のレイテンシ
#include <cstdio>
#include <vector>
#include "driver/dither_pwm.hpp"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/structs/dma_debug.h"
static pwm_hw_t g_pwm_regs; pwm_hw_t* pwm_hw=&g_pwm_regs; static dma_hw_t g_dma_regs; dma_hw_t* dma_hw=&g_dma_regs; static dma_debug_hw_t g_dbg; dma_debug_hw_t* dma_debug_hw=&g_dbg;
int main(){
  dpwm::DitherPwm pwm; dpwm::DitherPwm::Config c; c.top=1499; c.samples_per_tick=100; c.lead_samples=4; c.guard_samples=0;
  uint slices[2]={2,3}; g_next_ch=0; if(!pwm.init(c,slices,2)){printf("init fail\n");return 1;}
  uintptr_t base[2]; for(int i=0;i<2;i++) base[i]=dma_hw->ch[pwm.dma_channel(i)].read_addr;
  std::vector<uint16_t> out; uint32_t period=0;
  auto step=[&](){ out.push_back(pwm_hw->slice[2].cc>>16); for(int i=0;i<2;i++){ auto&ch=dma_hw->ch[pwm.dma_channel(i)]; if(ch.started){ uint32_t off=(ch.read_addr-base[i])&1023u; pwm_hw->slice[slices[i]].cc=*(uint32_t*)(base[i]+off); ch.read_addr=base[i]+((off+4)&1023u);} } period++; };
  pwm.set_levels_q16(0,0,0); pwm.set_levels_q16(1,0,0); pwm.start();
  // tick k: 5 周期(50us)経過してから update、残り 95 周期
  uint32_t step_tick=30; uint32_t q_old=dpwm::DitherPwm::q16_from_counts(600.0f), q_new=dpwm::DitherPwm::q16_from_counts(700.0f);
  uint32_t first_new=0, tick_period=0;
  for(uint32_t k=0;k<60;k++){
    for(int p=0;p<5;p++) step();
    uint32_t q=(k>=step_tick)?q_new:q_old; pwm.set_levels_q16(0,0,q); pwm.set_levels_q16(1,0,q); pwm.update();
    if(k==step_tick) tick_period=period;
    for(int p=0;p<95;p++) step();
  }
  for(size_t i=0;i<out.size();i++) if(out[i]>=700){first_new=i;break;}
  const dpwm::DitherStats& st=pwm.stats(0);
  printf("step cmd issued at period %u (tick %u + 5)  first output>=700 at period %u  -> latency = %d periods (%d us @100kHz)\n", tick_period, step_tick, first_new, (int)first_new-(int)tick_period, ((int)first_new-(int)tick_period)*10);
  printf("stats: late=%u early=%u underrun=%u stall=%u\n", st.late_samples, st.early_samples, st.underrun_events, st.stall_ticks);
  return 0;
}
