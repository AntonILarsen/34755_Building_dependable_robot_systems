#include "timer0.h"
hw_timer_t *timer_cfg = NULL;
extern uint8_t isr_flag;
void IRAM_ATTR Timer0_ISR()
{
  isr_flag = 1;
}
uint64_t setup_timer(uint16_t PRE_SCALER, float TIM_PERIOD, float APB_CLK)
{
    uint64_t TICKS = TIM_PERIOD*APB_CLK/((float) PRE_SCALER);
    bool COUNT_UP = true;
    bool AUTO_RELOAD = true;
    timer_cfg = timerBegin(0, PRE_SCALER, COUNT_UP);
    timerAttachInterrupt(timer_cfg, &Timer0_ISR, true);
    timerAlarmWrite(timer_cfg, TICKS, AUTO_RELOAD);
    timerAlarmEnable(timer_cfg);
    return TICKS;
}