#ifndef __TIMER_H__
#define __TIMER_H__

#include "stm32f4xx.h"
#include "stdbool.h"

void timer_start_global_timebase(TIM_TypeDef* globalTimer);
void timer_init_it(TIM_TypeDef* timer, uint32_t arr);
void timer_enable_peripheral(TIM_TypeDef* timer);
void timer_set_dir(TIM_TypeDef* timer, bool dir);
void timer_update_request_source(TIM_TypeDef* timer, bool urs);


uint64_t micros(void);
uint64_t millis(void);
uint32_t seconds(void);





#endif /*__ TIMER_H__ */