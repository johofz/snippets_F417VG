#include "timer.h"
#include <assert.h>

#define TIM_DIR_UP      0
#define TIM_DIR_DOWN    1

volatile uint32_t timeBaseOverflow = 0; // Overflow counter for global timebase timer (TIM6)

void timer_start_global_timebase(TIM_TypeDef* globalTimer)
{
    assert(globalTimer);

    uint32_t arr = 0xFFFF;
    uint32_t timerBaseFreq = SystemCoreClock / 2;

    switch ((uint32_t)globalTimer)
    {
    case (uint32_t)TIM1:
        RCC->APB2ENR |= (RCC_APB2ENR_TIM1EN);
        timerBaseFreq = SystemCoreClock;
        break;

    case (uint32_t)TIM2:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN);
        arr = 0xFFFFFFFF;
        break;

    case (uint32_t)TIM3:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM3EN);
        break;

    case (uint32_t)TIM4:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM4EN);
        break;

    case (uint32_t)TIM5:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM5EN);
        arr = 0xFFFFFFFF;
        break;

    case (uint32_t)TIM6:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM6EN);
        break;

    case (uint32_t)TIM7:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM7EN);
        break;

    case (uint32_t)TIM8:
        RCC->APB2ENR |= (RCC_APB2ENR_TIM8EN);
        timerBaseFreq = SystemCoreClock;
        break;

    case (uint32_t)TIM9:
        RCC->APB2ENR |= (RCC_APB2ENR_TIM9EN);
        timerBaseFreq = SystemCoreClock;
        break;

    case (uint32_t)TIM10:
        RCC->APB2ENR |= (RCC_APB2ENR_TIM10EN);
        timerBaseFreq = SystemCoreClock;
        break;

    case (uint32_t)TIM11:
        RCC->APB2ENR |= (RCC_APB2ENR_TIM11EN);
        timerBaseFreq = SystemCoreClock;
        break;

    case (uint32_t)TIM12:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM12EN);
        break;

    case (uint32_t)TIM13:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM13EN);
        break;

    case (uint32_t)TIM14:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM14EN);
        break;

    default:
        assert(0);
    }

    globalTimer->PSC = timerBaseFreq / 1e6;
    globalTimer->ARR = arr;

    globalTimer->CR1 |= (TIM_CR1_URS);      // Only overflow & underflow trigger update event
    globalTimer->DIER |= (TIM_DIER_UIE);    // Enable update event interrupt

    globalTimer->CR1 |= (TIM_CR1_CEN);      // Start timer
}

void timer_init_it(TIM_TypeDef* timer, uint32_t arr)
{
    assert(timer);

    uint32_t maxArr = 0xFFFF;
    if ((uint32_t)timer == (uint32_t)TIM2 || (uint32_t)timer == (uint32_t)TIM5)
    {
        maxArr = 0xFFFFFFFF;
    }

    assert(arr <= maxArr);

    timer->DIER |= (TIM_DIER_UIE);  // Enable update interrupt
    timer->ARR = arr;
    timer->CR1 |= (TIM_CR1_CEN);
}

void timer_enable_peripheral(TIM_TypeDef* timer)
{
    assert(timer);

    switch ((uint32_t)timer)
    {
    case (uint32_t)TIM1:
        RCC->APB2ENR |= (RCC_APB2ENR_TIM1EN);
        break;
    case (uint32_t)TIM2:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN);
        break;
    case (uint32_t)TIM3:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM3EN);
        break;
    case (uint32_t)TIM4:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM4EN);
        break;
    case (uint32_t)TIM5:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM5EN);
        break;
    case (uint32_t)TIM6:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM6EN);
        break;
    case (uint32_t)TIM7:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM7EN);
        break;
    case (uint32_t)TIM8:
        RCC->APB2ENR |= (RCC_APB2ENR_TIM8EN);
        break;
    case (uint32_t)TIM9:
        RCC->APB2ENR |= (RCC_APB2ENR_TIM9EN);
        break;
    case (uint32_t)TIM10:
        RCC->APB2ENR |= (RCC_APB2ENR_TIM10EN);
        break;
    case (uint32_t)TIM11:
        RCC->APB2ENR |= (RCC_APB2ENR_TIM11EN);
        break;
    case (uint32_t)TIM12:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM12EN);
        break;
    case (uint32_t)TIM13:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM13EN);
        break;
    case (uint32_t)TIM14:
        RCC->APB1ENR |= (RCC_APB1ENR_TIM14EN);
        break;
    default:
        assert(0);
    }
}

void timer_set_dir(TIM_TypeDef* timer, bool dir)
{
    assert(timer);

    timer->CR1 |= (dir << TIM_CR1_DIR_Pos);
}

void timer_update_request_source(TIM_TypeDef* timer, bool urs)
{
    assert(timer);

    timer->CR1 |= (urs << TIM_CR1_URS_Pos);
}

uint64_t micros(void)
{
    return TIM6->CNT + timeBaseOverflow * TIM6->ARR;
}

uint64_t millis(void)
{
    return micros() * 1e3;
}

uint32_t seconds(void)
{
    return micros() * 1e6;
}

/* #################################################################### */
/* ########################### IRQ Handlers ########################### */
/* #################################################################### */

void TIM1_BRK_TIM9_IRQHandler(void)
{
    while (1);
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    while (1);
}

void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    while (1);
}

void TIM1_CC_IRQHandler(void)
{
    while (1);
}

void TIM2_IRQHandler(void)
{
    while (1);
}

void TIM3_IRQHandler(void)
{
    while (1);
}

void TIM4_IRQHandler(void)
{
    while (1);
}

void TIM8_BRK_TIM12_IRQHandler(void)
{
    while (1);
}

void TIM8_UP_TIM13_IRQHandler(void)
{
    while (1);
}

void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    while (1);
}

void TIM8_CC_IRQHandler(void)
{
    while (1);
}

void TIM5_IRQHandler(void)
{
    while (1);
}

void TIM6_DAC_IRQHandler(void)
{
    if (TIM6->SR & (TIM_SR_UIF))
    {
        timeBaseOverflow++;
        TIM6->SR &= ~(TIM_SR_UIF);
    }
}

void TIM7_IRQHandler(void)
{
    if (TIM6->SR & (TIM_SR_UIF))
    {
        TIM6->SR &= ~(TIM_SR_UIF);
    }
}


