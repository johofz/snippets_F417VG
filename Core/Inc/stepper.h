#pragma once

#include "stm32f4xx.h"

struct StepperInitData
{
    GPIO_TypeDef* port;
    uint16_t pin;

    TIM_TypeDef* signalTimer;
    TIM_TypeDef* posTimer;

    DMA_Stream_TypeDef* dma;

    uint32_t* rampUpData;
    uint32_t* rampDownData;
    uint16_t rampDataLen;
};


class Stepper
{
private:
    GPIO_TypeDef* m_port;
    uint16_t m_pin;

    TIM_TypeDef* m_signalTimer;
    TIM_TypeDef* m_posTimer;

    DMA_Stream_TypeDef* m_dma;

    uint32_t* m_rampUpData;
    uint32_t* m_rampDownData;
    uint16_t m_rampDataLen;

    void InitTimers();

public:
    Stepper();
    ~Stepper();

    void Init(StepperInitData* initData);
};