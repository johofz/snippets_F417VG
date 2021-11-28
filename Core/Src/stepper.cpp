#include "stepper.h"
#include <assert.h>


Stepper::Stepper()
{
}

Stepper::~Stepper()
{
}

void Stepper::Init(StepperInitData* initData)
{
    assert(initData);

    m_port = initData->port;
    m_pin = initData->pin;

    m_signalTimer = initData->signalTimer;
    m_posTimer = initData->posTimer;
    m_dma = initData->dma;

    m_rampUpData = initData->rampUpData;
    m_rampDownData = initData->rampDownData;
    m_rampDataLen = initData->rampDataLen;

    InitTimers();
}

void Stepper::InitTimers()
{
    assert(m_signalTimer && m_posTimer);

    // Signal-Timer initialisieren
    m_signalTimer->CR1 = 0x00;
    m_signalTimer->CR1 |= (TIM_CR1_URS);    // Update request source: Only counter overflow/underflow

    m_signalTimer->CR2 = 0x00;
    m_signalTimer->CR2 |= (2 << TIM_CR2_MMS_Pos);   // Master mode selection: Update

    m_signalTimer->DIER = 0x00;
    m_signalTimer->DIER |= (TIM_DIER_UDE);  // Update DMA request enable

    m_signalTimer->CCMR1 = 0x00;
    m_signalTimer->CCMR1 |= (3 << TIM_CCMR1_OC1M_Pos);  // Output compare 1 mode: Toggle

    m_signalTimer->CCER = 0x00;
}