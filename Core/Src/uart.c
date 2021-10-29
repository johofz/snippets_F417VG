/*
 * USART Code-Snippets
 * MCU: STMF4xx
 * Editor: Johanes Höfler
 * (Exaples for USART1)
 */

#include "stm32f4xx.h"
#include "uart.h"
#include "timer.h"
#include <assert.h>

#define NVIC_MAX_PRIORITY   15
#define MAX_RX_BUF_SIZE     0xFFFF
#define MAX_TX_BUF_SIZE     0xFFFF
#define MAX_BAUDE_RATE      115200

 /* Simple USART initialisation */
void usart_init_async(USART_TypeDef* usart)
{
    assert(usart);

    switch ((uint32_t)usart)
    {
    case (uint32_t)USART1:
    {
        RCC->APB2ENR |= (RCC_APB2ENR_USART1EN); // enable USART peripheral
        RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);  // enable GPIO peripheral

        GPIOA->MODER |= (2 << 18);  // PA9 (TX) mode: alternate function
        GPIOA->MODER |= (2 << 20);  // PA10 (RX) mode: alternate function
        GPIOA->AFR[1] |= (7 << 4);  // PA9 (TX) alternate function: AF7 (USART1_TX)
        GPIOA->AFR[1] |= (7 << 8);  // PA10 (RX) alternate function: AF7 (USART1_RX)
        break;
    }

    case (uint32_t)USART2:
    {
        RCC->APB1ENR |= (RCC_APB1ENR_USART2EN); // enable USART peripheral
        RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);  // enable GPIO peripheral

        GPIOA->MODER |= (2 << 4);  // PA2 (TX) mode: alternate function
        GPIOA->MODER |= (2 << 6);  // PA3 (RX) mode: alternate function
        GPIOA->AFR[0] |= (7 << 8);  // PA2 (TX) alternate function: AF7 (USART2_TX)
        GPIOA->AFR[0] |= (7 << 12);  // PA3 (RX) alternate function: AF7 (USART2_RX)
        break;
    }

    case (uint32_t)USART3:
    {
        RCC->APB1ENR |= (RCC_APB1ENR_USART3EN); // enable USART peripheral
        RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);  // enable GPIO peripheral

        GPIOB->MODER |= (2 << 20);  // PB10 (TX) mode: alternate function
        GPIOB->MODER |= (2 << 22);  // PB11 (RX) mode: alternate function
        GPIOB->AFR[1] |= (7 << 8);  // PB10 (TX) alternate function: AF7 (USART3_TX)
        GPIOB->AFR[1] |= (7 << 12); // PB11 (RX) alternate function: AF7 (USART3_RX)
        break;
    }

    case (uint32_t)USART6:
    {
        RCC->APB2ENR |= (RCC_APB2ENR_USART6EN); // enable USART peripheral
        RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);  // enable GPIO peripheral

        GPIOC->MODER |= (2 << 12);  // PC6 (TX) mode: alternate function
        GPIOC->MODER |= (2 << 14);  // PC7 (RX) mode: alternate function
        GPIOC->AFR[0] |= (7 << 24); // PC6 (TX) alternate function: AF7 (USART6_TX)
        GPIOC->AFR[0] |= (7 << 28); // PC7 (RX) alternate function: AF7 (USART6_RX)
        break;
    }

    case (uint32_t)UART4:
    {
        RCC->APB1ENR |= (RCC_APB1ENR_UART4EN); // enable UART peripheral
        RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);  // enable GPIO peripheral

        GPIOA->MODER |= (2 << 0);  // PA0 (TX) mode: alternate function
        GPIOA->MODER |= (2 << 2);  // PA1 (RX) mode: alternate function
        GPIOA->AFR[0] |= (8 << 0);  // PA0 (TX) alternate function: AF8 (UART4_TX)
        GPIOA->AFR[0] |= (8 << 4);  // PA1 (RX) alternate function: AF8 (UART4_RX)
        break;
    }

    case (uint32_t)UART5:
    {
        RCC->APB1ENR |= (RCC_APB1ENR_UART5EN); // enable UART peripheral
        RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);  // enable GPIO peripheral
        RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN);  // enable GPIO peripheral

        GPIOC->MODER |= (2 << 24);  // PC12 (TX) mode: alternate function
        GPIOD->MODER |= (2 << 4);  // PD2 (RX) mode: alternate function
        GPIOC->AFR[1] |= (8 << 16);  // PC12 (TX) alternate function: AF8 (UART5_TX)
        GPIOD->AFR[0] |= (8 << 8);  // PD2 (RX) alternate function: AF8 (UART5_RX)
        break;
    }

    default:
        assert(0);
    }

    usart->CR1 = 0x00;  // clear control register 1
    usart->CR1 |= (USART_CR1_RE | USART_CR1_TE);    // enable reciver & transmitter
    usart->CR1 |= (USART_CR1_UE);   // enable USART
}

void usart_register_irq(USART_TypeDef* usart, uint32_t priority)
{
    assert(usart && priority <= NVIC_MAX_PRIORITY);

    switch ((uint32_t)usart)
    {
    case (uint32_t)USART1:
        __NVIC_SetPriority(USART1_IRQn, priority);  // set interrupt priority
        __NVIC_EnableIRQ(USART1_IRQn);	            // enable interrupt
        break;

    case (uint32_t)USART2:
        __NVIC_SetPriority(USART2_IRQn, priority);
        __NVIC_EnableIRQ(USART2_IRQn);
        break;

    case (uint32_t)USART3:
        __NVIC_SetPriority(USART3_IRQn, priority);
        __NVIC_EnableIRQ(USART3_IRQn);
        break;

    case (uint32_t)USART6:
        __NVIC_SetPriority(USART6_IRQn, priority);
        __NVIC_EnableIRQ(USART6_IRQn);
        break;

    case (uint32_t)UART4:
        __NVIC_SetPriority(UART4_IRQn, priority);
        __NVIC_EnableIRQ(UART4_IRQn);
        break;

    case (uint32_t)UART5:
        __NVIC_SetPriority(UART5_IRQn, priority);
        __NVIC_EnableIRQ(UART5_IRQn);
        break;

    default:
        assert(0);
    }
}

void usart_set_baude(USART_TypeDef* usart, uint32_t baude)
{
    assert(usart && baude <= MAX_BAUDE_RATE);

    float usartDiv = (float)SystemCoreClock / ((float)baude * 16.0f);
    uint16_t mentissa = (uint16_t)usartDiv;
    uint16_t fraction = (uint16_t)((usartDiv - mentissa) * 16.0f);
    usart->BRR |= (mentissa << 4);
    usart->BRR |= (fraction << 0);
}

void usart_init_dma_receive(USART_TypeDef* usart, uint8_t* rxBuf, uint16_t size)
{
    assert(usart && rxBuf && size <= MAX_RX_BUF_SIZE);

    DMA_Stream_TypeDef* dma;

    switch ((uint32_t)usart)
    {
    case (uint32_t)USART1:
    {
        RCC->AHB1ENR |= (RCC_AHB1ENR_DMA2EN);
        __NVIC_SetPriority(DMA2_Stream2_IRQn, 0);
        __NVIC_EnableIRQ(DMA2_Stream2_IRQn);
        dma = DMA2_Stream2;
        break;
    }
    case (uint32_t)USART2:
    {
        RCC->AHB1ENR |= (RCC_AHB1ENR_DMA1EN);
        __NVIC_SetPriority(DMA1_Stream5_IRQn, 0);
        __NVIC_EnableIRQ(DMA1_Stream5_IRQn);
        dma = DMA1_Stream5;
        break;
    }
    case (uint32_t)USART3:
    {
        RCC->AHB1ENR |= (RCC_AHB1ENR_DMA1EN);
        __NVIC_SetPriority(DMA1_Stream1_IRQn, 0);
        __NVIC_EnableIRQ(DMA1_Stream1_IRQn);
        dma = DMA1_Stream1;
        break;
    }
    case (uint32_t)USART6:
    {
        RCC->AHB1ENR |= (RCC_AHB1ENR_DMA2EN);
        __NVIC_SetPriority(DMA2_Stream1_IRQn, 0);
        __NVIC_EnableIRQ(DMA2_Stream1_IRQn);
        dma = DMA2_Stream1;
        break;
    }
    case (uint32_t)UART4:
    {
        RCC->AHB1ENR |= (RCC_AHB1ENR_DMA1EN);
        __NVIC_SetPriority(DMA1_Stream2_IRQn, 0);
        __NVIC_EnableIRQ(DMA1_Stream2_IRQn);
        dma = DMA1_Stream2;
        break;
    }
    case (uint32_t)UART5:
    {
        RCC->AHB1ENR |= (RCC_AHB1ENR_DMA1EN);
        __NVIC_SetPriority(DMA1_Stream0_IRQn, 0);
        __NVIC_EnableIRQ(DMA1_Stream0_IRQn);
        dma = DMA1_Stream0;
        break;
    }

    default:
        assert(0);
    }

    dma->CR |= (DMA_SxCR_EN);   // disable DMA stream

    dma->CR &= ~((DMA_SxCR_CHSEL_Msk) |     // Channel selection: channel 0
        (DMA_SxCR_MBURST_Msk) |             // Memory burst transfer configuration: single transfer
        (DMA_SxCR_PBURST_Msk) |             // Peripheral burst transfer configuration: single transfer
        (DMA_SxCR_CT) |                     // Current target (only in double buffer mode)
        (DMA_SxCR_DBM) |                    // Double buffer mode: No buffer switching at the end of transfer
        (DMA_SxCR_PINCOS) |                 // Peripheral increment offset size: Linked to the PSIZE
        (DMA_SxCR_MSIZE_Msk) |              // Memory data size: Byte (8-bit)
        (DMA_SxCR_PSIZE_Msk) |              // Peripheral data size: Byte (8-bit)
        (DMA_SxCR_PINC) |                   // Peripheral increment mode: Peripheral address pointer is fixed
        (DMA_SxCR_CIRC) |                   // Circular mode: disabled
        (DMA_SxCR_DIR_Msk) |                // Data transfer direction: Peripheral-to-memory
        (DMA_SxCR_PFCTRL));                 // Peripheral flow controller: DMA is the flow controller

    dma->CR |= ((1 << DMA_SxCR_PL_Pos) |    // Priority level: meduim
        (DMA_SxCR_MINC) |                   // Memory increment mode: memory address pointer is incremented
        (DMA_SxCR_TCIE) |                   // Transfer complete interrupt: enabled
        (DMA_SxCR_HTIE) |                   // Half transfer interrupt: enabled
        (DMA_SxCR_TEIE) |                   // Transfer error interrupt: enabled
        (DMA_SxCR_DMEIE));                  // Direct mode error interrupt: enabled

    dma->NDTR = size;                   // Size of the RX buffer
    dma->PAR = (uint32_t)&usart->DR;    // Address of the Peripheral register to read from
    dma->M0AR = (uint32_t)rxBuf;        // Address of the RX buffer to store data in

    dma->CR |= (DMA_SxCR_EN);   // enable DMA
}

uint32_t usart_rcv_timeout(USART_TypeDef* usart, uint8_t* rxBuf, uint16_t size, uint32_t timeout)
{
    assert(usart && rxBuf && size <= MAX_RX_BUF_SIZE);

    uint64_t endTime = millis() + timeout;
    uint32_t bytesRcvd = 0;

    for (int i = 0; i < size; i++)
    {
        rxBuf[i] = usart->DR;
        bytesRcvd++;
        while (!(usart->SR & (USART_SR_RXNE))) // Wait for new data in data register
        {
            if (millis() > endTime) // check if timeout condition is met
            {
                return bytesRcvd;
            }
        }
    }

    return bytesRcvd;
}

uint32_t usart_rcv_until(USART_TypeDef* usart, uint8_t* rxBuf, uint16_t size, char terminator, uint32_t timeout)
{
    assert(usart && rxBuf && size <= MAX_RX_BUF_SIZE);

    uint64_t endTime = millis() + timeout;
    uint32_t bytesRcvd = 0;

    for (int i = 0; i < size; i++)
    {
        rxBuf[i] = usart->DR;
        bytesRcvd++;

        if (rxBuf[i] == terminator)
        {
            break;
        }

        while (!(usart->SR & (USART_SR_RXNE))) // Wait for new data in data register
        {
            if (millis() > endTime) // check if timeout condition is met
            {
                return bytesRcvd;
            }
        }
    }

    return bytesRcvd;
}

void usart_rcv_idle(USART_TypeDef* usart, uint8_t* rxBuf, uint16_t size)
{
    usart_init_async(usart);
    usart_register_irq(usart, 0);
    usart_set_baude(usart, 115200);

    usart->CR1 |= (USART_CR1_IDLEIE);

    usart_init_dma_receive(usart, rxBuf, size);
}

void usart_send(USART_TypeDef* usart, uint8_t* txBuf, uint16_t size)
{
    assert(usart);
    assert(txBuf);
    assert(size <= MAX_TX_BUF_SIZE);

    for (int i = 0; i < size; i++)
    {
        usart->DR = txBuf[i];
        while (!(usart->SR & (USART_SR_TXE))); // Wait for byte to be send to the shift register
    }

    while (usart->SR & (USART_SR_TC));  // Wait for last byte to be transferred
}

void usart_send_dma(USART_TypeDef* usart, uint8_t* txBuf, uint16_t size)
{

}