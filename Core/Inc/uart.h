#ifndef __UART_H__
#define __UART_H__

#include "stdbool.h"

void usart_init_async(USART_TypeDef* usart);
void usart_register_irq(USART_TypeDef* usart, uint32_t priority);
void usart_set_baude(USART_TypeDef* usart, uint32_t baude);
void usart_init_dma_receive(USART_TypeDef* usart, uint8_t* rxBuf, uint16_t size);
void usart_init_dma_transmit(USART_TypeDef* usart, uint8_t* txBuf, uint16_t size);

uint32_t usart_rcv_timeout(USART_TypeDef* usart, uint8_t* rxBuf, uint16_t size, uint32_t timeout);
uint32_t usart_rcv_until(USART_TypeDef* usart, uint8_t* rxBuf, uint16_t size, char terminator, uint32_t timeout);
void usart_rcv_idle(USART_TypeDef* usart, uint8_t* rxBuf, uint16_t size);
void usart_send(USART_TypeDef* usart, uint8_t* txBuf, uint16_t size);
void usart_send_dma(USART_TypeDef* usart, uint8_t* txBuf, uint16_t size);

#endif /*__ UART_H__ */