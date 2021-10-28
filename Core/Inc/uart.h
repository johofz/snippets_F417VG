#ifndef __UART_H__
#define __UART_H__

void usart_init_async(USART_TypeDef* usart);
void usart_register_irq(USART_TypeDef* usart, uint32_t priority);
void usart_set_baude(USART_TypeDef* usart, uint32_t baude);

#endif /*__ UART_H__ */