#ifndef _DEBUG_UART_H
#define _DEBUG_UART_H

#include <stdint.h>

//定义最大接收字节数 200
#define USART_REC_LEN 200

extern uint16_t USART_RX_STA;
extern uint8_t USART_RX_BUF[USART_REC_LEN];

void debug_uart_init(void);

#endif
