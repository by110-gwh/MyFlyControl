#ifndef _UART0_H
#define _UART0_H

#include <stdint.h>

void uart0_init(void);
void uart0_send_data(uint8_t data);

#endif
