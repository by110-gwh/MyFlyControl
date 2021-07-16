#ifndef __SPI0_H
#define __SPI0_H

#include <stdint.h>

void spi0_init(void);
int spi0_transmit(uint8_t *send_data, uint8_t *rec_data, uint8_t len);
uint8_t spi0_read_write(uint8_t data);

#endif










