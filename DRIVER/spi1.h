#ifndef __SPI1_H
#define __SPI1_H

#include <stdint.h>

void spi1_init(void);
int spi1_transmit(uint8_t *send_data, uint8_t *rec_data, uint8_t len);
uint8_t spi1_read_write(uint8_t data);

#endif










