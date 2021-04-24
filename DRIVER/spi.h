#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include <stdint.h>

void spi_init(void);
uint8_t spi_single_wirte_read(uint8_t dat);
void spi_multi_wirte_read(uint8_t deviceNum, uint8_t *out, uint8_t *in, int len);

#endif










