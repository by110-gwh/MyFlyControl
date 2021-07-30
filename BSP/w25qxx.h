#ifndef __W25QXX_H
#define __W25QXX_H 

#include <stdint.h>

void w25qxx_init(void);
uint8_t w25qxx_read_sr(void);
void w25qxx_write_sr(uint8_t sr);
void w25qxx_write_enable(void);
void w25qxx_write_disable(void);
uint16_t w25qxx_read_id(void);
void w25qxx_read(uint8_t *pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
void w25qxx_write_page(uint8_t* buf,uint32_t addr,uint16_t cnt);
void w25qxx_write(uint8_t* buf,uint32_t addr,uint16_t cnt);
void w25qxx_erase_chip(void);
void w25qxx_erase_sector(uint32_t addr);
void w25qxx_wait_busy(void);
void w25qxx_power_down(void);
void w25qxx_wakeup(void);

#endif
