#ifndef __I2C_H
#define __I2C_H

#include <stdint.h>

void i2c_init(void);
int i2c_transmit(uint8_t slave_address, uint8_t is_write, uint8_t *pdata, uint8_t count);
int i2c_single_write(uint8_t slave_address,uint8_t reg_address,uint8_t reg_data);
int i2c_single_write_it(uint8_t slave_address,uint8_t reg_address,uint8_t reg_data);
int i2c_single_read(uint8_t slave_address,uint8_t reg_address,uint8_t *reg_data);
int i2c_single_read_it(uint8_t slave_address,uint8_t reg_address,uint8_t *reg_data);
int i2c_multi_read(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt);
int i2c_multi_read_it(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt);
int i2c_multi_write(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt);
int i2c_multi_write_it(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt);

#endif










