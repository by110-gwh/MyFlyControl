
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include "i2c1.h"

#include "FreeRTOS.h"
#include "task.h"

#define VL53L1X_ADDRESS 0x29

static uint8_t i2c_buffer[256];

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    int8_t Status = 0;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    memcpy(&i2c_buffer[2], pdata, count);
    if (i2c1_transmit(VL53L1X_ADDRESS, 1, i2c_buffer, count + 2))
        Status = 1;
    return Status;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
    int8_t Status = 0;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    if (i2c1_transmit(VL53L1X_ADDRESS, 1, i2c_buffer, 2)) {
        Status = 1;
        goto done;
    }
    if (i2c1_transmit(VL53L1X_ADDRESS, 0, pdata, count)) {
        Status = 1;
        goto done;
    }
done:
    return Status;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    int8_t Status = 0;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    i2c_buffer[2] = data;
    if (i2c1_transmit(VL53L1X_ADDRESS, 1, i2c_buffer, 3))
        Status = 1;
    return Status;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    int8_t Status = 0;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    i2c_buffer[2] = data >> 8;
    i2c_buffer[2] = data & 0xFF;
    if (i2c1_transmit(VL53L1X_ADDRESS, 1, i2c_buffer, 4))
        Status = 1;
    return Status;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    int8_t Status = 0;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    i2c_buffer[2] = (data >> 24) & 0xFF;
    i2c_buffer[3] = (data >> 16) & 0xFF;
    i2c_buffer[4] = (data >> 8)  & 0xFF;
    i2c_buffer[5] = (data >> 0) & 0xFF;
    if (i2c1_transmit(VL53L1X_ADDRESS, 1, i2c_buffer, 6))
        Status = 1;
    return Status;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
    uint8_t Status = 0;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    if (i2c1_transmit(VL53L1X_ADDRESS, 1, i2c_buffer, 2)) {
        Status = 1;
        goto done;
    }
    if (i2c1_transmit(VL53L1X_ADDRESS, 0, data, 1)) {
        Status = 1;
        goto done;
    }
done:
    return Status;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
    uint8_t Status = 0;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    if (i2c1_transmit(VL53L1X_ADDRESS, 1, i2c_buffer, 2)) {
        Status = 1;
        goto done;
    }
    if (i2c1_transmit(VL53L1X_ADDRESS, 0, (uint8_t *)i2c_buffer, 2)) {
        Status = 1;
        goto done;
    }
    *data = ((uint16_t)i2c_buffer[0] << 8) + (uint16_t)i2c_buffer[1];
done:
    return Status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
    uint8_t Status = 0;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    if (i2c1_transmit(VL53L1X_ADDRESS, 1, i2c_buffer, 2)) {
        Status = 1;
        goto done;
    }
    if (i2c1_transmit(VL53L1X_ADDRESS, 0, (uint8_t *)i2c_buffer, 4)) {
        Status = 1;
        goto done;
    }
    *data = ((uint32_t)i2c_buffer[0] << 24) + ((uint32_t)i2c_buffer[1] << 16) + ((uint32_t)i2c_buffer[2] << 8) + (uint32_t)i2c_buffer[3];
done:
    return Status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	uint8_t status  = 0;
	vTaskDelay((wait_ms / portTICK_RATE_MS));
	return status;
}
