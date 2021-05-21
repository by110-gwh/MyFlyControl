
/*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file is part of VL53L1 Core and is dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
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
*
********************************************************************************
*
*/


#include "vl53l1_platform.h"
// #include "vl53l1_platform_log.h"
#include "vl53l1_api.h"

// #include "stm32xxx_hal.h"
#include <string.h>
// #include <time.h>
// #include <math.h>
#include "i2c.h"

#include "FreeRTOS.h"
#include "task.h"

#define VL53L1X_ADDRESS 0x

// #define I2C_TIME_OUT_BASE   10
// #define I2C_TIME_OUT_BYTE   1

// #ifdef VL53L1_LOG_ENABLE
// #define trace_print(level, ...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_PLATFORM, level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)
// #define trace_i2c(...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_NONE, VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)
// #endif

// #ifndef HAL_I2C_MODULE_ENABLED
// #warning "HAL I2C module must be enable "
// #endif

//extern I2C_HandleTypeDef hi2c1;
//#define VL53L0X_pI2cHandle    (&hi2c1)

/* when not customized by application define dummy one */
// #ifndef VL53L1_GetI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
// #   define VL53L1_GetI2cBus(...) (void)0
// #endif

// #ifndef VL53L1_PutI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
// #   define VL53L1_PutI2cBus(...) (void)0
// #endif

uint8_t i2c_buffer[256];

// int _I2CWrite(VL53L1_DEV Dev, uint8_t *pdata, uint32_t count) {
//     int status = 0;
//     return status;
// }

// int _I2CRead(VL53L1_DEV Dev, uint8_t *pdata, uint32_t count) {
//    int status = 0;
//    return Status;
// }

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    memcpy(&i2c_buffer[2], pdata, count);
    if (i2c_transmit(0x29, 1, i2c_buffer, count + 2))
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    if (i2c_transmit(0x29, 1, i2c_buffer, 2)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    if (i2c_transmit(0x29, 0, pdata, count)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
done:
    return Status;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    i2c_buffer[2] = data;
    if (i2c_transmit(0x29, 1, i2c_buffer, 3))
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    return Status;
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    i2c_buffer[2] = data >> 8;
    i2c_buffer[2] = data & 0xFF;
    if (i2c_transmit(0x29, 1, i2c_buffer, 4))
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    return Status;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    i2c_buffer[2] = (data >> 24) & 0xFF;
    i2c_buffer[3] = (data >> 16) & 0xFF;
    i2c_buffer[4] = (data >> 8)  & 0xFF;
    i2c_buffer[5] = (data >> 0) & 0xFF;
    if (i2c_transmit(0x29, 1, i2c_buffer, 6))
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    return Status;
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    uint8_t data;

    Status = VL53L1_RdByte(Dev, index, &data);
    if (Status) {
        goto done;
    }
    data = (data & AndData) | OrData;
    Status = VL53L1_WrByte(Dev, index, data);
done:
    return Status;
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    if (i2c_transmit(0x29, 1, i2c_buffer, 2)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    if (i2c_transmit(0x29, 0, data, 1)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
done:
    return Status;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    if (i2c_transmit(0x29, 1, i2c_buffer, 2)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    if (i2c_transmit(0x29, 0, (uint8_t *)i2c_buffer, 2)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    *data = ((uint16_t)i2c_buffer[0] << 8) + (uint16_t)i2c_buffer[1];
done:
    return Status;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    i2c_buffer[0] = index >> 8;
    i2c_buffer[1] = index & 0xFF;
    if (i2c_transmit(0x29, 1, i2c_buffer, 2)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    if (i2c_transmit(0x29, 0, (uint8_t *)i2c_buffer, 4)) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    *data = ((uint32_t)i2c_buffer[0] << 24) + ((uint32_t)i2c_buffer[1] << 16) + ((uint32_t)i2c_buffer[2] << 8) + (uint32_t)i2c_buffer[3];
done:
    return Status;
}

VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;
    *ptick_count_ms = xTaskGetTickCount();
	return status;
}

//#define trace_print(level, ...) \
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_PLATFORM, \
//	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

//#define trace_i2c(...) \
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_NONE, \
//	VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	return status;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms){
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	vTaskDelay((wait_ms / portTICK_RATE_MS));
	return status;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us){
	VL53L1_Error status  = VL53L1_ERROR_NONE;
    uint32_t delay_ms = (wait_us + 900) / 1000;
    if(delay_ms == 0) {
        delay_ms = 1;
    }
	vTaskDelay((wait_us / portTICK_RATE_MS));
	return status;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	uint32_t     start_time_ms = 0;
	uint32_t     current_time_ms = 0;
	uint32_t     polling_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
    
    /* calculate time limit in absolute time */
    VL53L1_GetTickCount(&start_time_ms);
    
    /* wait until value is found, timeout reached on error occurred */
    while ((status == VL53L1_ERROR_NONE) && (polling_time_ms < timeout_ms) && (found == 0)) {

		if (status == VL53L1_ERROR_NONE)
			status = VL53L1_RdByte(pdev, index, &byte_value);

		if ((byte_value & mask) == value)
			found = 1;

		if (status == VL53L1_ERROR_NONE  &&
			found == 0 &&
			poll_delay_ms > 0)
			status = VL53L1_WaitMs(pdev, poll_delay_ms);

		/* Update polling time (Compare difference rather than absolute to negate 32bit wrap around issue) */
		VL53L1_GetTickCount(&current_time_ms);
		polling_time_ms = current_time_ms - start_time_ms;
	}
    
    if (found == 0 && status == VL53L1_ERROR_NONE)
		status = VL53L1_ERROR_TIME_OUT;
    
	return status;
}




