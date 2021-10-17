#include "paramer_save.h"
#include <string.h>

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/eeprom.h"

#define PARAMETER_SAVE_ADDR 0

//保存的参数结构体
volatile paramer_save_t paramer_save_data;

/**********************************************************************************************************
*函 数 名: save_paramer_init
*功能说明: 初始化参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void save_paramer_init()
{
	paramer_save_data.inited = 0xAE;
	paramer_save_data.rc_ch1_max = 2000;
	paramer_save_data.rc_ch1_min = 1000;
	paramer_save_data.rc_ch2_max = 2000;
	paramer_save_data.rc_ch2_min = 1000;
	paramer_save_data.rc_ch3_max = 2000;
	paramer_save_data.rc_ch3_min = 1000;
	paramer_save_data.rc_ch4_max = 2000;
	paramer_save_data.rc_ch4_min = 1000;
	paramer_save_data.rc_ch5_max = 2000;
	paramer_save_data.rc_ch5_min = 1000;
	paramer_save_data.rc_ch6_max = 2000;
	paramer_save_data.rc_ch6_min = 1000;
	paramer_save_data.rc_ch7_max = 2000;
	paramer_save_data.rc_ch7_min = 1000;
	paramer_save_data.rc_ch8_max = 2000;
	paramer_save_data.rc_ch8_min = 1000;
	paramer_save_data.accel_x_offset = 0;
	paramer_save_data.accel_y_offset = 0;
	paramer_save_data.accel_z_offset = 0;
	paramer_save_data.accel_x_scale = 1;
	paramer_save_data.accel_y_scale = 1;
	paramer_save_data.accel_z_scale = 1;
	paramer_save_data.mag_x_offset = 0;
	paramer_save_data.mag_y_offset = 0;
	paramer_save_data.mag_z_offset = 0;
	paramer_save_data.gyro_x_offset = 0;
	paramer_save_data.gyro_y_offset = 0;
	paramer_save_data.gyro_z_offset = 0;
	paramer_save_data.esc_calibration_flag = 0;
}

/**********************************************************************************************************
*函 数 名: paramer_save_init
*功能说明: 参数保存模块初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void paramer_save_init()
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    ROM_EEPROMInit();
}

/**********************************************************************************************************
*函 数 名: write_save_paramer
*功能说明: 保存参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void write_save_paramer()
{
    ROM_EEPROMProgram((uint32_t *)(&paramer_save_data), PARAMETER_SAVE_ADDR, sizeof(paramer_save_t));
}

/**********************************************************************************************************
*函 数 名: read_save_paramer
*功能说明: 读取保存的参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void read_save_paramer()
{
	ROM_EEPROMRead((uint32_t *)(&paramer_save_data), PARAMETER_SAVE_ADDR, sizeof(paramer_save_t));
	if (paramer_save_data.inited != 0xAE) {
		save_paramer_init();
		write_save_paramer();
	}
}
