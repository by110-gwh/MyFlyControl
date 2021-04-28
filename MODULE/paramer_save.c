#include "paramer_save.h"
#include <string.h>
#include "stm32f1xx_hal.h"

//����Ĳ����ṹ��
volatile paramer_save_t paramer_save_data;

/**********************************************************************************************************
*�� �� ��: save_paramer_init
*����˵��: ��ʼ������
*��    ��: ��
*�� �� ֵ: ��
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
	paramer_save_data.gyro_y_offset = 0;
	paramer_save_data.esc_calibration_flag = 0;
}

/**********************************************************************************************************
*�� �� ��: write_save_paramer
*����˵��: �������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void write_save_paramer()
{
	uint32_t i;
	FLASH_EraseInitTypeDef pEraseInit;
	uint32_t PageError;
	
	//�����ж�
	__disable_irq();
	HAL_FLASH_Unlock();
	
	pEraseInit.PageAddress = PARAMETER_SAVE_ADDR;
	pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	pEraseInit.NbPages = 1;
	pEraseInit.Banks = FLASH_BANK_1;
	HAL_FLASHEx_Erase(&pEraseInit, &PageError);
	
	for (i = 0; i < (sizeof(paramer_save_t) + 3) / 4; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAMETER_SAVE_ADDR + i * 4, ((uint32_t *)&paramer_save_data)[i]) != HAL_OK) {
            break;
        }
    }
	
	HAL_FLASH_Lock();
	//�����ж�
	__enable_irq();
}

/**********************************************************************************************************
*�� �� ��: read_save_paramer
*����˵��: ��ȡ����Ĳ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void read_save_paramer()
{
	uint32_t *ReadAddress = (uint32_t *)PARAMETER_SAVE_ADDR;
	memcpy((uint8_t *)&paramer_save_data, ReadAddress, sizeof(paramer_save_t));
	if (paramer_save_data.inited != 0xAE) {
		save_paramer_init();
		write_save_paramer();
	}
}
