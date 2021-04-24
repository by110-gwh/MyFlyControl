#include "ppm.h"
#include "stm32f1xx_hal.h"
#include "time_cnt.h"
#include <string.h>
#include "remote_control.h"


/**********************************************************************************************************
*�� �� ��: PPM_Init
*����˵��: PPM��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void PPM_Init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**********************************************************************************************************
*�� �� ��: EXTI9_5_IRQHandler
*����˵��: PPM IO�ж�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void EXTI9_5_IRQHandler(void)
{
	static uint16_t PPM_buf[8];
	static Testime ppm_time;
	static uint8_t ppm_sample_cnt;
	uint16_t ppm_time_delta;

	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
	{
		//ϵͳ����ʱ���ȡ
		Get_Time_Period(&ppm_time);
		ppm_time_delta = ppm_time.Time_Delta;
		//PPM������ʼ
		if (ppm_time_delta >= 2200) {
			ppm_sample_cnt = 0;
		} else if (ppm_time_delta >= 900 && ppm_time_delta <= 2100) {
			PPM_buf[ppm_sample_cnt++] = ppm_time_delta;
			//���ν�������
			if (ppm_sample_cnt >= 8) {
				rc_callback(PPM_buf);
				ppm_sample_cnt = 0;
			}
		}
	}
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
}

