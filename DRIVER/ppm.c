#include "ppm.h"
#include "stm32f1xx_hal.h"
#include "time_cnt.h"
#include <string.h>
#include "remote_control.h"


/**********************************************************************************************************
*函 数 名: PPM_Init
*功能说明: PPM初始化
*形    参: 无
*返 回 值: 无
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
*函 数 名: EXTI9_5_IRQHandler
*功能说明: PPM IO中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void EXTI9_5_IRQHandler(void)
{
	static uint16_t PPM_buf[8];
	static Testime ppm_time;
	static uint8_t ppm_sample_cnt;
	uint16_t ppm_time_delta;

	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
	{
		//系统运行时间获取
		Get_Time_Period(&ppm_time);
		ppm_time_delta = ppm_time.Time_Delta;
		//PPM解析开始
		if (ppm_time_delta >= 2200) {
			ppm_sample_cnt = 0;
		} else if (ppm_time_delta >= 900 && ppm_time_delta <= 2100) {
			PPM_buf[ppm_sample_cnt++] = ppm_time_delta;
			//单次解析结束
			if (ppm_sample_cnt >= 8) {
				rc_callback(PPM_buf);
				ppm_sample_cnt = 0;
			}
		}
	}
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
}

