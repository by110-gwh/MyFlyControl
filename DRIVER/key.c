#include "key.h"
#include "stm32f1xx_hal.h"
#include "bitband.h"

#include "FreeRTOS.h"
#include "task.h"

#define key0 PCin(8)
#define key1 PCin(9)


//长按时间，单位ms
#define LONG_PRESS_TIME 2000

/**********************************************************************************************************
*函 数 名: key_init
*功能说明: 按键初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void key_init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**********************************************************************************************************
*函 数 名: key_scan
*功能说明: 按键扫描
*形    参: 无
*返 回 值: [7:1]：按键值 [0]：是否为长按
**********************************************************************************************************/
uint8_t key_scan()
{
	//按下时间
	uint32_t press_time;
	//按键按下标志
	static uint8_t key0_press_flag;
	static uint8_t key1_press_flag;
	
	//按键一
	if(key0 == 0 && !key0_press_flag) {
		vTaskDelay(10);
		if(key0 == 0) {
			press_time = 0;
			key0_press_flag = 1;
			while(key0 == 0 && press_time < LONG_PRESS_TIME) {
				vTaskDelay(100);
				press_time += 100;
			}
			//是否为长按
			if (press_time >= LONG_PRESS_TIME)
				return KEY0 << 1 | 1;
			else
				return KEY0 << 1;
		}
	} else if (key0 == 1) {
		key0_press_flag = 0;
	}
  
	//按键二
	if(key1 == 0 && !key1_press_flag) {
		vTaskDelay(10);
		if(key1 == 0) {
			press_time = 0;
			key1_press_flag = 1;
			while(key1 == 0 && press_time < LONG_PRESS_TIME) {
				vTaskDelay(100);
				press_time += 100;
			}
			//是否为长按
			if (press_time >= LONG_PRESS_TIME)
				return KEY1 << 1 | 1;
			else
				return KEY1 << 1;
		}
	} else if (key1 == 1) {
		key1_press_flag = 0;
	}
	return 0;
}
