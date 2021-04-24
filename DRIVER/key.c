#include "key.h"
#include "stm32f1xx_hal.h"
#include "bitband.h"

#include "FreeRTOS.h"
#include "task.h"

#define key0 PCin(8)
#define key1 PCin(9)


//����ʱ�䣬��λms
#define LONG_PRESS_TIME 2000

/**********************************************************************************************************
*�� �� ��: key_init
*����˵��: ������ʼ��
*��    ��: ��
*�� �� ֵ: ��
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
*�� �� ��: key_scan
*����˵��: ����ɨ��
*��    ��: ��
*�� �� ֵ: [7:1]������ֵ [0]���Ƿ�Ϊ����
**********************************************************************************************************/
uint8_t key_scan()
{
	//����ʱ��
	uint32_t press_time;
	//�������±�־
	static uint8_t key0_press_flag;
	static uint8_t key1_press_flag;
	
	//����һ
	if(key0 == 0 && !key0_press_flag) {
		vTaskDelay(10);
		if(key0 == 0) {
			press_time = 0;
			key0_press_flag = 1;
			while(key0 == 0 && press_time < LONG_PRESS_TIME) {
				vTaskDelay(100);
				press_time += 100;
			}
			//�Ƿ�Ϊ����
			if (press_time >= LONG_PRESS_TIME)
				return KEY0 << 1 | 1;
			else
				return KEY0 << 1;
		}
	} else if (key0 == 1) {
		key0_press_flag = 0;
	}
  
	//������
	if(key1 == 0 && !key1_press_flag) {
		vTaskDelay(10);
		if(key1 == 0) {
			press_time = 0;
			key1_press_flag = 1;
			while(key1 == 0 && press_time < LONG_PRESS_TIME) {
				vTaskDelay(100);
				press_time += 100;
			}
			//�Ƿ�Ϊ����
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
