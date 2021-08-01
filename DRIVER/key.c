#include "key.h"

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

#include "FreeRTOS.h"
#include "task.h"

#define key0 (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3) & GPIO_PIN_3)
#define key1 (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2) & GPIO_PIN_2)


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
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    ROM_GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
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
	} else if (key0 != 0) {
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
	} else if (key1 != 0) {
		key1_press_flag = 0;
	}
	return 0;
}
