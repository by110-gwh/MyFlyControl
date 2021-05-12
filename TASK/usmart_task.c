#include "usmart_task.h"
#include "usmart.h"

#include "FreeRTOS.h"
#include "task.h"

//�����ջ��С
#define USMART_TASK_STACK            256
//�������ȼ�
#define USMART_TASK_PRIORITY         7

//����������
static xTaskHandle usmart_task_handle;
//�����˳���־
volatile uint8_t usmart_task_exit;

/**********************************************************************************************************
*�� �� ��: usmart_task
*����˵��: USMART����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
portTASK_FUNCTION(usmart_task,  parameters)
{
	//��ʼ��usmart
	usmart_dev.init(72);
    while (!usmart_task_exit) {
		//ִ��usmartɨ��
		usmart_dev.scan();
		vTaskDelay(100);
	}
}

/**********************************************************************************************************
*�� �� ��: usmart_task_create
*����˵��: USMART������񴴽�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void usmart_task_create(void)
{
	usmart_task_exit = 0;
	xTaskCreate(usmart_task, "usmart_task", USMART_TASK_STACK, NULL, USMART_TASK_PRIORITY, &usmart_task_handle);
}
