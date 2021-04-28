#include "esc_task.h"
#include "paramer_save.h"
#include "remote_control.h"
#include "display_task.h"
#include "pwm.h"

#include "FreeRTOS.h"
#include "task.h"

//�����ջ��С
#define ESC_TASK_STACK            256
//�������ȼ�
#define ESC_TASK_PRIORITY         5

//����������
static xTaskHandle esc_task_handle;
//�����˳���־
volatile uint8_t esc_task_exit;

/**********************************************************************************************************
*�� �� ��: esc_calibration
*����˵��: ִ�е����׼
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void esc_calibration()
{
	paramer_save_data.esc_calibration_flag = 1;
	write_save_paramer();
	vTaskSuspend(NULL);
}

/**********************************************************************************************************
*�� �� ��: esc_task
*����˵��: �����׼����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
portTASK_FUNCTION(esc_task,  parameters)
{
	rc_init();
	pwm_init();
	//���ң�����Ƿ�����
	page_number = 17;
	while (!rc_is_on())
		vTaskDelay(1);
	page_number = 15;
    while (!esc_task_exit) {
		pwm_set(rc_raw_data[2], rc_raw_data[2], rc_raw_data[2], rc_raw_data[2]);
	}
}

/**********************************************************************************************************
*�� �� ��: esc_task_create
*����˵��: �����׼������񴴽�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void esc_task_create(void)
{
	paramer_save_data.esc_calibration_flag = 0;
	write_save_paramer();
	esc_task_exit = 0;
	xTaskCreate(esc_task, "esc_task", ESC_TASK_STACK, NULL, ESC_TASK_PRIORITY, &esc_task_handle);
}
