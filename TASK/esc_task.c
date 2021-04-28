#include "esc_task.h"
#include "paramer_save.h"
#include "remote_control.h"
#include "display_task.h"
#include "pwm.h"

#include "FreeRTOS.h"
#include "task.h"

//任务堆栈大小
#define ESC_TASK_STACK            256
//任务优先级
#define ESC_TASK_PRIORITY         5

//声明任务句柄
static xTaskHandle esc_task_handle;
//任务退出标志
volatile uint8_t esc_task_exit;

/**********************************************************************************************************
*函 数 名: esc_calibration
*功能说明: 执行电调较准
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void esc_calibration()
{
	paramer_save_data.esc_calibration_flag = 1;
	write_save_paramer();
	vTaskSuspend(NULL);
}

/**********************************************************************************************************
*函 数 名: esc_task
*功能说明: 电调较准任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(esc_task,  parameters)
{
	rc_init();
	pwm_init();
	//检测遥控器是否连接
	page_number = 17;
	while (!rc_is_on())
		vTaskDelay(1);
	page_number = 15;
    while (!esc_task_exit) {
		pwm_set(rc_raw_data[2], rc_raw_data[2], rc_raw_data[2], rc_raw_data[2]);
	}
}

/**********************************************************************************************************
*函 数 名: esc_task_create
*功能说明: 电调较准相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void esc_task_create(void)
{
	paramer_save_data.esc_calibration_flag = 0;
	write_save_paramer();
	esc_task_exit = 0;
	xTaskCreate(esc_task, "esc_task", ESC_TASK_STACK, NULL, ESC_TASK_PRIORITY, &esc_task_handle);
}
