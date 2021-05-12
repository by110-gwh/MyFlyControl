#include "usmart_task.h"
#include "usmart.h"

#include "FreeRTOS.h"
#include "task.h"

//任务堆栈大小
#define USMART_TASK_STACK            256
//任务优先级
#define USMART_TASK_PRIORITY         7

//声明任务句柄
static xTaskHandle usmart_task_handle;
//任务退出标志
volatile uint8_t usmart_task_exit;

/**********************************************************************************************************
*函 数 名: usmart_task
*功能说明: USMART任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(usmart_task,  parameters)
{
	//初始化usmart
	usmart_dev.init(72);
    while (!usmart_task_exit) {
		//执行usmart扫描
		usmart_dev.scan();
		vTaskDelay(100);
	}
}

/**********************************************************************************************************
*函 数 名: usmart_task_create
*功能说明: USMART相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void usmart_task_create(void)
{
	usmart_task_exit = 0;
	xTaskCreate(usmart_task, "usmart_task", USMART_TASK_STACK, NULL, USMART_TASK_PRIORITY, &usmart_task_handle);
}
