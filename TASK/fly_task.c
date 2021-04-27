#include "fly_task.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "NCLink.h"
#include "ppm.h"
#include "imu.h"
#include "motor_output.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//任务堆栈大小
#define FLY_TASK_STACK 256
//任务优先级
#define FLY_TASK_PRIORITY 13

//声明任务句柄
xTaskHandle fly_task_handle;
//任务退出标志
uint8_t fly_task_exit;

/**********************************************************************************************************
*函 数 名: fly_task
*功能说明: 飞行定时任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(fly_task, pvParameters)
{
    portTickType xLastWakeTime;

    //挂起调度器
    //vTaskSuspendAll();
	
	imu_init();
	ahrs_init();
	NCLink_Init();
    //唤醒调度器
    //xTaskResumeAll();


    xLastWakeTime = xTaskGetTickCount();
    while (!fly_task_exit)
    {
        
		//获取imu数据
		get_imu_data();
		ahrs_update();
		motor_output_output();
		NCLink_SEND_StateMachine();
        //睡眠5ms
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
	vTaskDelete(NULL);
}


/**********************************************************************************************************
*函 数 名: fly_task_create
*功能说明: 传感器组件相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void fly_task_create(void)
{
	fly_task_exit = 0;
    xTaskCreate(fly_task, "fly_task", FLY_TASK_STACK, NULL, FLY_TASK_PRIORITY, &fly_task_handle);
}
