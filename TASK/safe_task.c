#include "safe_task.h"
#include "pwm.h"
#include "remote_control.h"
#include "fly_task.h"

#include "FreeRTOS.h"
#include "task.h"

//任务堆栈大小
#define SAFE_TASK_STACK 128
//任务优先级
#define SAFE_TASK_PRIORITY 10


//声明任务句柄
xTaskHandle safe_task_handle;

/**********************************************************************************************************
*函 数 名: safe_task_motor_stop
*功能说明: 紧急停机
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void safe_task_motor_stop(void)
{
    __disable_fiq();
    while (1) {
        pwm_set(1000, 1000, 1000, 1000);
    }
}

/**********************************************************************************************************
*函 数 名: safe_task
*功能说明: 飞行安全任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(safe_task, pvParameters)
{
    //等待任务初始化
    vTaskDelay((1000 / portTICK_RATE_MS));
    while (1) {
        //遥控器错误
        if (remote_control_updata == 0)
            safe_task_motor_stop();
        //飞行任务卡死
        if (fly_task_exit == 0 && fly_task_updata == 0)
            safe_task_motor_stop();
        
        fly_task_updata = 0;
        remote_control_updata = 0;
        //睡眠200ms
        vTaskDelay((200 / portTICK_RATE_MS));
    }
}


/**********************************************************************************************************
*函 数 名: safe_task_create
*功能说明: 飞行安全任务组件相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void safe_task_create(void)
{
    xTaskCreate(safe_task, "safe_task", SAFE_TASK_STACK, NULL, SAFE_TASK_PRIORITY, &safe_task_handle);
}
