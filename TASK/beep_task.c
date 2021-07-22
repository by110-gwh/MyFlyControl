#include "beep_task.h"
#include "beep.h"

#include "FreeRTOS.h"
#include "task.h"

//任务堆栈大小
#define BEEP_TASK_STACK 128
//任务优先级
#define BEEP_TASK_PRIORITY 6

//声明任务句柄
xTaskHandle beep_task_handle;

//蜂鸣器占空比
uint8_t beep_duty;
//蜂鸣器周期
uint16_t beep_cycle;
//蜂鸣器蜂鸣次数
uint8_t beep_time;

/**********************************************************************************************************
*函 数 名: beep_task
*功能说明: 蜂鸣器任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(beep_task, pvParameters)
{
    //蜂鸣器初始化
    beep_init();
    while (1) {
        //需要蜂鸣
        if (beep_time) {
            //蜂鸣
            beep_control(1);
            vTaskDelay((beep_cycle * 10 * beep_duty / 100 / portTICK_RATE_MS));
            //停止蜂鸣
            beep_control(0);
            vTaskDelay((beep_cycle * 10 * (100 - beep_duty) / 100 / portTICK_RATE_MS));
            //更新蜂鸣
            if (beep_time != 0xFF && beep_time != 0) {
                beep_time--;
            }
        }
        //停止蜂鸣
        beep_control(0);
        vTaskDelay(100);
    }
}


/**********************************************************************************************************
*函 数 名: beep_task_create
*功能说明: 蜂鸣器任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void beep_task_create(void)
{
    xTaskCreate(beep_task, "beep_task", BEEP_TASK_STACK, NULL, BEEP_TASK_PRIORITY, &beep_task_handle);
}
