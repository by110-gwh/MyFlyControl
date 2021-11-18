#include "laser_task.h"
#include "laser.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

//任务堆栈大小
#define LASER_TASK_STACK 128
//任务优先级
#define LASER_TASK_PRIORITY 8

//任务退出标志
volatile uint8_t laser_task_exit = 1;
//声明任务句柄
xTaskHandle laser_task_handle;

//激光占空比
uint8_t laser_duty;
//激光周期
uint16_t laser_cycle;
//激光闪烁次数
uint8_t laser_time;

/**********************************************************************************************************
*函 数 名: laser_task
*功能说明: 激光管理任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(laser_task, pvParameters)
{
    laser_init();
    while (!laser_task_exit) {
        //需要蜂鸣
        if (laser_time) {
            //亮
            laser_on();
            vTaskDelay((laser_cycle * 10 * laser_duty / 100 / portTICK_RATE_MS));
            //熄灭
            laser_off();
            vTaskDelay((laser_cycle * 10 * (100 - laser_duty) / 100 / portTICK_RATE_MS));
            //更新蜂鸣
            if (laser_time != 0xFF && laser_time != 0) {
                laser_time--;
            }
        }
        //停止蜂鸣
        laser_off();
        vTaskDelay(100);
    }
    vTaskDelete(NULL);
}


/**********************************************************************************************************
*函 数 名: laser_task_create
*功能说明: 激光管理相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void laser_task_create(void)
{
    laser_task_exit = 0;
    xTaskCreate(laser_task, "laser_task", LASER_TASK_STACK, NULL, LASER_TASK_PRIORITY, &laser_task_handle);
}
