#include "sr04.h"
#include "stm32f1xx_hal.h"
#include "time_cnt.h"
#include "bitband.h"

#include "FreeRTOS.h"
#include "task.h"

//任务堆栈大小
#define SR04_TASK_STACK            256
//任务优先级
#define SR04_TASK_PRIORITY         12
//触发引脚
#define TRIG PBout(8)

//声明任务句柄
xTaskHandle sr04_task_handle;
//任务退出标志
volatile uint8_t sr04_task_exit;
//高度原始数据
uint16_t high_raw_data;

/**********************************************************************************************************
*函 数 名: sr04_task
*功能说明: 超声波定高任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(sr04_task, parameters)
{
    portTickType xLastWakeTime;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
	
    //初始化GPIO时钟
	__HAL_RCC_GPIOB_CLK_ENABLE();
    
    //初始化触发引脚
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    //初始化中断引脚
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    xLastWakeTime = xTaskGetTickCount();
    while (!sr04_task_exit) {
        //触发开始测量
        TRIG = 1;
        vTaskDelay(1);
        TRIG = 0;
        
        //睡眠100ms
        vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_RATE_MS));
    }
	vTaskDelete(NULL);
}

/**********************************************************************************************************
*函 数 名: sr04_task_create
*功能说明: 主函数相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void sr04_task_create(void)
{
	sr04_task_exit = 0;
    xTaskCreate(sr04_task, "sr04_task", SR04_TASK_STACK, NULL, SR04_TASK_PRIORITY, &sr04_task_handle);
}

/**********************************************************************************************************
*函 数 名: sr04_exit_callback
*功能说明: 超声波外部中断回调函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void sr04_exit_callback()
{
    static Testime timer;
    static uint8_t wait_fall;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    //接收上升沿
    if (wait_fall == 0) {
        //记录时间
        Get_Time_Period(&timer);
        //设置为下降沿触发中断
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        wait_fall = 1;
    //接收下降沿
    } else {
        //计算时间差
        Get_Time_Period(&timer);
        //计算距离
        high_raw_data = timer.Time_Delta * 0.340 / 2;
        if (high_raw_data > 2000)
            high_raw_data = 0;
        //设置为下降沿触发中断
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        wait_fall =-0;
    }
}

