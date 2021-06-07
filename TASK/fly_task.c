#include "fly_task.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "ppm.h"
#include "imu.h"
#include "vl53l1x.h"
#include "remote_control.h"
#include "controller.h"
#include "motor_output.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stdio.h"

//任务堆栈大小
#define FLY_TASK_STACK 512
//任务优先级
#define FLY_TASK_PRIORITY 13

//声明任务句柄
xTaskHandle fly_task_handle;
//任务退出标志
volatile uint8_t fly_task_exit;

/**********************************************************************************************************
*函 数 名: fly_task
*功能说明: 飞行定时任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(fly_task, pvParameters)
{
    portTickType xLastWakeTime;
	
	imu_init();
	ahrs_init();
    controller_init();
    vl53l1x_init();
    navigation_init();

    xLastWakeTime = xTaskGetTickCount();
    while (!fly_task_exit) {
		//获取imu数据
		get_imu_data();
        //获取高度
        vl53l1x_task();
        //姿态解算
		ahrs_update();
        //导航加速度计算
        navigation_prepare();
        //高度位置估计
        high_kalman_filter();
        //控制器
        controller_run();
        //控制器输出
		motor_output_output();
        
        //extern float high_vel, high_acce, high_pos;
        //printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n", high_vel, high_raw_data / 10.0, high_pos, high_acce, navigation_acce.z); 
        printf("%d\r\n", throttle_motor_output);
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
