#include "fly_task.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "ppm.h"
#include "imu.h"
#include "motor_output.h"
#include "attitude_self_stabilization.h"
#include "angle_control.h"
#include "gyro_control.h"
// #include "vl53l1x.h"
#include "sr04.h"

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

    //挂起调度器
    //vTaskSuspendAll();
	
	imu_init();
	ahrs_init();
    angle_control_init();
    gyro_control_init();
    // vl53l1x_init();
    sr04_task_create();
    navigation_init();
    //唤醒调度器
    //xTaskResumeAll();


    xLastWakeTime = xTaskGetTickCount();
    while (!fly_task_exit) {
		//获取imu数据
		get_imu_data();
		ahrs_update();
        navigation_prepare();
        high_kalman_filter();
		attitude_self_stabilization_control();
        angle_control();
        gyro_control();
		motor_output_output();
        extern float high_vel, high_acce, high_pos;
        printf("%0.3f,%0.3f,%0.3f\r\n", high_vel, high_raw_data / 10.0, high_pos); 
        //睡眠5ms
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
    sr04_task_exit = 1;
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
