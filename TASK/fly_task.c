#include "fly_task.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "ppm.h"
#include "imu.h"
#include "vl53l1x.h"
#include "remote_control.h"
#include "controller.h"
#include "motor_output.h"
#include "optical_flow_task.h"

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
volatile uint8_t fly_task_exit = 1;
//飞行任务更新标志，用于安全检查
volatile uint8_t fly_task_updata;
#include "gyro_control.h"
#include "angle_control.h"
#include "high_control.h"
#include "horizontal_control.h"
/**********************************************************************************************************
*函 数 名: fly_task
*功能说明: 飞行定时任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(fly_task, pvParameters)
{
    portTickType xLastWakeTime;
	
    optical_flow_task_create();
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
        pos_filter();
        //控制器
        controller_run();
        //控制器输出
		motor_output_output();
        
        extern float pos_x, pos_y;
        extern float speed_x, speed_y;
//        printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n", high_vel, high_raw_data / 10.0 * Cos_Roll * Cos_Pitch, high_pos, high_acce, navigation_acce.z); 
//        printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n", high_pos_pid_data.expect, high_pos_pid_data.feedback, high_speed_pid_data.expect, high_speed_pid_data.feedback, high_speed_pid_data.control_output); 
//        printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n", horizontal_pos_y_pid_data.expect, horizontal_pos_y_pid_data.feedback, horizontal_speed_y_pid_data.expect,
//            horizontal_speed_y_pid_data.feedback, horizontal_speed_y_pid_data.control_output, pitch_angle_pid_data.expect); 
        printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n", navigation_acce.y / 10, speed_y, pos_y, optical_flow_pos_y, optical_flow_speed_y); 
//        printf("%0.3f\r\n", yaw_angle_pid_data.feedback); 
        //printf("%d\r\n", throttle_motor_output);
        fly_task_updata = 1;
        //睡眠5ms
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
	vTaskDelete(NULL);
    optical_flow_task_exit = 1;
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
