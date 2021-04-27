#include "module_task.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "NCLink.h"
#include "ppm.h"
#include "imu.h"
#include "control.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//任务堆栈大小
#define IMU_SENSOR_READ_TASK_STACK            256
//任务优先级
#define IMU_SENSOR_READ_TASK_PRIORITY         13

//声明任务句柄
xTaskHandle imuSensorReadTask;
//任务退出标志
uint8_t imuSensorReadTaskExit;
//系统消息队列
QueueHandle_t acce_raw_queue;
QueueHandle_t acce_correct_raw_queue;
QueueHandle_t gyro_raw_queue;
QueueHandle_t temp_raw_queue;

/**********************************************************************************************************
*函 数 名: vImuSensorReadTask
*功能说明: IMU传感器数据读取任务，此任务具有最高优先级，运行频率为1KHz
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vImuSensorReadTask, pvParameters)
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
    while (!imuSensorReadTaskExit)
    {
        
		//获取imu数据
		get_imu_data();
		ahrs_update();
		control_output();
		NCLink_SEND_StateMachine();
        //睡眠5ms
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
	vTaskDelete(NULL);
}


/**********************************************************************************************************
*函 数 名: ModuleTaskCreate
*功能说明: 传感器组件相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ModuleTaskCreate(void)
{
	imuSensorReadTaskExit = 0;
    xTaskCreate(vImuSensorReadTask, "imuSensorRead", IMU_SENSOR_READ_TASK_STACK, NULL, IMU_SENSOR_READ_TASK_PRIORITY, &imuSensorReadTask);
}
