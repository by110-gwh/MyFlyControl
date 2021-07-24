#include "optical_flow_task.h"
#include "pmw3901.h"
#include "math.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "imu.h"
#include "Filter.h"

#include "FreeRTOS.h"
#include "task.h"

//任务堆栈大小
#define OPTICAL_FLOW_TASK_STACK 128
//任务优先级
#define OPTICAL_FLOW_TASK_PRIORITY 12
//任务退出标志
volatile uint8_t optical_flow_task_exit = 1;
//声明任务句柄
xTaskHandle optical_flow_task_handle;

#define PI 3.1415926f
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)

float optical_flow_speed_x;
float optical_flow_speed_y;
float optical_flow_pos_x;
float optical_flow_pos_y;
static float optical_flow_pos_x_integral;
static float optical_flow_pos_y_integral;

/**********************************************************************************************************
*函 数 名: optical_flow_task
*功能说明: 光流任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(optical_flow_task, pvParameters)
{
    portTickType xLastWakeTime;
    
    pmw3901_init();
    optical_flow_pos_x_integral = 0;
    optical_flow_pos_y_integral = 0;
    xLastWakeTime = xTaskGetTickCount();
    while (!optical_flow_task_exit) {
        int16_t dx, dy;
        uint16_t qual;
        pmw3901_read_motion(&dx, &dy, &qual);
        if (qual) {
            //用于补偿的角速度
            float gyro_data_x = gyroDataFilterOptical.x * GYRO_CALIBRATION_COFF * DEG2RAD;
            float gyro_data_y = gyroDataFilterOptical.y * GYRO_CALIBRATION_COFF * DEG2RAD;
            //补偿后的光流速度
            float opt_data_x = dx * OPTICAL_SCALS * 10 + gyro_data_y;
            float opt_data_y = dy * OPTICAL_SCALS * 10 - gyro_data_x;
            optical_flow_speed_x = opt_data_x * pos_z;
            optical_flow_speed_y = opt_data_y * pos_z;
            
            //光流累计移动
            optical_flow_pos_x_integral += dx * OPTICAL_SCALS * pos_z;
            optical_flow_pos_y_integral += dy * OPTICAL_SCALS * pos_z;
            //光流位置补偿
            optical_flow_pos_x = optical_flow_pos_x_integral + pos_z * tanf(Roll * DEG2RAD);
            optical_flow_pos_y = optical_flow_pos_y_integral - pos_z * tanf(Pitch * DEG2RAD);
        }
        //睡眠100ms
        vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_RATE_MS));
    }
}


/**********************************************************************************************************
*函 数 名: optical_flow_task_create
*功能说明: 光流相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void optical_flow_task_create(void)
{
    optical_flow_task_exit = 0;
    xTaskCreate(optical_flow_task, "optical_flow_task", OPTICAL_FLOW_TASK_STACK, NULL, OPTICAL_FLOW_TASK_PRIORITY, &optical_flow_task_handle);
}
