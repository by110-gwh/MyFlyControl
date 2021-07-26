#include "route_plan_task.h"
#include "high_control.h"
#include "angle_control.h"
#include "horizontal_control.h"
#include "route_plan_attitude_stabilization.h"
#include "ahrs_aux.h"
#include "remote_control.h"

#include "FreeRTOS.h"
#include "task.h"

//任务堆栈大小
#define ROUTE_PLAN_TASK_STACK            256
//任务优先级
#define ROUTE_PLAN_TASK_PRIORITY         11

//声明任务句柄
static xTaskHandle route_plan_task_handle;
//任务退出标志
volatile uint8_t route_plan_task_exit;


/**********************************************************************************************************
*函 数 名: fly_high
*功能说明: 飞行高度控制
*形    参: 目标高度 速度
*返 回 值: 无
**********************************************************************************************************/
void fly_high(uint8_t targer_high, float speed) {
    while (high_pos_pid_data.expect != targer_high) {
        if (targer_high > high_pos_pid_data.expect) {
            if (targer_high - high_pos_pid_data.expect > speed) {
                high_pos_pid_data.expect += speed;
            } else {
                high_pos_pid_data.expect = targer_high;
            }
        } else {
            if (high_pos_pid_data.expect - targer_high > speed) {
                high_pos_pid_data.expect -= speed;
            } else {
                high_pos_pid_data.expect = targer_high;
            }
        }
        vTaskDelay(10);
    }
}

/**********************************************************************************************************
*函 数 名: fly_forward
*功能说明: 向前飞控制
*形    参: 目标位置 速度
*返 回 值: 无
**********************************************************************************************************/
void fly_forward(uint8_t targer_distan, float speed) {
    while (horizontal_pos_y_pid_data.expect != targer_distan) {
        if (targer_distan > horizontal_pos_y_pid_data.expect) {
            if (targer_distan - horizontal_pos_y_pid_data.expect > speed) {
                horizontal_pos_y_pid_data.expect += speed;
            } else {
                horizontal_pos_y_pid_data.expect = targer_distan;
            }
        } else {
            if (horizontal_pos_y_pid_data.expect - targer_distan > speed) {
                horizontal_pos_y_pid_data.expect -= speed;
            } else {
                horizontal_pos_y_pid_data.expect = targer_distan;
            }
        }
        vTaskDelay(10);
    }
}

/**********************************************************************************************************
*函 数 名: fly_right
*功能说明: 向右飞控制
*形    参: 目标位置 速度
*返 回 值: 无
**********************************************************************************************************/
void fly_right(uint8_t targer_distan, float speed) {
    while (horizontal_pos_x_pid_data.expect != targer_distan) {
        if (targer_distan > horizontal_pos_x_pid_data.expect) {
            if (targer_distan - horizontal_pos_x_pid_data.expect > speed) {
                horizontal_pos_x_pid_data.expect += speed;
            } else {
                horizontal_pos_x_pid_data.expect = targer_distan;
            }
        } else {
            if (horizontal_pos_x_pid_data.expect - targer_distan > speed) {
                horizontal_pos_x_pid_data.expect -= speed;
            } else {
                horizontal_pos_x_pid_data.expect = targer_distan;
            }
        }
        vTaskDelay(10);
    }
}

/**********************************************************************************************************
*函 数 名: route_plan_task
*功能说明: 路径规划任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(route_plan_task,  parameters)
{
    //升高到100cm
    fly_high(100, 0.5);
    vTaskDelay(2000);
    fly_forward(100,0.3);
    vTaskDelay(2000);
    fly_right(100,0.3);
    vTaskDelay(2000);
    //下降到0cm
    fly_high(0, 0.5);
    vTaskDelay(200);
    
    save_throttle_control = Throttle_Control;
    save_high_expect = high_pos_pid_data.expect;
    route_plan_finish = 1;
    route_plan_stop_flag = 1;
    vTaskDelete(NULL);
}

/**********************************************************************************************************
*函 数 名: route_plan_task_create
*功能说明: 路径规划相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void route_plan_task_create(void)
{
	route_plan_task_exit = 0;
	xTaskCreate(route_plan_task, "route_plan_task", ROUTE_PLAN_TASK_STACK, NULL, ROUTE_PLAN_TASK_PRIORITY, &route_plan_task_handle);
}

/**********************************************************************************************************
*函 数 名: route_plan_task_delete
*功能说明: 路径规划相关任务删除
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void route_plan_task_delete(void)
{
    vTaskDelete(route_plan_task_handle);
}
