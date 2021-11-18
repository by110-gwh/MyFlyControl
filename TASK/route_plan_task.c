#include "route_plan_task.h"
#include "high_control.h"
#include "angle_control.h"
#include "horizontal_control.h"
#include "route_plan_attitude_stabilization.h"
#include "ahrs_aux.h"
#include "remote_control.h"
#include <math.h>
#include "beep_task.h"
#include "main_task.h"

#include "FreeRTOS.h"
#include "task.h"

//�����ջ��С
#define ROUTE_PLAN_TASK_STACK            256
//�������ȼ�
#define ROUTE_PLAN_TASK_PRIORITY         11

//����������
static xTaskHandle route_plan_task_handle;
//�����˳���־
volatile uint8_t route_plan_task_exit;

#define ABS(X)  (((X) > 0)? (X) : -(X))

/**********************************************************************************************************
*�� �� ��: fly_high
*����˵��: ���и߶ȿ���
*��    ��: Ŀ��߶� �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void fly_high(int targer_high, float speed)
{
    int cnt;
    float last_dstan;
    
    if ((targer_high > 0 && speed < 0) || (targer_high < 0 && speed > 0)) {
        speed = -speed;
    }
    
    cnt = targer_high / speed * 20;
    last_dstan = targer_high - (cnt * speed / 20);
    
    while (cnt--) {
        high_pos_pid_data.expect += speed / 20;
        vTaskDelay(50);
    }
    high_pos_pid_data.expect += last_dstan;
}

/**********************************************************************************************************
*�� �� ��: fly_forward
*����˵��: ��ǰ�ɿ���
*��    ��: Ŀ��λ�� �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void fly_forward(int targer_distan, float speed)
{
    int cnt;
    float last_dstan;
    
    if ((targer_distan > 0 && speed < 0) || (targer_distan < 0 && speed > 0)) {
        speed = -speed;
    }
    
    cnt = targer_distan / speed * 20;
    last_dstan = targer_distan - (cnt * speed / 20);
    
    while (cnt--) {
        horizontal_pos_y_pid_data.expect += speed / 20;
        vTaskDelay(50);
    }
    horizontal_pos_y_pid_data.expect += last_dstan;
}

/**********************************************************************************************************
*�� �� ��: fly_right
*����˵��: ���ҷɿ���
*��    ��: Ŀ��λ�� �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void fly_right(int targer_distan, float speed)
{
    int cnt;
    float last_dstan;
    
    if ((targer_distan > 0 && speed < 0) || (targer_distan < 0 && speed > 0)) {
        speed = -speed;
    }
    
    cnt = targer_distan / speed * 20;
    last_dstan = targer_distan - (cnt * speed / 20);
    
    while (cnt--) {
        horizontal_pos_x_pid_data.expect += speed / 20;
        vTaskDelay(50);
    }
    horizontal_pos_x_pid_data.expect += last_dstan;
}

/**********************************************************************************************************
*�� �� ��: fly_turn
*����˵��: ԭ��ת��
*��    ��: Ŀ��Ƕ� �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void fly_turn(int targer_angle, float speed)
{
    int cnt;
    float last_dstan;
    
    if ((targer_angle > 0 && speed < 0) || (targer_angle < 0 && speed > 0)) {
        speed = -speed;
    }
    
    cnt = targer_angle / speed * 20;
    last_dstan = targer_angle - (cnt * speed / 20);
    
    while (cnt--) {
        yaw_angle_pid_data.expect += speed / 20;
        if (yaw_angle_pid_data.expect > 180)
            yaw_angle_pid_data.expect -= 360;
        if (yaw_angle_pid_data.expect < -180)
            yaw_angle_pid_data.expect += 360;
        vTaskDelay(50);
    }
    yaw_angle_pid_data.expect += last_dstan;
    if (yaw_angle_pid_data.expect > 180)
        yaw_angle_pid_data.expect -= 360;
    if (yaw_angle_pid_data.expect < -180)
        yaw_angle_pid_data.expect += 360;
}

/**********************************************************************************************************
*�� �� ��: route_plan_task
*����˵��: ·���滮����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
portTASK_FUNCTION(route_plan_task,  parameters)
{
    //���ߵ�90cm
    fly_high(130 - high_pos_pid_data.expect, 50);
    vTaskDelay(1000);
    //�½���0cm
    fly_high(-20 - high_pos_pid_data.expect, 50);
    
    vTaskDelay(400);
    save_throttle_control = Throttle_Control;
    save_high_expect = high_pos_pid_data.expect;
    route_plan_finish = 1;
    route_plan_stop_flag = 1;
    route_plan_task_delete();
}

/**********************************************************************************************************
*�� �� ��: route_plan_task_create
*����˵��: ·���滮������񴴽�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void route_plan_task_create(void)
{
	route_plan_task_exit = 0;
    xTaskCreate(route_plan_task, "route_plan_task", ROUTE_PLAN_TASK_STACK, NULL, ROUTE_PLAN_TASK_PRIORITY, &route_plan_task_handle);
}

/**********************************************************************************************************
*�� �� ��: route_plan_task_delete
*����˵��: ·���滮�������ɾ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void route_plan_task_delete(void)
{
    if (route_plan_task_handle) {
        void *p_temp = route_plan_task_handle;
        route_plan_task_handle = NULL;
        vTaskDelete(p_temp);
    }
}
