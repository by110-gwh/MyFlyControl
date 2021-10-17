#include "route_plan_task.h"
#include "high_control.h"
#include "angle_control.h"
#include "horizontal_control.h"
#include "route_plan_attitude_stabilization.h"
#include "ahrs_aux.h"
#include "remote_control.h"
#include <math.h>
#include "openmv.h"
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


/**********************************************************************************************************
*�� �� ��: fly_high
*����˵��: ���и߶ȿ���
*��    ��: Ŀ��߶� �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void fly_high(int targer_high, float speed) {
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
static void fly_forward(int targer_distan, float speed) {
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
static void fly_right(int targer_distan, float speed) {
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
*�� �� ��: fly_forward_high
*����˵��: ԭ����ǰ45�����
*��    ��: Ŀ��λ�� �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void fly_forward_high(int targer_distan, float speed) {
    int cnt;
    float last_dstan;
    
    if ((targer_distan > 0 && speed < 0) || (targer_distan < 0 && speed > 0)) {
        speed = -speed;
    }
    
    cnt = targer_distan / speed * 20;
    last_dstan = targer_distan - (cnt * speed / 20);
    
    while (cnt--) {
        high_pos_pid_data.expect += speed / 20;
        horizontal_pos_y_pid_data.expect += speed / 20;
        vTaskDelay(50);
    }
    high_pos_pid_data.expect += last_dstan;
    horizontal_pos_y_pid_data.expect += last_dstan;
}

/**********************************************************************************************************
*�� �� ��: fly_circle
*����˵��: ����������Ȧ
*��    ��: ��ʼ�Ƕ� ��ֹ�Ƕ� �뾶 �ٶȶ�ÿ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void fly_circle(int start_angle, int end_angle, int r, float speed) {
    int cnt;
    float last_angle;
    float this_angle;
    
    last_angle = start_angle;
    cnt = (end_angle - start_angle) / speed * 20;
    if (cnt < 0) {
        cnt = -cnt;
    }
    
    while (cnt--) {
        this_angle = last_angle + speed / 20;
        float a = cos(this_angle / 57.3f);
        float b = cos(last_angle / 57.3f);
        horizontal_pos_x_pid_data.expect += r * (a - b);
        horizontal_pos_y_pid_data.expect += r * (sin(this_angle / 57.3f) - sin(last_angle / 57.3f));
        last_angle = this_angle;
        vTaskDelay(50);
    }
}

/**********************************************************************************************************
*�� �� ��: fly_turn
*����˵��: ԭ��ת��
*��    ��: Ŀ��Ƕ� �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void fly_turn(int targer_angle, float speed) {
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
*�� �� ��: find_pole
*����˵��: ���ҷ���Ѱ�Ҹ�
*��    ��: Ŀ��λ�� �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void find_pole(int max_distence, float speed) {
    
    int cnt;
    
    cnt = max_distence / speed * 20;
    openmv_updata_flag = 0;
    //���ҷ�ֱ���ҵ���
    while ((openmv_updata_flag & (1 << 1)) == 0) {
        //���෴����Ѱ�Ҹ�
        if (cnt == 0) {
            speed = -speed;
            max_distence = -max_distence;
            cnt = max_distence / speed * 20;
        }
        horizontal_pos_y_pid_data.expect += speed / 20;
        vTaskDelay(50);
        cnt--;
    }
    vTaskDelay(1000);
    //�ȴ������м�
    while (pole_distance > 10 || pole_distance < -10) {
        //���¸˵�λ��
        openmv_updata_flag = 0;
        vTaskDelay(50);
        if ((openmv_updata_flag & (1 << 1)) == 0)
            horizontal_pos_y_pid_data.expect += pole_distance * 0.001;
        else
            //�������Ʒɻ�λ��
            horizontal_pos_y_pid_data.expect += pole_distance * 0.005;
    }
}

/**********************************************************************************************************
*�� �� ��: find_line
*����˵��: ���·���Ѱ����
*��    ��: ����ƶ��߶� �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void find_line(int max_high, float speed) {
    
    int cnt;
    
    cnt = max_high / speed * 20;
    openmv_updata_flag = 0;
    //���·�ֱ���ҵ���
    while ((openmv_updata_flag & (1 << 2)) == 0) {
        //���෴����Ѱ����
        if (cnt == 0) {
            speed = -speed;
            max_high = -max_high;
            cnt = max_high / speed * 20;
        }
        high_pos_pid_data.expect += speed / 20;
        vTaskDelay(50);
        cnt--;
    }
    vTaskDelay(1000);
    //�ȴ������м�
    while (line_high > 10 || line_high < -10) {
        //�����ߵ�λ��
        openmv_updata_flag = 0;
        vTaskDelay(50);
        //�������Ʒɻ�λ��
        if ((openmv_updata_flag & (1 << 1)) == 0)
            high_pos_pid_data.expect -= line_high * 0.001;
        else
            high_pos_pid_data.expect -= line_high * 0.005;
    }
}

/**********************************************************************************************************
*�� �� ��: find_line
*����˵��: Ѱ��������
*��    ��: Ҫ�ߵľ��� �ٶ�
*�� �� ֵ: ʣ�����
**********************************************************************************************************/
static int find_bar_code(int targer_distan, float speed) {
    
    int cnt;
    float last_dstan;
    
    if ((targer_distan > 0 && speed < 0) || (targer_distan < 0 && speed > 0)) {
        speed = -speed;
    }
    
    cnt = targer_distan / speed * 20;
    last_dstan = targer_distan - (cnt * speed / 20);
    
    while (cnt && (openmv_updata_flag & (1 << 3)) == 0) {
        horizontal_pos_y_pid_data.expect += speed / 20;
        cnt--;
        vTaskDelay(50);
    }
    horizontal_pos_y_pid_data.expect += last_dstan;
    return cnt * speed / 20;
}

/**********************************************************************************************************
*�� �� ��: route_plan_task1
*����˵��: ·���滮����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
portTASK_FUNCTION(route_plan_task1,  parameters)
{
    int last_distance;
    //���ߵ�120cm
    fly_high(100 - 20, 50);
    vTaskDelay(2000);
    fly_turn(90, 20);
    vTaskDelay(2000);
    
    //�½���0cm
    fly_high(0 - high_pos_pid_data.expect, 50);
    vTaskDelay(200);
    
    save_throttle_control = Throttle_Control;
    save_high_expect = high_pos_pid_data.expect;
    route_plan_finish = 1;
    route_plan_stop_flag = 1;
    vTaskDelete(NULL);
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
    xTaskCreate(route_plan_task1, "route_plan_task", ROUTE_PLAN_TASK_STACK, NULL, ROUTE_PLAN_TASK_PRIORITY, &route_plan_task_handle);
}

/**********************************************************************************************************
*�� �� ��: route_plan_task_delete
*����˵��: ·���滮�������ɾ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void route_plan_task_delete(void)
{
    vTaskDelete(route_plan_task_handle);
}
