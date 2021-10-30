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
#include "sr04.h"

#include "FreeRTOS.h"
#include "task.h"

//��λ���������˵ľ���
#define DISTANCE_TO_POLE 30

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
*�� �� ��: fly_forward_high
*����˵��: ԭ����ǰ45�����
*��    ��: Ŀ��λ�� �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void fly_forward_high(int targer_distan, float speed)
{
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
static void fly_circle(int start_angle, int end_angle, int r, float speed)
{
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
*�� �� ��: find_pole
*����˵��: ���ҷ���Ѱ�Ҹ�
*��    ��: Ŀ��λ�� �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void find_pole(int max_distence, float speed)
{
    
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
*�� �� ��: find_pole_sr04
*����˵��: ͨ�����������ҷ���Ѱ�Ҹ�
*��    ��: ��Զ���� �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void find_pole_sr04(int max_distence, float speed)
{
    int cnt;
    
    cnt = max_distence / speed * 20;
    //��ǰ��ֱ���ҵ���
    while (sr04_distance == 0) {
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
    
    //����ɶ��˵ľ���
    while (sr04_distance > DISTANCE_TO_POLE + 5 || sr04_distance < DISTANCE_TO_POLE - 5) {
        //���¸˵�λ��
		//�������Ʒɻ�λ��
		if (sr04_distance) {
            //�ٶ��޷�
            if (sr04_distance - DISTANCE_TO_POLE > 50)
                horizontal_pos_x_pid_data.expect -= (70 - DISTANCE_TO_POLE) * 0.015;
            else
                horizontal_pos_x_pid_data.expect -= (sr04_distance - DISTANCE_TO_POLE) * 0.015;
        } else
            horizontal_pos_y_pid_data.expect += speed / 2 / 20;  
        vTaskDelay(50);
    }
	horizontal_pos_x_pid_data.expect = horizontal_pos_x_pid_data.feedback;
}

/**********************************************************************************************************
*�� �� ��: run_out_pole
*����˵��: �����
*��    ��: ��Զ���� �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void run_out_pole(int distence, float speed)
{
    int cnt;
    
    cnt = distence / speed * 20;
    
    while (cnt) {
        if (sr04_distance) {
            cnt = distence / speed * 20;
        }
        horizontal_pos_y_pid_data.expect += speed / 20;
        vTaskDelay(50);
        cnt--;
    }
    
	horizontal_pos_x_pid_data.expect = horizontal_pos_x_pid_data.feedback;
}

/**********************************************************************************************************
*�� �� ��: find_bar_code
*����˵��: Ѱ��������
*��    ��: Ҫ�ߵľ��� �ٶ�
*�� �� ֵ: ʣ�����
**********************************************************************************************************/
static int find_bar_code(int targer_distan, float speed)
{
    //�������ľ����������
    float distance_adj_x = 0;
    int cnt;
    float last_dstan;
    
    if ((targer_distan > 0 && speed < 0) || (targer_distan < 0 && speed > 0)) {
        speed = -speed;
    }
    
    cnt = targer_distan / speed * 20;
    last_dstan = targer_distan - (cnt * speed / 20);
    
    while (cnt && (openmv_updata_flag & (1 << 3)) == 0) {
        
        //���������������ľ���
        if (sr04_distance) {
            distance_adj_x = sr04_distance - DISTANCE_TO_POLE;
            distance_adj_x -= horizontal_pos_x_pid_data.feedback - horizontal_pos_x_pid_data.expect;
        }
        //x�������
        if (distance_adj_x) {
            //�ٶ��޷�
            if (distance_adj_x > 50) {
                horizontal_pos_x_pid_data.expect -= 50 * 0.015;
                distance_adj_x -= 50 * 0.015;
            } else {
                horizontal_pos_x_pid_data.expect -= distance_adj_x * 0.015;
                distance_adj_x -= distance_adj_x * 0.015;
            }
        }
        
        horizontal_pos_y_pid_data.expect += speed / 20;
        cnt--;
        vTaskDelay(50);
    }
    horizontal_pos_y_pid_data.expect += last_dstan;
    return cnt * speed / 20;
}

/**********************************************************************************************************
*�� �� ��: line_patrol
*����˵��: ��������
*��    ��: Ҫ�ߵľ��� �ٶ�
*�� �� ֵ: ʣ�����
**********************************************************************************************************/
static void line_patrol(int targer_distan, float speed)
{
    //�������ľ����������
    float distance_adj_x = 0;
    int cnt;
    float last_dstan;
    
    if ((targer_distan > 0 && speed < 0) || (targer_distan < 0 && speed > 0)) {
        speed = -speed;
    }
    
    cnt = targer_distan / speed * 20;
    last_dstan = targer_distan - (cnt * speed / 20);
    
    while (cnt) {
        
        //���������������ľ���
        if (sr04_distance) {
            distance_adj_x = sr04_distance - DISTANCE_TO_POLE;
            distance_adj_x -= horizontal_pos_x_pid_data.feedback - horizontal_pos_x_pid_data.expect;
        }
        //x�������
        if (distance_adj_x) {
            //�ٶ��޷�
            if (distance_adj_x > 50) {
                horizontal_pos_x_pid_data.expect -= 50 * 0.015;
                distance_adj_x -= 50 * 0.015;
            } else {
                horizontal_pos_x_pid_data.expect -= distance_adj_x * 0.015;
                distance_adj_x -= distance_adj_x * 0.015;
            }
        }
        
        horizontal_pos_y_pid_data.expect += speed / 20;
        cnt--;
        vTaskDelay(50);
    }
    horizontal_pos_y_pid_data.expect += last_dstan;
}

/**********************************************************************************************************
*�� �� ��: front_follow_line
*����˵��: ��ǰ����Ѱ������һ��·��
*��    ��: �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void front_follow_line(float speed)
{
    //���ҵ�������
    float distance_adj_x = 0;
    
    //��ǰ��ֱ���ҵ���һ��·��
    while ((openmv_updata_flag & (1 << 4 | 1 << 5)) == 0) {
        //openmv��������ƫ����
        if (openmv_updata_flag & (1 << 3)) {
            openmv_updata_flag &= ~(1 << 3);
            distance_adj_x = front_line_offset;
            distance_adj_x -= horizontal_pos_x_pid_data.feedback - horizontal_pos_x_pid_data.expect;
        }
        //x�������
        if (distance_adj_x) {
            //�ٶ��޷�
            if (distance_adj_x > 750) {
                horizontal_pos_x_pid_data.expect -= 50 * 0.001;
                distance_adj_x -= 50 * 0.001;
            } else {
                horizontal_pos_x_pid_data.expect -= distance_adj_x * 0.001;
                distance_adj_x -= distance_adj_x * 0.001;
            }
        }
        
        //��ǰ�ƶ�
        horizontal_pos_y_pid_data.expect += speed / 20;
        vTaskDelay(50);
    }
}

/**********************************************************************************************************
*�� �� ��: front_follow_line_w
*����˵��: ͨ���ı�ƫ���ķ�ʽ����ǰ����Ѱ������һ��·��
*��    ��: �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void front_follow_line_w(float speed)
{
    //��ǰ��ֱ���ҵ���һ��·��
    while ((openmv_updata_flag & (1 << 4 | 1 << 5)) == 0) {
        //openmv��������ƫ����
        if (openmv_updata_flag & (1 << 3)) {
            openmv_updata_flag &= ~(1 << 3);
            yaw_angle_pid_data.expect += front_line_offset * 0.001;
        }
        
        //��ǰ�ƶ�
        horizontal_pos_y_pid_data.expect += speed / 20;
        vTaskDelay(50);
    }
}

/**********************************************************************************************************
*�� �� ��: rotate_around_pole
*����˵��: �Ƹ���ת�����߷�ʽ
*��    ��: �Ƕ� �ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void rotate_around_pole(int targer_angle, float speed)
{
    int cnt;
    float turned_angle = 0;
    
    if (speed < 0) {
        speed = -speed;
    }
    while (1) {
        fly_forward(20, 20);
        //��ǰ��ֱ��Զ���
        while (sr04_distance) {
            //�ٶ��޷�
            if (sr04_distance - DISTANCE_TO_POLE > 50)
                horizontal_pos_x_pid_data.expect -= (70 - DISTANCE_TO_POLE) * 0.015;
            else
                horizontal_pos_x_pid_data.expect -= (sr04_distance - DISTANCE_TO_POLE) * 0.015;
            horizontal_pos_y_pid_data.expect += speed / 20;
            vTaskDelay(50);
        }
        horizontal_pos_y_pid_data.expect = horizontal_pos_y_pid_data.feedback;
        
        //�ı�ƫ�������ҵ���
        cnt = 10 / speed * 20;
        while (cnt--) {
            yaw_angle_pid_data.expect += speed / 20;
            turned_angle += speed / 20;
            if (yaw_angle_pid_data.expect > 180)
                yaw_angle_pid_data.expect -= 360;
            if (yaw_angle_pid_data.expect < -180)
                yaw_angle_pid_data.expect += 360;
            //ת���ƶ��Ƕ�
            if (turned_angle >= targer_angle)
                return;
            vTaskDelay(50);
        }
        while (sr04_distance) {
            yaw_angle_pid_data.expect += speed / 20;
            turned_angle += speed / 20;
            if (yaw_angle_pid_data.expect > 180)
                yaw_angle_pid_data.expect -= 360;
            if (yaw_angle_pid_data.expect < -180)
                yaw_angle_pid_data.expect += 360;
            //ת���ƶ��Ƕ�
            if (turned_angle >= targer_angle)
                return;
            vTaskDelay(50);
        }
        turned_angle -= yaw_angle_pid_data.expect - yaw_angle_pid_data.feedback;
        yaw_angle_pid_data.expect = yaw_angle_pid_data.feedback;
    }
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
    
    //���ߵ�90cm
    fly_high(90 - high_pos_pid_data.expect, 30);
    vTaskDelay(1000);
    //���˾���
	find_pole_sr04(100, 20);
    //��������
    last_distance = find_bar_code(300, 20);
    //�ɹ�ʶ��������
    if (last_distance > 50) {
        //ͣ��һ��
        vTaskDelay(5000);
        //������ʾ
        beep_duty = 50;
    }
    //����ʣ�µľ���
    line_patrol(last_distance, 20);
    //���˾���
	find_pole_sr04(100, 20);
    //�뿪��
    run_out_pole(10, 20);
    //ת90��
    fly_turn(90, 20);
    vTaskDelay(2000);
    //���˾���
	find_pole_sr04(100, 20);
    //�뿪��
    run_out_pole(20, 20);
    //ת90��
    fly_turn(90, 20);
    vTaskDelay(2000);
    //���˾���
	find_pole_sr04(100, 20);
    //��������
    last_distance = find_bar_code(300, 20);
    //�ɹ�ʶ��������
    if (last_distance > 50) {
        //ͣ��һ��
        vTaskDelay(5000);
        //������ʾ
        beep_duty = 50;
    }
    //����ʣ�µľ���
    line_patrol(last_distance, 20);
    
//    rotate_around_pole(45, 20);
    vTaskDelay(2000);
    
    //�½���0cm
    fly_high(-10 - high_pos_pid_data.expect, 35);
    
    vTaskDelay(400);
    save_throttle_control = Throttle_Control;
    save_high_expect = high_pos_pid_data.expect;
    route_plan_finish = 1;
    route_plan_stop_flag = 1;
    route_plan_task_delete();
}

/**********************************************************************************************************
*�� �� ��: route_plan_task2
*����˵��: ·���滮����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
portTASK_FUNCTION(route_plan_task2,  parameters)
{
    //���ߵ�120cm
    fly_high(70 - high_pos_pid_data.expect, 30);
    vTaskDelay(1000);
    //�½���0cm
    fly_high(0 - high_pos_pid_data.expect, 35);
    
    vTaskDelay(400);
    save_throttle_control = Throttle_Control;
    save_high_expect = high_pos_pid_data.expect;
    route_plan_finish = 1;
    route_plan_stop_flag = 1;
    route_plan_task_delete();
}

/**********************************************************************************************************
*�� �� ��: route_plan_task3
*����˵��: ·���滮����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
portTASK_FUNCTION(route_plan_task3,  parameters)
{
    //���ߵ�120cm
    fly_high(70 - high_pos_pid_data.expect, 30);
    vTaskDelay(1000);
    //�½���0cm
    fly_high(0 - high_pos_pid_data.expect, 35);
    
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
    if (fly_task_num == 2)
        xTaskCreate(route_plan_task2, "route_plan_task", ROUTE_PLAN_TASK_STACK, NULL, ROUTE_PLAN_TASK_PRIORITY, &route_plan_task_handle);
    else if (fly_task_num == 3)
        xTaskCreate(route_plan_task3, "route_plan_task", ROUTE_PLAN_TASK_STACK, NULL, ROUTE_PLAN_TASK_PRIORITY, &route_plan_task_handle);
    else
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
    if (route_plan_task_handle) {
        void *p_temp = route_plan_task_handle;
        route_plan_task_handle = NULL;
        vTaskDelete(p_temp);
    }
}
