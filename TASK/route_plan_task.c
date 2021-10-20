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
*函 数 名: fly_forward
*功能说明: 向前飞控制
*形    参: 目标位置 速度
*返 回 值: 无
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
*函 数 名: fly_right
*功能说明: 向右飞控制
*形    参: 目标位置 速度
*返 回 值: 无
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
*函 数 名: fly_forward_high
*功能说明: 原地向前45度起飞
*形    参: 目标位置 速度
*返 回 值: 无
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
*函 数 名: fly_circle
*功能说明: 不动方向绕圈
*形    参: 起始角度 终止角度 半径 速度度每秒
*返 回 值: 无
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
*函 数 名: fly_turn
*功能说明: 原地转弯
*形    参: 目标角度 速度
*返 回 值: 无
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
*函 数 名: find_pole
*功能说明: 左右飞行寻找杆
*形    参: 目标位置 速度
*返 回 值: 无
**********************************************************************************************************/
static void find_pole(int max_distence, float speed) {
    
    int cnt;
    
    cnt = max_distence / speed * 20;
    openmv_updata_flag = 0;
    //向右飞直到找到杆
    while ((openmv_updata_flag & (1 << 1)) == 0) {
        //往相反方向寻找杆
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
    //等待杆在中间
    while (pole_distance > 10 || pole_distance < -10) {
        //更新杆的位置
        openmv_updata_flag = 0;
        vTaskDelay(50);
        if ((openmv_updata_flag & (1 << 1)) == 0)
            horizontal_pos_y_pid_data.expect += pole_distance * 0.001;
        else
            //比例控制飞机位置
            horizontal_pos_y_pid_data.expect += pole_distance * 0.005;
    }
}

/**********************************************************************************************************
*函 数 名: find_pole_sr04
*功能说明: 通过超声波左右飞行寻找杆
*形    参: 最远距离 速度
*返 回 值: 无
**********************************************************************************************************/
static void find_pole_sr04(int max_distence, float speed) {
    
    int cnt;
    
    cnt = max_distence / speed * 20;
    //向前飞直到找到杆
    while (sr04_distance == 0) {
        //往相反方向寻找杆
        if (cnt == 0) {
            speed = -speed;
            max_distence = -max_distence;
            cnt = max_distence / speed * 20;
        }
        horizontal_pos_y_pid_data.expect += speed / 20;
        vTaskDelay(50);
        cnt--;
    }
    
    //向右飞定杆的距离
    while (sr04_distance > 35 || sr04_distance < 25) {
        //更新杆的位置
		//比例控制飞机位置
		if (sr04_distance) {
            //速度限幅
            if (sr04_distance - 30 > 50)
                horizontal_pos_x_pid_data.expect -= (70 - 30) * 0.015;
            else
                horizontal_pos_x_pid_data.expect -= (sr04_distance - 30) * 0.015;
        } else
            horizontal_pos_y_pid_data.expect += speed / 2 / 20;  
        vTaskDelay(50);
    }
	horizontal_pos_x_pid_data.expect = horizontal_pos_x_pid_data.feedback;
}

/**********************************************************************************************************
*函 数 名: find_line
*功能说明: 寻找条形码
*形    参: 要走的距离 速度
*返 回 值: 剩余距离
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
*函 数 名: route_plan_task1
*功能说明: 路径规划任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(route_plan_task1,  parameters)
{
    int last_distance;
    
    //升高到120cm
    fly_high(70 - 20, 30);
    vTaskDelay(1000);
    //定杆距离
	find_pole_sr04(100, 20);
    //找条形码
    last_distance = find_bar_code(250, 20);
    //成功识别到条形码
    if (last_distance > 50) {
        //停留一下
        vTaskDelay(5000);
        //发出提示
        beep_duty = 50;
    }
    //飞完剩下的距离
    fly_forward(last_distance, 20);
    
    //下降到0cm
    fly_high(0 - high_pos_pid_data.expect, 35);
    
    vTaskDelay(400);
    save_throttle_control = Throttle_Control;
    save_high_expect = high_pos_pid_data.expect;
    route_plan_finish = 1;
    route_plan_stop_flag = 1;
    route_plan_task_delete();
}

/**********************************************************************************************************
*函 数 名: route_plan_task2
*功能说明: 路径规划任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(route_plan_task2,  parameters)
{
    //升高到120cm
    fly_high(70 - 20, 30);
    vTaskDelay(1000);
    //下降到0cm
    fly_high(0 - high_pos_pid_data.expect, 35);
    
    vTaskDelay(400);
    save_throttle_control = Throttle_Control;
    save_high_expect = high_pos_pid_data.expect;
    route_plan_finish = 1;
    route_plan_stop_flag = 1;
    route_plan_task_delete();
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
    if (fly_task_num == 1)
        xTaskCreate(route_plan_task1, "route_plan_task", ROUTE_PLAN_TASK_STACK, NULL, ROUTE_PLAN_TASK_PRIORITY, &route_plan_task_handle);
    else
        xTaskCreate(route_plan_task2, "route_plan_task", ROUTE_PLAN_TASK_STACK, NULL, ROUTE_PLAN_TASK_PRIORITY, &route_plan_task_handle);
}

/**********************************************************************************************************
*函 数 名: route_plan_task_delete
*功能说明: 路径规划相关任务删除
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void route_plan_task_delete(void)
{
    if (route_plan_task_handle) {
        void *p_temp = route_plan_task_handle;
        route_plan_task_handle = NULL;
        vTaskDelete(p_temp);
    }
}
