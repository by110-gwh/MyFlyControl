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
#include "laser_task.h"
#include "laser.h"
#include "uart0.h"

#include "FreeRTOS.h"
#include "task.h"

//定位超声波到杆的距离
#define DISTANCE_TO_POLE 30

//任务堆栈大小
#define ROUTE_PLAN_TASK_STACK            256
//任务优先级
#define ROUTE_PLAN_TASK_PRIORITY         11

#define open_size 30

//声明任务句柄
static xTaskHandle route_plan_task_handle;
//任务退出标志
volatile uint8_t route_plan_task_exit;

#define ABS(X)  (((X) > 0)? (X) : -(X))

/**********************************************************************************************************
*函 数 名: fly_high
*功能说明: 飞行高度控制
*形    参: 目标高度 速度
*返 回 值: 无
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
*函 数 名: fly_forward
*功能说明: 向前飞控制
*形    参: 目标位置 速度
*返 回 值: 无
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
*函 数 名: fly_right
*功能说明: 向右飞控制
*形    参: 目标位置 速度
*返 回 值: 无
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
*函 数 名: fly_forward_high
*功能说明: 原地向前45度起飞
*形    参: 目标位置 速度
*返 回 值: 无
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
*函 数 名: fly_circle
*功能说明: 不动方向绕圈
*形    参: 起始角度 终止角度 半径 速度度每秒
*返 回 值: 无
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
*函 数 名: fly_turn
*功能说明: 原地转弯
*形    参: 目标角度 速度
*返 回 值: 无
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

void local_A(void)
{
    int err1 = 0;
    int err2 = 0;
    float distance_adj_x = 0;
    float distance_adj_y = 0;
    int time_cnt = 20 * 5;
    
    openmv_updata_flag &= ~(1 << 27 | 1 << 20);
    
    while (--time_cnt && (line_fr >= -open_size + 5 || line_fr <= -open_size - 5 || line_lb >= open_size + 5 || line_lb <= open_size - 5)) {
        if (err1 > 20 || err2 > 20)
            return;
        //openmv采样到中心点的距离
        if (openmv_updata_flag & (1 << 27) && (line_fr < open_size)) {
            distance_adj_x = line_fr + open_size;
            openmv_updata_flag &= ~(1 << 27);
            err1 = 0;
        } else {
            err1++;
        }
        if (openmv_updata_flag & (1 << 20) && (line_lb > -open_size)) {
            distance_adj_y = line_lb - open_size;
            openmv_updata_flag &= ~(1 << 20);
            err2 = 0;
        } else {
            err2++;
        }
        //x方向调整
        if (distance_adj_x) {
            //速度限幅
            if (distance_adj_x > 25) {
                horizontal_pos_x_pid_data.expect -= 25 * 0.02;
                distance_adj_x -= 25 * 0.02;
            } else if (distance_adj_x < -25) {
                horizontal_pos_x_pid_data.expect -= -25 * 0.02;
                distance_adj_x -= 25 * 0.02;
            } else {
                horizontal_pos_x_pid_data.expect -= distance_adj_x * 0.02;
                distance_adj_x -= distance_adj_x * 0.02;
            }
        }
        //y方向调整
        if (distance_adj_y) {
            //速度限幅
            if (distance_adj_y > 25) {
                horizontal_pos_y_pid_data.expect -= 25 * 0.02;
                distance_adj_y -= 25 * 0.02;
            } else if (distance_adj_y < -25) {
                horizontal_pos_y_pid_data.expect -= -25 * 0.02;
                distance_adj_y -= 25 * 0.02;
            } else {
                horizontal_pos_y_pid_data.expect -= distance_adj_y * 0.02;
                distance_adj_y -= distance_adj_y * 0.02;
            }
        }
        vTaskDelay(50);
    }
    horizontal_pos_x_pid_data.expect = horizontal_pos_x_pid_data.feedback;
    horizontal_pos_y_pid_data.expect = horizontal_pos_y_pid_data.feedback;
}

void local_22(void)
{
    int err1 = 0;
    int err2 = 0;
    float distance_adj_x = 0;
    float distance_adj_y = 0;
    int time_cnt = 20 * 5;
    
    openmv_updata_flag &= ~(1 << 21 | 1 << 26);
    while (--time_cnt && (line_bl >= open_size + 5 || line_bl <= open_size - 5 || line_rf >= -open_size + 5 || line_rf <= -open_size - 5)) {
        if (err1 > 20 || err2 > 20)
            return;
        //openmv采样到中心点的距离
        if (openmv_updata_flag & (1 << 21) && (line_bl > -open_size)) {
            distance_adj_x = line_bl - open_size;
            openmv_updata_flag &= ~(1 << 21);
            err1 = 0;
        } else {
            err1++;
        }
        if (openmv_updata_flag & (1 << 26) && (line_rf < open_size)) {
            distance_adj_y = line_rf + open_size;
            openmv_updata_flag &= ~(1 << 26);
            err2 = 0;
        } else {
            err2++;
        }
        //x方向调整
        if (distance_adj_x) {
            //速度限幅
            if (distance_adj_x > 25) {
                horizontal_pos_x_pid_data.expect -= 25 * 0.02;
                distance_adj_x -= 25 * 0.02;
            } else if (distance_adj_x < -25) {
                horizontal_pos_x_pid_data.expect -= -25 * 0.02;
                distance_adj_x -= 25 * 0.02;
            } else {
                horizontal_pos_x_pid_data.expect -= distance_adj_x * 0.02;
                distance_adj_x -= distance_adj_x * 0.02;
            }
        }
        //y方向调整
        if (distance_adj_y) {
            //速度限幅
            if (distance_adj_y > 25) {
                horizontal_pos_y_pid_data.expect -= 25 * 0.02;
                distance_adj_y -= 25 * 0.02;
            } else if (distance_adj_y < -25) {
                horizontal_pos_y_pid_data.expect -= -25 * 0.02;
                distance_adj_y -= 25 * 0.02;
            } else {
                horizontal_pos_y_pid_data.expect -= distance_adj_y * 0.02;
                distance_adj_y -= distance_adj_y * 0.02;
            }
        }
        vTaskDelay(50);
    }
    horizontal_pos_x_pid_data.expect = horizontal_pos_x_pid_data.feedback;
    horizontal_pos_y_pid_data.expect = horizontal_pos_y_pid_data.feedback;
}

void local_11(void)
{
    int err1 = 0;
    int err2 = 0;
    float distance_adj_x = 0;
    float distance_adj_y = 0;
    int time_cnt = 20 * 5;
    
    openmv_updata_flag &= ~(1 << 21 | 1 << 26);
    while (--time_cnt && (line_br >= -open_size + 5 || line_br <= -open_size - 5 || line_lf >= -open_size + 5 || line_lf <= -open_size - 5)) {
        if (err1 > 20 || err2 > 20)
            return;
        //openmv采样到中心点的距离
        if (openmv_updata_flag & (1 << 21) && (line_br > -open_size)) {
            distance_adj_x = line_br + open_size;
            openmv_updata_flag &= ~(1 << 21);
            err1 = 0;
        } else {
            err1++;
        }
        if (openmv_updata_flag & (1 << 26) && (line_lf < open_size)) {
            distance_adj_y = line_lf + open_size;
            openmv_updata_flag &= ~(1 << 26);
            err2 = 0;
        } else {
            err2++;
        }
        //x方向调整
        if (distance_adj_x) {
            //速度限幅
            if (distance_adj_x > 25) {
                horizontal_pos_x_pid_data.expect -= 25 * 0.02;
                distance_adj_x -= 25 * 0.02;
            } else if (distance_adj_x < -25) {
                horizontal_pos_x_pid_data.expect -= -25 * 0.02;
                distance_adj_x -= 25 * 0.02;
            } else {
                horizontal_pos_x_pid_data.expect -= distance_adj_x * 0.02;
                distance_adj_x -= distance_adj_x * 0.02;
            }
        }
        //y方向调整
        if (distance_adj_y) {
            //速度限幅
            if (distance_adj_y > 25) {
                horizontal_pos_y_pid_data.expect -= 25 * 0.02;
                distance_adj_y -= 25 * 0.02;
            } else if (distance_adj_y < -25) {
                horizontal_pos_y_pid_data.expect -= -25 * 0.02;
                distance_adj_y -= 25 * 0.02;
            } else {
                horizontal_pos_y_pid_data.expect -= distance_adj_y * 0.02;
                distance_adj_y -= distance_adj_y * 0.02;
            }
        }
        vTaskDelay(50);
    }
    horizontal_pos_x_pid_data.expect = horizontal_pos_x_pid_data.feedback;
    horizontal_pos_y_pid_data.expect = horizontal_pos_y_pid_data.feedback;
}

static void fly_work_end_rf(int max_distance, float speed)
{
    int cnt;
    float last_dstan;
    int state = 0;
    
    if ((max_distance > 0 && speed < 0) || (max_distance < 0 && speed > 0)) {
        speed = -speed;
    }
    
    openmv_updata_flag &= ~(1 << 26);
    cnt = max_distance / speed * 20;
    last_dstan = max_distance - (cnt * speed / 20);
    
    while (cnt-- && !(openmv_updata_flag & (1 << 26))) {
        state = 1;
        horizontal_pos_y_pid_data.expect += speed / 20;
        vTaskDelay(50);
    }
    horizontal_pos_y_pid_data.expect += last_dstan;
    
    if (state) {
        if (speed > 0)
            fly_forward(20, 25);
        else
            fly_forward(-20, 25);
    }
}

static void fly_work_end_lf(int max_distance, float speed)
{
    int cnt;
    float last_dstan;
    int state = 0;
    
    if ((max_distance > 0 && speed < 0) || (max_distance < 0 && speed > 0)) {
        speed = -speed;
    }
    
    openmv_updata_flag &= ~(1 << 22);
    cnt = max_distance / speed * 20;
    last_dstan = max_distance - (cnt * speed / 20);
    
    while (cnt-- && !(openmv_updata_flag & (1 << 22))) {
        state = 1;
        horizontal_pos_y_pid_data.expect += speed / 20;
        vTaskDelay(50);
    }
    horizontal_pos_y_pid_data.expect += last_dstan;
    
    if (state) {
        if (speed > 0)
            fly_forward(20, 25);
        else
            fly_forward(-20, 25);
    }
}

static void fly_work_end_lb(int max_distance, float speed)
{
    int cnt;
    float last_dstan;
    int state = 0;
    
    if ((max_distance > 0 && speed < 0) || (max_distance < 0 && speed > 0)) {
        speed = -speed;
    }
    
    openmv_updata_flag &= ~(1 << 20);
    cnt = max_distance / speed * 20;
    last_dstan = max_distance - (cnt * speed / 20);
    
    while (cnt-- && !(openmv_updata_flag & (1 << 20))) {
        state = 1;
        horizontal_pos_y_pid_data.expect += speed / 20;
        vTaskDelay(50);
    }
    horizontal_pos_y_pid_data.expect += last_dstan;
    
    if (state) {
        if (speed > 0)
            fly_forward(20, 25);
        else
            fly_forward(-20, 25);
    }
}

void find_landing(void)
{
    float distance_adj_x = 0;
    float distance_adj_y = 0;
    
    openmv_updata_flag &= ~(1 << 1);
    while (land_x > 3 || land_x < -3 || land_y > 3 || land_y < -3) {
        //openmv采样到中心点的距离
        if (openmv_updata_flag & (1 << 1)) {
            distance_adj_x = land_y;
            distance_adj_x -= horizontal_pos_x_pid_data.feedback - horizontal_pos_x_pid_data.expect;
            distance_adj_y = land_x;
            distance_adj_y -= horizontal_pos_y_pid_data.feedback - horizontal_pos_y_pid_data.expect;
            openmv_updata_flag &= ~(1 << 1);
        }
        //x方向调整
        if (distance_adj_x) {
            //速度限幅
            if (distance_adj_x > 50) {
                horizontal_pos_x_pid_data.expect -= 50 * 0.015;
                distance_adj_x -= 50 * 0.015;
            } else if (distance_adj_x < -50) {
                horizontal_pos_x_pid_data.expect -= -50 * 0.015;
                distance_adj_x -= -50 * 0.015;
            } else {
                horizontal_pos_x_pid_data.expect -= distance_adj_x * 0.015;
                distance_adj_x -= distance_adj_x * 0.015;
            }
        }
        //y方向调整
        if (distance_adj_y) {
            //速度限幅
            if (distance_adj_y > 50) {
                horizontal_pos_y_pid_data.expect -= 50 * 0.015;
                distance_adj_y -= 50 * 0.015;
            } else if (distance_adj_y < -50) {
                horizontal_pos_y_pid_data.expect -= -50 * 0.015;
                distance_adj_y -= -50 * 0.015;
            } else {
                horizontal_pos_y_pid_data.expect -= distance_adj_y * 0.015;
                distance_adj_y -= distance_adj_y * 0.015;
            }
        }
        vTaskDelay(50);
    }
    horizontal_pos_x_pid_data.expect = horizontal_pos_x_pid_data.feedback;
    horizontal_pos_y_pid_data.expect = horizontal_pos_y_pid_data.feedback;
}

///**********************************************************************************************************
//*函 数 名: find_pole_sr04
//*功能说明: 通过超声波左右飞行寻找杆
//*形    参: 最远距离 速度
//*返 回 值: 无
//**********************************************************************************************************/
//static void find_pole_sr04(int max_distence, float speed)
//{
//    int cnt;
//    
//    cnt = max_distence / speed * 20;
//    //向前飞直到找到杆
//    while (sr04_distance == 0 && cnt) {
//        //往相反方向寻找杆
//        if (cnt == 0) {
//            return;
//        }
//        horizontal_pos_y_pid_data.expect -= speed / 20;
//        vTaskDelay(50);
//        cnt--;
//    }
//    
//    //向左飞定杆的距离
//    while (sr04_distance > DISTANCE_TO_POLE + 5 || sr04_distance < DISTANCE_TO_POLE - 5) {
//        //更新杆的位置
//		//比例控制飞机位置
//		if (sr04_distance) {
//            //速度限幅
//            if (sr04_distance - DISTANCE_TO_POLE > 50)
//                horizontal_pos_x_pid_data.expect -= (70 - DISTANCE_TO_POLE) * 0.015;
//            else
//                horizontal_pos_x_pid_data.expect -= (sr04_distance - DISTANCE_TO_POLE) * 0.015;
//        } else
//            horizontal_pos_y_pid_data.expect -= speed / 2 / 20;  
//        vTaskDelay(50);
//    }
//	horizontal_pos_x_pid_data.expect = horizontal_pos_x_pid_data.feedback;
//}

static void laser_start(void)
{
    laser_duty = 10;
    laser_cycle = 100;
    laser_time = 0xFF;
}

static void laser_stop(void)
{
    laser_duty = 0;
    laser_cycle = 100;
    laser_time = 0xFF;
    laser_on();
}


/**********************************************************************************************************
*函 数 名: route_plan_task1
*功能说明: 路径规划任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(route_plan_task1,  parameters)
{
    float last_distance_adj;
    int last_distance;
    int i;
    
    //升高到90cm
    fly_high(130 - high_pos_pid_data.expect, 50);
    vTaskDelay(1000);
    //移动到A
    fly_right(-20, 20);
    fly_right(-180, 40);
    fly_right(-20, 20);
    fly_forward(50, 30);
    
    //定A
    local_A();
    laser_on();
    vTaskDelay(200);
    laser_off();

    //到28
    fly_right(-50, 25);
    vTaskDelay(200);
    laser_start();

    //28到22
    fly_forward(300, 25);
    fly_work_end_rf(50, 25);
    laser_stop();
    local_22();

    //到15
    fly_right(50, 20);
    vTaskDelay(200);
    laser_start();

    //到A
    fly_forward(-300, 25);
    fly_work_end_lb(-50, 25);
    laser_stop();
    local_A();
    
    //到14
    fly_right(50, 20);
    fly_forward(150, 25);
    vTaskDelay(500);
    laser_start();
    
    //到11
    fly_forward(150, 25);
    fly_work_end_lf(50, 25);
    laser_stop();
    local_11();
    
    //到9
    fly_right(50, 20);
    fly_forward(-100, 25);
    vTaskDelay(500);
    laser_off();
    laser_start();
    
    //到10
    fly_forward(-50, 25);
    fly_work_end_lb(-50, 25);
    laser_stop();
    
    //到8
    fly_right(50, 20);
    vTaskDelay(200);
    laser_start();
    
    //到5
    fly_forward(150, 25);
    fly_work_end_rf(25, 25);
    laser_stop();
    local_22();
    
    //到1
    fly_right(50, 20);
    vTaskDelay(200);
    laser_start();
    
    //到起降点
    fly_forward(-150, 25);
    vTaskDelay(500);
    laser_stop();
    laser_off();
    
    uart0_send_data(0x01);
    fly_forward(-200, 25);
    fly_high(100 - high_pos_pid_data.expect, 50);
    vTaskDelay(1000);
    
    find_landing();

    //下降到0cm
    fly_high(-20 - high_pos_pid_data.expect, 50);
    
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
    fly_high(70 - high_pos_pid_data.expect, 30);
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
*函 数 名: route_plan_task3
*功能说明: 路径规划任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(route_plan_task3,  parameters)
{
    //升高到120cm
    fly_high(70 - high_pos_pid_data.expect, 30);
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
    if (fly_task_num == 2)
        xTaskCreate(route_plan_task2, "route_plan_task", ROUTE_PLAN_TASK_STACK, NULL, ROUTE_PLAN_TASK_PRIORITY, &route_plan_task_handle);
    else if (fly_task_num == 3)
        xTaskCreate(route_plan_task3, "route_plan_task", ROUTE_PLAN_TASK_STACK, NULL, ROUTE_PLAN_TASK_PRIORITY, &route_plan_task_handle);
    else
        xTaskCreate(route_plan_task1, "route_plan_task", ROUTE_PLAN_TASK_STACK, NULL, ROUTE_PLAN_TASK_PRIORITY, &route_plan_task_handle);
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
