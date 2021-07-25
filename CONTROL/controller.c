#include "controller.h"
#include "remote_control.h"
#include "ahrs_aux.h"
#include "attitude_self_stabilization.h"
#include "high_attitude_stabilization.h"
#include "horizontal_attitude_stabilization.h"
#include "route_plan_attitude_stabilization.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "high_control.h"
#include "horizontal_control.h"
#include "motor_output.h"
#include "route_plan_task.h"
#include "beep_task.h"
#include "navigation.h"

uint8_t controller_last_state;
uint8_t controller_state;

/**********************************************************************************************************
*函 数 名: control_init
*功能说明: 控制器初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void controller_init()
{
    angle_control_init();
    gyro_control_init();
    high_control_init();
    horizontal_control_init();
    controller_state = 0;
}

/**********************************************************************************************************
*函 数 名: constrain_float
*功能说明: 限制最大最小值
*形    参: 要限制的值 最小值 最大值
*返 回 值: 输出值
**********************************************************************************************************/
static float constrain_float(float amt, float low, float high)
{
	return ((amt) < (low) ? (low) : ((amt) > (high)? (high) : (amt)));
}

/**********************************************************************************************************
*函 数 名: throttle_angle_compensate
*功能说明: 油门倾角补偿
*形    参: 原有门量
*返 回 值: 补偿后的油门量
**********************************************************************************************************/
static uint16_t throttle_angle_compensate(uint16_t throttle)
{
	uint16_t throttle_output;
	float CosPitch_CosRoll = Cos_Pitch * Cos_Roll;
	//补偿限制
	if(CosPitch_CosRoll<=0.50f)
		CosPitch_CosRoll=0.50f;
	//大于起转油门量
	if(throttle >= 1000) {
		//油门倾角补偿
		throttle_output = 1000 + (throttle - 1000) / CosPitch_CosRoll;
		throttle_output = (uint16_t)(constrain_float(throttle_output, 1000, 2000));
	} else
		throttle_output = throttle;
	return throttle_output;
}

/**********************************************************************************************************
*函 数 名: controller_run
*功能说明: 控制器执行
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void controller_run()
{
    static uint16_t route_plan_enter_cnt;
    controller_last_state = controller_state;
    
    if ((rc_raw_data[4] < 1250 || Throttle_Control < 500) && (controller_state != 4 || high_pos_pid_data.expect < 5)) {
        //纯姿态模式
        controller_state = 1;
        route_plan_enter_cnt = 0;
    } else if (rc_raw_data[6] > 1500 && route_plan_enter_cnt < 200) {
        route_plan_enter_cnt++;
    } else if (rc_raw_data[6] > 1500 && route_plan_enter_cnt == 200) {
        //路径规划模式
        controller_state = 4;
        route_plan_enter_cnt = 0;
    } else if (rc_raw_data[4] < 1750) {
        //定高模式
        controller_state = 2;
        route_plan_enter_cnt = 0;
    } else {
        //定点模式
        controller_state = 3;
        route_plan_enter_cnt = 0;
    }
    
    //纯姿态模式
    if (controller_state == 1) {
        //开始运行定高控制器
        if (controller_last_state != 2) {
            high_pos_pid_integrate_reset();
            high_speed_pid_integrate_reset();
            horizontal_pos_x_pid_integrate_reset();
            horizontal_pos_y_pid_integrate_reset();
            horizontal_speed_x_pid_integrate_reset();
            horizontal_speed_y_pid_integrate_reset();
            horizontal_pos_x_pid_data.short_circuit_flag = 1;
            horizontal_pos_y_pid_data.short_circuit_flag = 1;
        }
        //纯姿态控制器
        attitude_self_stabilization_control();
        //角度环控制器
        angle_control();
        //角速度控制器
        gyro_control();
        //油门补偿
        throttle_motor_output = throttle_angle_compensate(Throttle_Control + 1000);
    //定高模式
    } else if (controller_state == 2) {
        //开始运行定高控制器
        if (controller_last_state != 2) {
            horizontal_pos_x_pid_integrate_reset();
            horizontal_pos_y_pid_integrate_reset();
            horizontal_speed_x_pid_integrate_reset();
            horizontal_speed_y_pid_integrate_reset();
            horizontal_pos_x_pid_data.short_circuit_flag = 1;
            horizontal_pos_y_pid_data.short_circuit_flag = 1;
        }
        //定高控制器
        high_attitude_stabilization_control();
        //高度环控制器
        high_control();
        //角度环控制器
        angle_control();
        //角速度控制器
        gyro_control();
        //油门补偿
        throttle_motor_output = throttle_angle_compensate(high_speed_pid_data.control_output + 1700);
    //定点模式
    } else if (controller_state == 3) {
        //定高控制器
        horizontal_attitude_stabilization_control();
        //位置控制器
        horizontal_control();
        //高度环控制器
        high_control();
        //角度环控制器
        angle_control();
        //角速度控制器
        gyro_control();
        //油门补偿
        throttle_motor_output = throttle_angle_compensate(high_speed_pid_data.control_output + 1700);
    //路径规划模式
    } else if (controller_state == 4) {
        //第一次运行路径控制器
        if (controller_last_state != 4) {
            high_pos_pid_data.expect = 20;
            yaw_angle_pid_data.short_circuit_flag = 0;
            yaw_angle_pid_data.expect = Yaw;
            horizontal_pos_y_pid_data.short_circuit_flag = 0;
            horizontal_pos_y_pid_data.expect = pos_y;
            horizontal_pos_x_pid_data.short_circuit_flag = 0;
            horizontal_pos_x_pid_data.expect = pos_x;
            save_throttle_control = Throttle_Control;
            route_plan_finish = 0;
            route_plan_stop_flag = 0;
            route_plan_task_create();
        }
        //路径控制器
        route_plan_attitude_stabilization_control();
        //如果路径自动运行完毕
        if (route_plan_stop_flag) {
            //电机停转
            throttle_motor_output = 0;
            yaw_gyro_pid_data.control_output = 0;
            pitch_gyro_pid_data.control_output = 0;
            roll_gyro_pid_data.control_output = 0;
        } else {
            //位置控制器
            horizontal_control();
            //高度环控制器
            high_control();
            //角度环控制器
            angle_control();
            //角速度控制器
            gyro_control();
            //油门补偿
            throttle_motor_output = throttle_angle_compensate(high_speed_pid_data.control_output + 1700);
        }
    }
}
