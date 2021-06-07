#include "controller.h"
#include "remote_control.h"
#include "ahrs_aux.h"
#include "attitude_self_stabilization.h"
#include "high_attitude_stabilization.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "high_control.h"
#include "motor_output.h"

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
	if(CosPitch_CosRoll<=0.50)
		CosPitch_CosRoll=0.50;
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
    controller_last_state = controller_state;
    
    if (rc_raw_data[4] < rc_calibration_data[4].middle) {
        //纯姿态模式
        controller_state = 1;
    } else if (Throttle_Control > 0) {
        //定高模式
        controller_state = 2;
    }
    
    if (controller_state == 1) {
        //第一次运行纯姿态控制器
        if (controller_last_state != 1) {
            high_pos_pid.short_circuit_flag = 1;
            high_vel_pid.short_circuit_flag = 1;
        }
        //纯姿态控制器
        attitude_self_stabilization_control();
    } else if (controller_state == 2) {
        //第一次运行定高控制器
        if (controller_last_state != 2) {
            high_vel_pid.short_circuit_flag = 0;
            high_pos_pid_integrate_reset();
            high_vel_pid_integrate_reset();
        }
        //定高控制器
        high_attitude_stabilization_control();
    }
    
    //高度环控制器
    high_control();
    //角度环控制器
    angle_control();
    //角速度控制器
    gyro_control();
    
    
    if (controller_state == 1) {
        //油门补偿
        throttle_motor_output = throttle_angle_compensate(high_vel_pid.control_output + 1000);
    } else if (controller_state == 2) {
        //油门补偿
        throttle_motor_output = throttle_angle_compensate(high_vel_pid.control_output + 1100);
    }
    
}
