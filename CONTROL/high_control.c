#include "high_control.h"
#include "pid.h"
#include "navigation.h"

pid_controler_t high_pos_pid;
pid_controler_t high_vel_pid;

/**********************************************************************************************************
*函 数 名: high_pos_control_pid_set
*功能说明: 高度位置环pid调整函数
*形    参: 10p参数 10i参数 10d参数
*返 回 值: 无
**********************************************************************************************************/
void high_pos_control_pid_set(uint8_t p, uint8_t i, uint8_t d)
{
    high_pos_pid.kp = p / 10.;
    high_pos_pid.ki = i / 10.;
    high_pos_pid.kd = d / 10.;
}

/**********************************************************************************************************
*函 数 名: high_pos_control_init
*功能说明: 高度pid结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void high_control_init()
{
    //高度位置pid参数初始化
    high_pos_pid.last_expect = 0;
    high_pos_pid.expect = 0;
    high_pos_pid.feedback = 0;

    high_pos_pid.err = 0;
    high_pos_pid.last_err = 0;
    high_pos_pid.err_max = 0;

    high_pos_pid.integrate_separation_err = 0;
    high_pos_pid.integrate = 0;
    high_pos_pid.integrate_max = 0;

    high_pos_pid.dis_err = 0;

    high_pos_pid.kp = 1;
    high_pos_pid.ki = 0;
    high_pos_pid.kd = 0;
    
    high_pos_pid.feedforward_kp = 0;
    high_pos_pid.feedforward_kd = 0;

    high_pos_pid.control_output = 0;
    high_pos_pid.control_output_limit = 0;

    high_pos_pid.short_circuit_flag = 0;
    high_pos_pid.err_callback = NULL;
    high_pos_pid.pri_data = NULL;
    
    
    //高度速度pid参数初始化
    high_vel_pid.last_expect = 0;
    high_vel_pid.expect = 0;
    high_vel_pid.feedback = 0;

    high_vel_pid.err = 0;
    high_vel_pid.last_err = 0;
    high_vel_pid.err_max = 0;

    high_vel_pid.integrate_separation_err = 0;
    high_vel_pid.integrate = 0;
    high_vel_pid.integrate_max = 0;

    high_vel_pid.dis_err = 0;

    high_vel_pid.kp = 1;
    high_vel_pid.ki = 0;
    high_vel_pid.kd = 0;
    
    high_vel_pid.feedforward_kp = 0;
    high_vel_pid.feedforward_kd = 0;

    high_vel_pid.control_output = 0;
    high_vel_pid.control_output_limit = 800;

    high_vel_pid.short_circuit_flag = 0;
    high_vel_pid.err_callback = NULL;
    high_vel_pid.pri_data = NULL;
}

/**********************************************************************************************************
*函 数 名: high_pos_pid_integrate_reset
*功能说明: 高度位置环pid积分清零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void high_pos_pid_integrate_reset()
{
	high_pos_pid.integrate = 0;
	high_pos_pid.pid_controller_dt.inited = 0;
	high_pos_pid.last_err = 0;
	high_pos_pid.pre_last_err = 0;
	high_pos_pid.last_expect = 0;
	high_pos_pid.control_output = 0;
}

/**********************************************************************************************************
*函 数 名: high_vel_pid_integrate_reset
*功能说明: 高度速度环pid积分清零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void high_vel_pid_integrate_reset()
{
	high_vel_pid.integrate = 0;
	high_vel_pid.pid_controller_dt.inited = 0;
	high_vel_pid.last_err = 0;
	high_vel_pid.pre_last_err = 0;
	high_vel_pid.last_expect = 0;
	high_vel_pid.control_output = 0;
}

/**********************************************************************************************************
*函 数 名: high_control
*功能说明: 高度环pid控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void high_control()
{
    //高度位置控制计数器
    static uint16_t high_pos_control_cnt;
    
    //竖直高度控制周期2*5=10ms
    if (high_pos_control_cnt >= 2) {
        //高度反馈
        high_pos_pid.feedback = high_pos;
        //高度位置pid控制
        pid_control(&high_pos_pid);
        
        high_pos_control_cnt = 0;
    }
    high_pos_control_cnt++;
    
    //高度速度pid控制
    high_vel_pid.expect = high_pos_pid.control_output;
    high_vel_pid.feedback = high_vel;
    pid_control(&high_vel_pid);
}
