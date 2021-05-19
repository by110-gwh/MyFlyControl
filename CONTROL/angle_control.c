#include "angle_control.h"
#include "pid.h"
#include "ahrs_aux.h"

pid_controler_t pitch_angle_pid;
pid_controler_t roll_angle_pid;
pid_controler_t yaw_angle_pid;

/**********************************************************************************************************
*函 数 名: yaw_err_correct
*功能说明: 偏航角误差角度校正
*形    参: pid控制器结构体
*返 回 值: 无
**********************************************************************************************************/
static void yaw_err_correct(pid_controler_t *controler)
{
    if(controler->err < -180)
        controler->err = controler->err + 360;
    if(controler->err > 180)
        controler->err = controler->err - 360;
}

/**********************************************************************************************************
*函 数 名: angle_control_pid_set
*功能说明: 角度环pid调整函数
*形    参: 10p参数 10i参数 10d参数
*返 回 值: 无
**********************************************************************************************************/
void angle_control_pid_set(uint8_t p, uint8_t i, uint8_t d)
{
	pitch_angle_pid.kp = p / 10.;
	pitch_angle_pid.ki = i / 10.;
	pitch_angle_pid.kd = d / 10.;
	angle_pid_integrate_reset();
}

/**********************************************************************************************************
*函 数 名: angle_control_init
*功能说明: 角度环pid结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void angle_control_init()
{
    //俯仰角pid参数初始化
    pitch_angle_pid.last_expect = 0;
    pitch_angle_pid.expect = 0;
    pitch_angle_pid.feedback = 0;

    pitch_angle_pid.err = 0;
    pitch_angle_pid.last_err = 0;
    pitch_angle_pid.err_max = 30;

    pitch_angle_pid.integrate_separation_err = 0;
    pitch_angle_pid.integrate = 0;
    pitch_angle_pid.integrate_max = 80;

    pitch_angle_pid.dis_err = 0;

    pitch_angle_pid.kp = 3;
    pitch_angle_pid.ki = 0;
    pitch_angle_pid.kd = 0;
    
    pitch_angle_pid.feedforward_kp = 0;
    pitch_angle_pid.feedforward_kd = 0;

    pitch_angle_pid.control_output = 0;
    pitch_angle_pid.control_output_limit = 450;

    pitch_angle_pid.short_circuit_flag = 0;
    pitch_angle_pid.err_callback = NULL;
    pitch_angle_pid.pri_data = NULL;

    //横滚角pid参数初始化
    roll_angle_pid.last_expect = 0;
    roll_angle_pid.expect = 0;
    roll_angle_pid.feedback = 0;

    roll_angle_pid.err = 0;
    roll_angle_pid.last_err = 0;
    roll_angle_pid.err_max = 30;

    roll_angle_pid.integrate_separation_err = 0;
    roll_angle_pid.integrate = 0;
    roll_angle_pid.integrate_max = 80;

    roll_angle_pid.dis_err = 0;

    roll_angle_pid.kp = 3;
    roll_angle_pid.ki = 0;
    roll_angle_pid.kd = 0;

    roll_angle_pid.feedforward_kp = 0;
    roll_angle_pid.feedforward_kd = 0;

    roll_angle_pid.control_output = 0;
    roll_angle_pid.control_output_limit = 450;

    roll_angle_pid.short_circuit_flag = 0;
    roll_angle_pid.err_callback = NULL;
    roll_angle_pid.pri_data = NULL;

    //偏航pid参数初始化
    yaw_angle_pid.last_expect = 0;
    yaw_angle_pid.expect = 0;
    yaw_angle_pid.feedback = 0;

    yaw_angle_pid.err = 0;
    yaw_angle_pid.last_err = 0;
    yaw_angle_pid.err_max = 45;

    yaw_angle_pid.integrate_separation_err = 0;
    yaw_angle_pid.integrate = 0;
    yaw_angle_pid.integrate_max = 150;

    yaw_angle_pid.dis_err = 0;

    yaw_angle_pid.kp = 3;
    yaw_angle_pid.ki = 0;
    yaw_angle_pid.kd = 0;

    yaw_angle_pid.feedforward_kp = 0;
    yaw_angle_pid.feedforward_kd = 0;

    yaw_angle_pid.control_output = 0;
    yaw_angle_pid.control_output_limit = 20;

    yaw_angle_pid.short_circuit_flag = 0;
    yaw_angle_pid.err_callback = yaw_err_correct;
    yaw_angle_pid.pri_data = NULL;
}

/**********************************************************************************************************
*函 数 名: angle_pid_integrate_reset
*功能说明: 角度环pid积分清零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void angle_pid_integrate_reset()
{
	yaw_angle_pid.integrate = 0;
	yaw_angle_pid.pid_controller_dt.inited = 0;
	yaw_angle_pid.last_err = 0;
	yaw_angle_pid.pre_last_err = 0;
	yaw_angle_pid.last_expect = 0;
	yaw_angle_pid.control_output = 0;
	
	pitch_angle_pid.integrate = 0;
	pitch_angle_pid.pid_controller_dt.inited = 0;
	pitch_angle_pid.last_err = 0;
	pitch_angle_pid.pre_last_err = 0;
	pitch_angle_pid.last_expect = 0;
	pitch_angle_pid.control_output = 0;
	
	roll_angle_pid.integrate = 0;
	roll_angle_pid.pid_controller_dt.inited = 0;
	roll_angle_pid.last_err = 0;
	roll_angle_pid.pre_last_err = 0;
	roll_angle_pid.last_expect = 0;
	roll_angle_pid.control_output = 0;
}

/**********************************************************************************************************
*函 数 名: angle_control
*功能说明: 角度环pid控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void angle_control()
{
    pitch_angle_pid.feedback = Pitch;
    pid_control(&pitch_angle_pid);

    roll_angle_pid.feedback = Roll;
    pid_control(&roll_angle_pid);

    yaw_angle_pid.feedback = Yaw;
    pid_control(&yaw_angle_pid);
}
