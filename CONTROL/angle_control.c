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
*函 数 名: angle_control_init
*功能说明: 角度环pid结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void angle_control_init()
{
    //俯仰角pid参数初始化
    pitch_angle_pid.expect = 0;
    pitch_angle_pid.feedback = 0;

    pitch_angle_pid.err = 0;
    pitch_angle_pid.last_err = 0;
    pitch_angle_pid.err_max = 30;

    pitch_angle_pid.integrate_separation_err = 0;
    pitch_angle_pid.integrate = 0;
    pitch_angle_pid.integrate_max = 80;

    pitch_angle_pid.dis_err = 0;

    pitch_angle_pid.kp = 4;
    pitch_angle_pid.ki = 0;
    pitch_angle_pid.kd = 0;

    pitch_angle_pid.control_output = 0;
    pitch_angle_pid.control_output_limit = 450;

    pitch_angle_pid.short_circuit_flag = 0;
    pitch_angle_pid.pri_data = NULL;

    //横滚角pid参数初始化
    roll_angle_pid.expect = 0;
    roll_angle_pid.feedback = 0;

    roll_angle_pid.err = 0;
    roll_angle_pid.last_err = 0;
    roll_angle_pid.err_max = 30;

    roll_angle_pid.integrate_separation_err = 0;
    roll_angle_pid.integrate = 0;
    roll_angle_pid.integrate_max = 80;

    roll_angle_pid.dis_err = 0;

    roll_angle_pid.kp = 4;
    roll_angle_pid.ki = 0;
    roll_angle_pid.kd = 0;

    roll_angle_pid.control_output = 0;
    roll_angle_pid.control_output_limit = 450;

    pitch_angle_pid.short_circuit_flag = 0;
    roll_angle_pid.pri_data = NULL;

    //偏航pid参数初始化
    yaw_angle_pid.expect = 0;
    yaw_angle_pid.feedback = 0;

    yaw_angle_pid.err = 0;
    yaw_angle_pid.last_err = 0;
    yaw_angle_pid.err_max = 45;

    yaw_angle_pid.integrate_separation_err = 0;
    yaw_angle_pid.integrate = 0;
    yaw_angle_pid.integrate_max = 150;

    yaw_angle_pid.dis_err = 0;

    yaw_angle_pid.kp = 4;
    yaw_angle_pid.ki = 0;
    yaw_angle_pid.kd = 0;

    yaw_angle_pid.control_output = 0;
    yaw_angle_pid.control_output_limit = 450;

    yaw_angle_pid.short_circuit_flag = 0;
    yaw_angle_pid.pri_data = yaw_err_correct;
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
