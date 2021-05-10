#include "gyro_control.h"
#include "pid.h"
#include "imu.h"
#include "angle_control.h"
#include "Filter.h"
#include "math.h"
#include "string.h"

pid_controler_t pitch_gyro_pid;
pid_controler_t roll_gyro_pid;
pid_controler_t yaw_gyro_pid;

/**********************************************************************************************************
*函 数 名: gyro_control_init
*功能说明: 角速度度环pid结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void gyro_control_init()
{
    //俯仰角pid参数初始化
    pitch_gyro_pid.last_expect = 0;
    pitch_gyro_pid.expect = 0;
    pitch_gyro_pid.feedback = 0;

    pitch_gyro_pid.err = 0;
    pitch_gyro_pid.last_err = 0;
    pitch_gyro_pid.err_max = 500;

    pitch_gyro_pid.integrate_separation_err = 0;
    pitch_gyro_pid.integrate = 0;
    pitch_gyro_pid.integrate_max = 300;

    pitch_gyro_pid.dis_err = 0;

    pitch_gyro_pid.kp = 0.75;
    pitch_gyro_pid.ki = 2.5;
    pitch_gyro_pid.kd = 2;
    
    pitch_gyro_pid.feedforward_kp = 0.1;
    pitch_gyro_pid.feedforward_kd = 0;

    pitch_gyro_pid.control_output = 0;
    pitch_gyro_pid.control_output_limit = 500;

    pitch_gyro_pid.short_circuit_flag = 0;
    pitch_gyro_pid.err_callback = NULL;
    pitch_gyro_pid.pri_data = NULL;

    //横滚角pid参数初始化
    roll_gyro_pid.last_expect = 0;
    roll_gyro_pid.expect = 0;
    roll_gyro_pid.feedback = 0;

    roll_gyro_pid.err = 0;
    roll_gyro_pid.last_err = 0;
    roll_gyro_pid.err_max = 500;

    roll_gyro_pid.integrate_separation_err = 0;
    roll_gyro_pid.integrate = 0;
    roll_gyro_pid.integrate_max = 300;

    roll_gyro_pid.dis_err = 0;

    roll_gyro_pid.kp = 0.75;
    roll_gyro_pid.ki = 2.5;
    roll_gyro_pid.kd = 2;

    roll_gyro_pid.feedforward_kp = 0.1;
    roll_gyro_pid.feedforward_kd = 0;

    roll_gyro_pid.control_output = 0;
    roll_gyro_pid.control_output_limit = 500;

    roll_gyro_pid.short_circuit_flag = 0;
    roll_gyro_pid.err_callback = NULL;
    roll_gyro_pid.pri_data = NULL;

    //偏航pid参数初始化
    yaw_gyro_pid.last_expect = 0;
    yaw_gyro_pid.expect = 0;
    yaw_gyro_pid.feedback = 0;

    yaw_gyro_pid.err = 0;
    yaw_gyro_pid.last_err = 0;
    yaw_gyro_pid.err_max = 300;

    yaw_gyro_pid.integrate_separation_err = 0;
    yaw_gyro_pid.integrate = 0;
    yaw_gyro_pid.integrate_max = 200;

    yaw_gyro_pid.dis_err = 0;

    yaw_gyro_pid.kp = 1.2;
    yaw_gyro_pid.ki = 0.5;
    yaw_gyro_pid.kd = 0;

    yaw_gyro_pid.feedforward_kp = 0;
    yaw_gyro_pid.feedforward_kd = 0.05;

    yaw_gyro_pid.control_output = 0;
    yaw_gyro_pid.control_output_limit = 500;

    yaw_gyro_pid.short_circuit_flag = 0;
    yaw_gyro_pid.err_callback = NULL;
    yaw_gyro_pid.pri_data = NULL;
}

/**********************************************************************************************************
*函 数 名: gyro_pid_integrate_reset
*功能说明: 角速度环pid积分清零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void gyro_pid_integrate_reset()
{
	yaw_gyro_pid.integrate = 0;
	roll_gyro_pid.integrate = 0;
	pitch_gyro_pid.integrate = 0;
}


/**********************************************************************************************************
*函 数 名: gyro_control
*功能说明: 角速度度环pid控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void gyro_control()
{
    pitch_gyro_pid.feedback = gyroDataFilter.x * GYRO_CALIBRATION_COFF;
    pitch_gyro_pid.expect = pitch_angle_pid.control_output;
    pid_control(&pitch_gyro_pid);

    roll_gyro_pid.feedback = gyroDataFilter.y * GYRO_CALIBRATION_COFF;
    roll_gyro_pid.expect = roll_angle_pid.control_output;
    pid_control(&roll_gyro_pid);

    yaw_gyro_pid.feedback = gyroDataFilter.z * GYRO_CALIBRATION_COFF;
    yaw_gyro_pid.expect = yaw_angle_pid.control_output;
    pid_control(&yaw_gyro_pid);
}
