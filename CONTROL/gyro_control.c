#include "gyro_control.h"
#include "pid.h"
#include "imu.h"
#include "angle_control.h"
#include "Filter.h"
#include "math.h"

#define ABS(X)  (((X) > 0)? (X) : -(X))

pid_controler_t pitch_gyro_pid;
pid_controler_t roll_gyro_pid;
pid_controler_t yaw_gyro_pid;

typedef struct {
    float last_dis_err;
    float pre_last_dis_err;
    Butter_BufferData buffer;
    float raw_kd;
} pitch_roll_err_correct_t;

//额外的pid参数
pitch_roll_err_correct_t pitch_pri_dat;
pitch_roll_err_correct_t roll_pri_dat;
pitch_roll_err_correct_t yaw_pri_dat;

Butter_Parameter gyro_filter_parameter_30Hz;
Butter_Parameter gyro_filter_parameter_20Hz;

/**********************************************************************************************************
*函 数 名: pitch_roll_err_correct
*功能说明: 额外的俯仰和偏航角速度pid计算
*形    参: pid控制器结构体
*返 回 值: 无
**********************************************************************************************************/
static void pitch_roll_err_correct(pid_controler_t *controler)
{
    //用于防跳变滤波
    float tempa, tempb, tempc, max, min;
    float dis_err_filter;

    //间隔了一次采样的微分
    controler->dis_err = controler->err - controler->pre_last_err;

    //均值滤波，保证得到数据不跳变，避免期望阶跃时，微分输出异常
    tempa = ((pitch_roll_err_correct_t *)controler->pri_data)->pre_last_dis_err;
    tempb = ((pitch_roll_err_correct_t *)controler->pri_data)->last_dis_err;
    tempc = controler->dis_err;
    max = tempa > tempb ? tempa : tempb;
    max = max > tempc ? max : tempc;
    min = tempa < tempb ? tempa : tempb;
    min = min < tempc ? min : tempc;
    if (tempa > min && tempa < max)
        controler->dis_err = tempa;
    if (tempb > min && tempb < max)
        controler->dis_err = tempb;
    if (tempc > min && tempc < max)
        controler->dis_err = tempc;
    ((pitch_roll_err_correct_t *)controler->pri_data)->pre_last_dis_err =
        ((pitch_roll_err_correct_t *)controler->pri_data)->last_dis_err;
    ((pitch_roll_err_correct_t *)controler->pri_data)->last_dis_err = controler->dis_err;

    //巴特沃斯低通后得到的微分项,30hz
    dis_err_filter = Butterworth_Filter(controler->dis_err,
        &((pitch_roll_err_correct_t *)controler->pri_data)->buffer,
        &gyro_filter_parameter_30Hz);

	if (dis_err_filter >= 500)
		dis_err_filter = 500;
	if (dis_err_filter <= -500)
		dis_err_filter = -500;
    //自适应微分参数
    controler->kd = ((pitch_roll_err_correct_t *)controler->pri_data)->raw_kd
        * (1 + ABS(dis_err_filter) / 500.0f);    
}

/**********************************************************************************************************
*函 数 名: yaw_err_correct
*功能说明: 额外的偏航角速度pid计算
*形    参: pid控制器结构体
*返 回 值: 无
**********************************************************************************************************/
static void yaw_err_correct(pid_controler_t *controler)
{
    //用于防跳变滤波
    float tempa, tempb, tempc, max, min;

    //均值滤波，保证得到数据不跳变，避免期望阶跃时，微分输出异常
    tempa = ((pitch_roll_err_correct_t *)controler->pri_data)->pre_last_dis_err;
    tempb = ((pitch_roll_err_correct_t *)controler->pri_data)->last_dis_err;
    tempc = controler->dis_err;
    max = tempa > tempb ? tempa : tempb;
    max = max > tempc ? max : tempc;
    min = tempa < tempb ? tempa : tempb;
    min = min < tempc ? min : tempc;
    if (tempa > min && tempa < max)
        controler->dis_err = tempa;
    if (tempb > min && tempb < max)
        controler->dis_err = tempb;
    if (tempc > min && tempc < max)
        controler->dis_err = tempc;
    ((pitch_roll_err_correct_t *)controler->pri_data)->pre_last_dis_err =
        ((pitch_roll_err_correct_t *)controler->pri_data)->last_dis_err;
    ((pitch_roll_err_correct_t *)controler->pri_data)->last_dis_err = controler->dis_err;
    //巴特沃斯低通后得到的微分项,30hz
    controler->dis_err = Butterworth_Filter(controler->dis_err,
        &((pitch_roll_err_correct_t *)controler->pri_data)->buffer,
        &gyro_filter_parameter_20Hz);
}


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
    pitch_gyro_pid.err_callback = pitch_roll_err_correct;
    pitch_gyro_pid.pri_data = &pitch_pri_dat;

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
    roll_gyro_pid.err_callback = pitch_roll_err_correct;
    roll_gyro_pid.pri_data = &roll_pri_dat;

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
    yaw_gyro_pid.err_callback = yaw_err_correct;
    yaw_gyro_pid.pri_data = &yaw_pri_dat;

    pitch_pri_dat.raw_kd = 2;
    roll_pri_dat.raw_kd = 2;
    yaw_pri_dat.raw_kd = 0;

    Set_Cutoff_Frequency(Sampling_Freq, 30, &gyro_filter_parameter_30Hz);
    Set_Cutoff_Frequency(Sampling_Freq, 20, &gyro_filter_parameter_20Hz);
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
