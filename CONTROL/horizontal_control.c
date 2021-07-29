#include "horizontal_control.h"
#include "pid.h"
#include "navigation.h"
#include "ahrs_aux.h"
#include "angle_control.h"

//重力加速度
#define GRAVITY_MSS 9.80665f

#define PI 3.1415926f
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)

pid_paramer_t horizontal_pos_x_pid_para = {
    .err_max = 0,
    .integrate_separation_err = 0,
    .integrate_max = 0,
    .kp = 1.3,
    .ki = 0,
    .kd = 0,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 200
};

pid_paramer_t horizontal_pos_y_pid_para = {
    .err_max = 0,
    .integrate_separation_err = 0,
    .integrate_max = 0,
    .kp = 1.3,
    .ki = 0,
    .kd = 0,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 200
};

pid_paramer_t horizontal_speed_x_pid_para = {
    .err_max = 0,
    .integrate_separation_err = 0,
    .integrate_max = 0,
    .kp = 2.6,
    .ki = 0,
    .kd = 0.1,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 500
};

pid_paramer_t horizontal_speed_y_pid_para = {
    .err_max = 0,
    .integrate_separation_err = 0,
    .integrate_max = 0,
    .kp = 2.6,
    .ki = 0,
    .kd = 0.1,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 500
};

pid_data_t horizontal_pos_x_pid_data;
pid_data_t horizontal_speed_x_pid_data;
pid_data_t horizontal_pos_y_pid_data;
pid_data_t horizontal_speed_y_pid_data;

/**********************************************************************************************************
*函 数 名: horizontal_pos_control_pid_set
*功能说明: 水平位置环x方向pid调整函数
*形    参: 10p参数 10i参数 10d参数
*返 回 值: 无
**********************************************************************************************************/
void horizontal_pos_x_control_pid_set(uint8_t p, uint8_t i, uint8_t d)
{
    horizontal_pos_x_pid_para.kp = p / 10.;
    horizontal_pos_x_pid_para.ki = i / 10.;
    horizontal_pos_x_pid_para.kd = d / 10.;
}

/**********************************************************************************************************
*函 数 名: horizontal_pos_control_pid_set
*功能说明: 水平位置环x方向pid调整函数
*形    参: 10p参数 10i参数 10d参数
*返 回 值: 无
**********************************************************************************************************/
void horizontal_speed_x_control_pid_set(uint8_t p, uint8_t i, uint8_t d)
{
    horizontal_speed_x_pid_para.kp = p / 10.;
    horizontal_speed_x_pid_para.ki = i / 10.;
    horizontal_speed_x_pid_para.kd = d / 10.;
}

/**********************************************************************************************************
*函 数 名: horizontal_pos_control_init
*功能说明: 水平pid结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void horizontal_control_init()
{
    horizontal_pos_x_pid_data.last_expect = 0;
	horizontal_pos_x_pid_data.expect = 0;
    horizontal_pos_x_pid_data.feedback = 0;
	horizontal_pos_x_pid_data.last_err = 0;
	horizontal_pos_x_pid_data.pre_last_err = 0;
    horizontal_pos_x_pid_data.integrate = 0;
    horizontal_pos_x_pid_data.dis_err = 0;
	horizontal_pos_x_pid_data.control_output = 0;
	horizontal_pos_x_pid_data.pid_controller_dt.inited = 0;
    horizontal_pos_x_pid_data.err_callback = NULL;
    horizontal_pos_x_pid_data.pri_data = NULL;
    horizontal_pos_x_pid_data.short_circuit_flag = 1;
    
    horizontal_speed_x_pid_data.last_expect = 0;
	horizontal_speed_x_pid_data.expect = 0;
    horizontal_speed_x_pid_data.feedback = 0;
	horizontal_speed_x_pid_data.last_err = 0;
	horizontal_speed_x_pid_data.pre_last_err = 0;
    horizontal_speed_x_pid_data.integrate = 0;
    horizontal_speed_x_pid_data.dis_err = 0;
	horizontal_speed_x_pid_data.control_output = 0;
	horizontal_speed_x_pid_data.pid_controller_dt.inited = 0;
    horizontal_speed_x_pid_data.err_callback = NULL;
    horizontal_speed_x_pid_data.pri_data = NULL;
    horizontal_speed_x_pid_data.short_circuit_flag = 0;
    
    horizontal_pos_y_pid_data.last_expect = 0;
	horizontal_pos_y_pid_data.expect = 0;
    horizontal_pos_y_pid_data.feedback = 0;
	horizontal_pos_y_pid_data.last_err = 0;
	horizontal_pos_y_pid_data.pre_last_err = 0;
    horizontal_pos_y_pid_data.integrate = 0;
    horizontal_pos_y_pid_data.dis_err = 0;
	horizontal_pos_y_pid_data.control_output = 0;
	horizontal_pos_y_pid_data.pid_controller_dt.inited = 0;
    horizontal_pos_y_pid_data.err_callback = NULL;
    horizontal_pos_y_pid_data.pri_data = NULL;
    horizontal_pos_y_pid_data.short_circuit_flag = 1;
    
    horizontal_speed_y_pid_data.last_expect = 0;
	horizontal_speed_y_pid_data.expect = 0;
    horizontal_speed_y_pid_data.feedback = 0;
	horizontal_speed_y_pid_data.last_err = 0;
	horizontal_speed_y_pid_data.pre_last_err = 0;
    horizontal_speed_y_pid_data.integrate = 0;
    horizontal_speed_y_pid_data.dis_err = 0;
	horizontal_speed_y_pid_data.control_output = 0;
	horizontal_speed_y_pid_data.pid_controller_dt.inited = 0;
    horizontal_speed_y_pid_data.err_callback = NULL;
    horizontal_speed_y_pid_data.pri_data = NULL;
    horizontal_speed_y_pid_data.short_circuit_flag = 0;
}

/**********************************************************************************************************
*函 数 名: horizontal_pos_x_pid_integrate_reset
*功能说明: 水平位置环x方向pid积分清零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void horizontal_pos_x_pid_integrate_reset()
{
	horizontal_pos_x_pid_data.integrate = 0;
	horizontal_pos_x_pid_data.pid_controller_dt.inited = 0;
	horizontal_pos_x_pid_data.last_err = 0;
	horizontal_pos_x_pid_data.pre_last_err = 0;
	horizontal_pos_x_pid_data.last_expect = 0;
	horizontal_pos_x_pid_data.control_output = 0;
}

/**********************************************************************************************************
*函 数 名: horizontal_pos_y_pid_integrate_reset
*功能说明: 水平位置环y方向pid积分清零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void horizontal_pos_y_pid_integrate_reset()
{
	horizontal_pos_y_pid_data.integrate = 0;
	horizontal_pos_y_pid_data.pid_controller_dt.inited = 0;
	horizontal_pos_y_pid_data.last_err = 0;
	horizontal_pos_y_pid_data.pre_last_err = 0;
	horizontal_pos_y_pid_data.last_expect = 0;
	horizontal_pos_y_pid_data.control_output = 0;
}

/**********************************************************************************************************
*函 数 名: horizontal_speed_x_pid_integrate_reset
*功能说明: 水平速度环x方向pid积分清零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void horizontal_speed_x_pid_integrate_reset()
{
	horizontal_speed_x_pid_data.integrate = 0;
	horizontal_speed_x_pid_data.pid_controller_dt.inited = 0;
	horizontal_speed_x_pid_data.last_err = 0;
	horizontal_speed_x_pid_data.pre_last_err = 0;
	horizontal_speed_x_pid_data.last_expect = 0;
	horizontal_speed_x_pid_data.control_output = 0;
}

/**********************************************************************************************************
*函 数 名: horizontal_speed_y_pid_integrate_reset
*功能说明: 水平速度环y方向pid积分清零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void horizontal_speed_y_pid_integrate_reset()
{
	horizontal_speed_y_pid_data.integrate = 0;
	horizontal_speed_y_pid_data.pid_controller_dt.inited = 0;
	horizontal_speed_y_pid_data.last_err = 0;
	horizontal_speed_y_pid_data.pre_last_err = 0;
	horizontal_speed_y_pid_data.last_expect = 0;
	horizontal_speed_y_pid_data.control_output = 0;
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
*函 数 名: fast_atan
*功能说明: 快速求反正切
*形    参: x
*返 回 值: atan(x)
**********************************************************************************************************/
static float fast_atan(float v)
{
    float v2 = v*v;
    return (v*(1.6867629106f+v2*0.4378497304f)/(1.6867633134f+v2));
}

/**********************************************************************************************************
*函 数 名: horizontal_speed_to_angles
*功能说明: 水平速度控制器输出给角度控制器
*形    参: 水平速度x输出 水平速度y输出 俯仰角指针 横滚角指针
*返 回 值: 无
**********************************************************************************************************/
static void horizontal_speed_to_angles(float horizontal_speed_x, float horizontal_speed_y, volatile float *pitch_angle, volatile float *roll_angle)
{
    float accel_right, accel_forward;
    float lean_angle_max = 30;
    accel_forward = horizontal_speed_y;//cm/s^2
    accel_right = horizontal_speed_x;//cm/s^2

    *pitch_angle = -constrain_float(fast_atan(accel_forward*Cos_Roll/(GRAVITY_MSS*100))*RAD2DEG,-lean_angle_max,lean_angle_max);//pitch
    *roll_angle = constrain_float(fast_atan(accel_right/(GRAVITY_MSS*100))*RAD2DEG,-lean_angle_max,lean_angle_max);//roll
}

/**********************************************************************************************************
*函 数 名: horizontal_control
*功能说明: 高度环pid控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void horizontal_control()
{
    //水平位置控制计数器
    static uint16_t horizontal_pos_control_cnt;
    //水平速度控制计数器
    static uint16_t horizontal_speed_control_cnt;
    
    //水平位置控制周期10*5=50ms
    if (horizontal_pos_control_cnt >= 10) {
        //水平位置反馈
        horizontal_pos_x_pid_data.feedback = pos_x;
        horizontal_pos_y_pid_data.feedback = pos_y;
        //水平位置pid控制
        pid_control(&horizontal_pos_x_pid_data, &horizontal_pos_x_pid_para);
        pid_control(&horizontal_pos_y_pid_data, &horizontal_pos_y_pid_para);
        
        horizontal_pos_control_cnt = 0;
    }
    horizontal_pos_control_cnt++;
    
    
    //水平速度控制周期4*5=50ms
    if (horizontal_speed_control_cnt >= 4) {
        //水平速度期待
        horizontal_speed_x_pid_data.expect = horizontal_pos_x_pid_data.control_output;
        horizontal_speed_y_pid_data.expect = horizontal_pos_y_pid_data.control_output;
        //水平速度反馈
        horizontal_speed_x_pid_data.feedback = speed_x;
        horizontal_speed_y_pid_data.feedback = speed_y;
        //水平速度pid控制
        pid_control(&horizontal_speed_x_pid_data, &horizontal_speed_x_pid_para);
        pid_control(&horizontal_speed_y_pid_data, &horizontal_speed_y_pid_para);
        
        horizontal_speed_control_cnt = 0;
    }
    horizontal_speed_control_cnt++;
    
    horizontal_speed_to_angles(horizontal_speed_x_pid_data.control_output, horizontal_speed_y_pid_data.control_output, &pitch_angle_pid_data.expect, &roll_angle_pid_data.expect);
}
