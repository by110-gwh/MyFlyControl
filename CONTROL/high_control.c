#include "high_control.h"
#include "pid.h"
#include "navigation.h"

pid_paramer_t high_pos_pid_para = {
    .err_max = 0,
    .integrate_separation_err = 0,
    .integrate_max = 0,
    .kp = 4,
    .ki = 0,
    .kd = 0,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 50
};

pid_paramer_t high_speed_pid_para = {
    .err_max = 0,
    .integrate_separation_err = 0,
    .integrate_max = 0,
    .kp = 4,
    .ki = 0.2,
    .kd = 0,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 800
};

pid_data_t high_pos_pid_data;
pid_data_t high_speed_pid_data;

/**********************************************************************************************************
*函 数 名: high_pos_control_pid_set
*功能说明: 高度位置环pid调整函数
*形    参: 10p参数 10i参数 10d参数
*返 回 值: 无
**********************************************************************************************************/
void high_pos_control_pid_set(uint8_t p, uint8_t i, uint8_t d)
{
    high_pos_pid_para.kp = p / 10.;
    high_pos_pid_para.ki = i / 10.;
    high_pos_pid_para.kd = d / 10.;
}

/**********************************************************************************************************
*函 数 名: high_pos_control_init
*功能说明: 高度pid结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void high_control_init()
{
    high_pos_pid_data.last_expect = 0;
	high_pos_pid_data.expect = 0;
    high_pos_pid_data.feedback = 0;
	high_pos_pid_data.last_err = 0;
	high_pos_pid_data.pre_last_err = 0;
    high_pos_pid_data.integrate = 0;
    high_pos_pid_data.dis_err = 0;
	high_pos_pid_data.control_output = 0;
	high_pos_pid_data.pid_controller_dt.inited = 0;
    high_pos_pid_data.err_callback = NULL;
    high_pos_pid_data.pri_data = NULL;
    high_pos_pid_data.short_circuit_flag = 0;
    
    high_speed_pid_data.last_expect = 0;
	high_speed_pid_data.expect = 0;
    high_speed_pid_data.feedback = 0;
	high_speed_pid_data.last_err = 0;
	high_speed_pid_data.pre_last_err = 0;
    high_speed_pid_data.integrate = 0;
    high_speed_pid_data.dis_err = 0;
	high_speed_pid_data.control_output = 0;
	high_speed_pid_data.pid_controller_dt.inited = 0;
    high_speed_pid_data.err_callback = NULL;
    high_speed_pid_data.pri_data = NULL;
    high_speed_pid_data.short_circuit_flag = 0;
}

/**********************************************************************************************************
*函 数 名: high_pos_pid_integrate_reset
*功能说明: 高度位置环pid积分清零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void high_pos_pid_integrate_reset()
{
	high_pos_pid_data.integrate = 0;
	high_pos_pid_data.pid_controller_dt.inited = 0;
	high_pos_pid_data.last_err = 0;
	high_pos_pid_data.pre_last_err = 0;
	high_pos_pid_data.last_expect = 0;
	high_pos_pid_data.control_output = 0;
}

/**********************************************************************************************************
*函 数 名: high_speed_pid_integrate_reset
*功能说明: 高度速度环pid积分清零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void high_speed_pid_integrate_reset()
{
	high_speed_pid_data.integrate = 0;
	high_speed_pid_data.pid_controller_dt.inited = 0;
	high_speed_pid_data.last_err = 0;
	high_speed_pid_data.pre_last_err = 0;
	high_speed_pid_data.last_expect = 0;
	high_speed_pid_data.control_output = 0;
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
        high_pos_pid_data.feedback = pos_z;
        //高度位置pid控制
        pid_control(&high_pos_pid_data, &high_pos_pid_para);
        
        high_pos_control_cnt = 0;
    }
    high_pos_control_cnt++;
    
    //高度速度pid控制
    high_speed_pid_data.expect = high_pos_pid_data.control_output;
    high_speed_pid_data.feedback = speed_z;
    pid_control(&high_speed_pid_data, &high_speed_pid_para);
}
