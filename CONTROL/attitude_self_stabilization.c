#include "attitude_self_stabilization.h"
#include "remote_control.h"
#include "motor_output.h"
#include "angle_control.h"
#include "ahrs_aux.h"

/**********************************************************************************************************
*函 数 名: constrain_float
*功能说明: 限制最大最小值
*形    参: 要限制的值 最小值 最大值
*返 回 值: 输出值
**********************************************************************************************************/
float constrain_float(float amt, float low, float high)
{
	return ((amt) < (low) ? (low) : ((amt) > (high)? (high) : (amt)));
}

/**********************************************************************************************************
*函 数 名: throttle_angle_compensate
*功能说明: 油门倾角补偿
*形    参: 原有门量
*返 回 值: 补偿后的油门量
**********************************************************************************************************/
uint16_t throttle_angle_compensate(uint16_t throttle)
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
*函 数 名: attitude_self_stabilization_control
*功能说明: 姿态自稳控制器
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void attitude_self_stabilization_control()
{
	throttle_motor_output = throttle_angle_compensate(Throttle_Control + 1000);
	pitch_angle_pid.expect = Pitch_Control;
	roll_angle_pid.expect = Roll_Control;

	//偏航杆置于中位
	if (Yaw_Control == 0) {
		//回中时赋角度期望值
		if (yaw_angle_pid.short_circuit_flag == 1) {
			yaw_angle_pid.expect = Yaw;
			//使能偏航pid计算
			yaw_angle_pid.short_circuit_flag = 0;
		}
	//波动偏航方向杆后，只进行内环角速度控制
	} else {
		//关闭偏航pid计算
		yaw_angle_pid.short_circuit_flag = 1;
		//偏航角期望给0,不进行角度控制
		yaw_angle_pid.expect = Yaw_Control;
	}
}
