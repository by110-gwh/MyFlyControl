#include "attitude_self_stabilization.h"
#include "remote_control.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "high_control.h"
#include "ahrs_aux.h"

/**********************************************************************************************************
*函 数 名: attitude_self_stabilization_control
*功能说明: 姿态自稳控制器
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void attitude_self_stabilization_control()
{
    //油门直接输出，由于设置了pid短路标志，故直接输出
    high_pos_pid.expect = Throttle_Control;
    
    //角度控制来自遥控器
	pitch_angle_pid.expect = Pitch_Control;
	roll_angle_pid.expect = Roll_Control;
    
    //未起飞时，姿态控制器归零
	if (Throttle_Control == 1000) {
		angle_pid_integrate_reset();
		gyro_pid_integrate_reset();
		yaw_angle_pid.expect = Yaw;
	}

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
