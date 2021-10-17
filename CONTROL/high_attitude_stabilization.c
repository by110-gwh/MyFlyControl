#include "high_attitude_stabilization.h"
#include "remote_control.h"
#include "motor_output.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "high_control.h"

/**********************************************************************************************************
*函 数 名: high_attitude_stabilization_control
*功能说明: 定高控制器
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void high_attitude_stabilization_control()
{
    high_pos_pid_data.expect = (Throttle_Control - 450) / (float)(1000 - 450) * 200 + 10;

    //角度控制来自遥控器
    pitch_angle_pid_data.expect = Pitch_Control;
	roll_angle_pid_data.expect = Roll_Control;

	//偏航杆置于中位
	if (Yaw_Control == 0) {
		//回中时赋角度期望值
		if (yaw_angle_pid_data.short_circuit_flag == 1) {
			yaw_angle_pid_data.expect = Yaw;
			//使能偏航pid计算
			yaw_angle_pid_data.short_circuit_flag = 0;
		}
	//波动偏航方向杆后，只进行内环角速度控制
	} else {
		//关闭偏航pid计算
		yaw_angle_pid_data.short_circuit_flag = 1;
		//偏航角期望给0,不进行角度控制
		yaw_angle_pid_data.expect = Yaw_Control;
	}
}
