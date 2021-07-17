#include "horizontal_attitude_stabilization.h"
#include "remote_control.h"
#include "motor_output.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "high_control.h"
#include "horizontal_control.h"

#define HORIZONTAL_SPEED_MAX 250

/**********************************************************************************************************
*函 数 名: horizontal_attitude_stabilization_control
*功能说明: 定点控制器
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void horizontal_attitude_stabilization_control()
{
    high_pos_pid_data.expect = (Throttle_Control - 500) / (float)(1000 - 500) * 200 + 10;
    
    if (Pitch_Control == 0) {
		//回中时赋y位置期望值
		if (horizontal_pos_y_pid_data.short_circuit_flag == 1) {
			horizontal_pos_y_pid_data.expect = pos_y;
			//使能偏航pid计算
			horizontal_pos_y_pid_data.short_circuit_flag = 0;
		}
	//波动偏航方向杆后，只进行内环角速度控制
	} else {
		//关闭偏航pid计算
		horizontal_pos_y_pid_data.short_circuit_flag = 1;
		//偏航角期望给0,不进行角度控制
		horizontal_pos_y_pid_data.expect = Pitch_Control * HORIZONTAL_SPEED_MAX / Pit_Rol_Max;
	}
    
    if (Roll_Control == 0) {
		//回中时赋y位置期望值
		if (horizontal_pos_x_pid_data.short_circuit_flag == 1) {
			horizontal_pos_x_pid_data.expect = pos_x;
			//使能偏航pid计算
			horizontal_pos_x_pid_data.short_circuit_flag = 0;
		}
	//波动偏航方向杆后，只进行内环角速度控制
	} else {
		//关闭偏航pid计算
		horizontal_pos_x_pid_data.short_circuit_flag = 1;
		//偏航角期望给0,不进行角度控制
		horizontal_pos_x_pid_data.expect = Roll_Control * HORIZONTAL_SPEED_MAX / Pit_Rol_Max;
	}

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
