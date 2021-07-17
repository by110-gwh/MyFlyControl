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
*�� �� ��: horizontal_attitude_stabilization_control
*����˵��: ���������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void horizontal_attitude_stabilization_control()
{
    high_pos_pid_data.expect = (Throttle_Control - 500) / (float)(1000 - 500) * 200 + 10;
    
    if (Pitch_Control == 0) {
		//����ʱ��yλ������ֵ
		if (horizontal_pos_y_pid_data.short_circuit_flag == 1) {
			horizontal_pos_y_pid_data.expect = pos_y;
			//ʹ��ƫ��pid����
			horizontal_pos_y_pid_data.short_circuit_flag = 0;
		}
	//����ƫ������˺�ֻ�����ڻ����ٶȿ���
	} else {
		//�ر�ƫ��pid����
		horizontal_pos_y_pid_data.short_circuit_flag = 1;
		//ƫ����������0,�����нǶȿ���
		horizontal_pos_y_pid_data.expect = Pitch_Control * HORIZONTAL_SPEED_MAX / Pit_Rol_Max;
	}
    
    if (Roll_Control == 0) {
		//����ʱ��yλ������ֵ
		if (horizontal_pos_x_pid_data.short_circuit_flag == 1) {
			horizontal_pos_x_pid_data.expect = pos_x;
			//ʹ��ƫ��pid����
			horizontal_pos_x_pid_data.short_circuit_flag = 0;
		}
	//����ƫ������˺�ֻ�����ڻ����ٶȿ���
	} else {
		//�ر�ƫ��pid����
		horizontal_pos_x_pid_data.short_circuit_flag = 1;
		//ƫ����������0,�����нǶȿ���
		horizontal_pos_x_pid_data.expect = Roll_Control * HORIZONTAL_SPEED_MAX / Pit_Rol_Max;
	}

	//ƫ����������λ
	if (Yaw_Control == 0) {
		//����ʱ���Ƕ�����ֵ
		if (yaw_angle_pid_data.short_circuit_flag == 1) {
			yaw_angle_pid_data.expect = Yaw;
			//ʹ��ƫ��pid����
			yaw_angle_pid_data.short_circuit_flag = 0;
		}
	//����ƫ������˺�ֻ�����ڻ����ٶȿ���
	} else {
		//�ر�ƫ��pid����
		yaw_angle_pid_data.short_circuit_flag = 1;
		//ƫ����������0,�����нǶȿ���
		yaw_angle_pid_data.expect = Yaw_Control;
	}
}
