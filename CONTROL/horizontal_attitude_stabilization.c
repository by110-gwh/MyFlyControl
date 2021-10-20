#include "horizontal_attitude_stabilization.h"
#include "remote_control.h"
#include "motor_output.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "high_control.h"
#include "horizontal_control.h"
#include "controller.h"

#define HORIZONTAL_SPEED_MAX 250

/**********************************************************************************************************
*�� �� ��: horizontal_attitude_stabilization_control
*����˵��: ���������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void horizontal_attitude_stabilization_control()
{
    high_pos_pid_data.expect = (Throttle_Control - HOLD_THROTTLE) / (float)(1000 - HOLD_THROTTLE) * 200 + 10;
    
    if (Pitch_Control == 0) {
		//����ʱ��yλ������ֵ
		if (horizontal_pos_y_pid_data.short_circuit_flag == 1) {
            //����û���ٶ�
            if (speed_y < 40 && speed_y > -40) {
                horizontal_pos_y_pid_data.expect = pos_y;
                //ʹ��ˮƽy����pid����
                horizontal_pos_y_pid_data.short_circuit_flag = 0;
            } else {
                //ɲ������
                horizontal_pos_y_pid_data.expect = 0;
            }
		}
	//����ˮƽy����˺�ֻ�����ڻ����ٶȿ���
	} else {
		//�ر�ˮƽy����pid����
		horizontal_pos_y_pid_data.short_circuit_flag = 1;
		//ˮƽy����������0,������ˮƽy�������
		horizontal_pos_y_pid_data.expect = -Pitch_Control * HORIZONTAL_SPEED_MAX / Pit_Rol_Max;
	}
    
    if (Roll_Control == 0) {
		//����ʱ��yλ������ֵ
		if (horizontal_pos_x_pid_data.short_circuit_flag == 1) {
            //����û���ٶ�
            if (speed_x < 40 && speed_x > -40) {
                horizontal_pos_x_pid_data.expect = pos_x;
                //ʹ��ˮƽx����pid����
                horizontal_pos_x_pid_data.short_circuit_flag = 0;
            } else {
                //ɲ������
                horizontal_pos_x_pid_data.expect = 0;
            }
		}
	//����ˮƽx����˺�ֻ�����ڻ����ٶȿ���
	} else {
		//�ر�ˮƽx����pid����
		horizontal_pos_x_pid_data.short_circuit_flag = 1;
		//ˮƽx����������0,������ˮƽx�������
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
