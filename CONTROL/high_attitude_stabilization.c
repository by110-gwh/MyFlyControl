#include "high_attitude_stabilization.h"
#include "remote_control.h"
#include "motor_output.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "high_control.h"

/**********************************************************************************************************
*�� �� ��: high_attitude_stabilization_control
*����˵��: ���߿�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void high_attitude_stabilization_control()
{
    
    high_pos_pid.expect = (Throttle_Control - 500) / (float)(1000 - 500) * 200 + 10;
    //ʹ�ܸ߶�λ��pid����
    high_pos_pid.short_circuit_flag = 0;

    //�Ƕȿ�������ң����
	pitch_angle_pid.expect = Pitch_Control;
	roll_angle_pid.expect = Roll_Control;

	//ƫ����������λ
	if (Yaw_Control == 0) {
		//����ʱ���Ƕ�����ֵ
		if (yaw_angle_pid.short_circuit_flag == 1) {
			yaw_angle_pid.expect = Yaw;
			//ʹ��ƫ��pid����
			yaw_angle_pid.short_circuit_flag = 0;
		}
	//����ƫ������˺�ֻ�����ڻ����ٶȿ���
	} else {
		//�ر�ƫ��pid����
		yaw_angle_pid.short_circuit_flag = 1;
		//ƫ����������0,�����нǶȿ���
		yaw_angle_pid.expect = Yaw_Control;
	}
}
