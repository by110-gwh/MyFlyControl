#include "attitude_self_stabilization.h"
#include "remote_control.h"
#include "motor_output.h"
#include "angle_control.h"
#include "ahrs_aux.h"

/**********************************************************************************************************
*�� �� ��: attitude_self_stabilization_control
*����˵��: ��̬���ȿ�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void attitude_self_stabilization_control()
{
	throttle_motor_output = Throttle_Control + 1000;
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
