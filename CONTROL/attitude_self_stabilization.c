#include "attitude_self_stabilization.h"
#include "remote_control.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "high_control.h"
#include "ahrs_aux.h"

/**********************************************************************************************************
*�� �� ��: attitude_self_stabilization_control
*����˵��: ��̬���ȿ�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void attitude_self_stabilization_control()
{
    //����ֱ�����������������pid��·��־����ֱ�����
    high_pos_pid.expect = Throttle_Control;
    
    //�Ƕȿ�������ң����
	pitch_angle_pid.expect = Pitch_Control;
	roll_angle_pid.expect = Roll_Control;
    
    //δ���ʱ����̬����������
	if (Throttle_Control == 1000) {
		angle_pid_integrate_reset();
		gyro_pid_integrate_reset();
		yaw_angle_pid.expect = Yaw;
	}

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
