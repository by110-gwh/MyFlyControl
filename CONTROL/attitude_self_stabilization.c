#include "attitude_self_stabilization.h"
#include "remote_control.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "high_control.h"
#include "horizontal_control.h"
#include "ahrs_aux.h"
#include "controller.h"
#include "motor_output.h"

/**********************************************************************************************************
*�� �� ��: attitude_self_stabilization_init
*����˵��: ��̬���ȿ�������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void attitude_self_stabilization_init(void)
{
    
}

/**********************************************************************************************************
*�� �� ��: attitude_self_stabilization_control
*����˵��: ��̬���ȿ�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void attitude_self_stabilization_control()
{
    //�Ƕȿ�������ң����
    pitch_angle_pid_data.expect = Pitch_Control;
	roll_angle_pid_data.expect = Roll_Control;
    
    //δ���ʱ����̬����������
	if (Throttle_Control == 0) {
		angle_pid_integrate_reset();
		gyro_pid_integrate_reset();
		yaw_angle_pid_data.expect = Yaw;
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
    
    //�ǶȻ�������
    angle_control();
    //���ٶȿ�����
    gyro_control();
    //���Ų���
    throttle_motor_output = throttle_angle_compensate(Throttle_Control + 1000);
}
