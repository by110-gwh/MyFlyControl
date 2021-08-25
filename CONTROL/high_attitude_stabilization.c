#include "high_attitude_stabilization.h"
#include "remote_control.h"
#include "motor_output.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "high_control.h"
#include "controller.h"
#include "high_control.h"
#include "horizontal_control.h"

/**********************************************************************************************************
*�� �� ��: high_attitude_stabilization_init
*����˵��: ���߿�������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void high_attitude_stabilization_init(void)
{
    high_pos_pid_integrate_reset();
    high_speed_pid_integrate_reset();
}

/**********************************************************************************************************
*�� �� ��: high_attitude_stabilization_control
*����˵��: ���߿�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void high_attitude_stabilization_control()
{
    high_pos_pid_data.expect = (Throttle_Control - HOLD_THROTTLE) / (float)(1000 - HOLD_THROTTLE) * 200 + 10;

    //�Ƕȿ�������ң����
    pitch_angle_pid_data.expect = Pitch_Control;
	roll_angle_pid_data.expect = Roll_Control;

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
    
    //�߶Ȼ�������
    high_control();
    //�ǶȻ�������
    angle_control();
    //���ٶȿ�����
    gyro_control();
    //���Ų���
    throttle_motor_output = throttle_angle_compensate(high_speed_pid_data.control_output + HOLD_THROTTLE + 1000);
}
