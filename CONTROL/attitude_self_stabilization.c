#include "attitude_self_stabilization.h"
#include "remote_control.h"
#include "motor_output.h"
#include "angle_control.h"
#include "ahrs_aux.h"

/**********************************************************************************************************
*�� �� ��: constrain_float
*����˵��: ���������Сֵ
*��    ��: Ҫ���Ƶ�ֵ ��Сֵ ���ֵ
*�� �� ֵ: ���ֵ
**********************************************************************************************************/
float constrain_float(float amt, float low, float high)
{
	return ((amt) < (low) ? (low) : ((amt) > (high)? (high) : (amt)));
}

/**********************************************************************************************************
*�� �� ��: throttle_angle_compensate
*����˵��: ������ǲ���
*��    ��: ԭ������
*�� �� ֵ: �������������
**********************************************************************************************************/
uint16_t throttle_angle_compensate(uint16_t throttle)
{
	uint16_t throttle_output;
	float CosPitch_CosRoll = Cos_Pitch * Cos_Roll;
	//��������
	if(CosPitch_CosRoll<=0.50)
		CosPitch_CosRoll=0.50;
	//������ת������
	if(throttle >= 1000) {
		//������ǲ���
		throttle_output = 1000 + (throttle - 1000) / CosPitch_CosRoll;
		throttle_output = (uint16_t)(constrain_float(throttle_output, 1000, 2000));
	} else
		throttle_output = throttle;
	return throttle_output;
}

/**********************************************************************************************************
*�� �� ��: attitude_self_stabilization_control
*����˵��: ��̬���ȿ�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void attitude_self_stabilization_control()
{
	throttle_motor_output = throttle_angle_compensate(Throttle_Control + 1000);
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
