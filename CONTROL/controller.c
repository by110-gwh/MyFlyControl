#include "controller.h"
#include "remote_control.h"
#include "ahrs_aux.h"
#include "attitude_self_stabilization.h"
#include "high_attitude_stabilization.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "high_control.h"
#include "motor_output.h"

uint8_t controller_last_state;
uint8_t controller_state;

/**********************************************************************************************************
*�� �� ��: control_init
*����˵��: ��������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void controller_init()
{
    angle_control_init();
    gyro_control_init();
    high_control_init();
    
}

/**********************************************************************************************************
*�� �� ��: constrain_float
*����˵��: ���������Сֵ
*��    ��: Ҫ���Ƶ�ֵ ��Сֵ ���ֵ
*�� �� ֵ: ���ֵ
**********************************************************************************************************/
static float constrain_float(float amt, float low, float high)
{
	return ((amt) < (low) ? (low) : ((amt) > (high)? (high) : (amt)));
}

/**********************************************************************************************************
*�� �� ��: throttle_angle_compensate
*����˵��: ������ǲ���
*��    ��: ԭ������
*�� �� ֵ: �������������
**********************************************************************************************************/
static uint16_t throttle_angle_compensate(uint16_t throttle)
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
*�� �� ��: controller_run
*����˵��: ������ִ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void controller_run()
{
    controller_last_state = controller_state;
    
    if (rc_raw_data[4] < rc_calibration_data[4].middle) {
        //����̬ģʽ
        controller_state = 1;
    } else if (Throttle_Control > 0) {
        //����ģʽ
        controller_state = 2;
    }
    
    if (controller_state == 1) {
        //��һ�����д���̬������
        if (controller_last_state != 1) {
            high_pos_pid.short_circuit_flag = 1;
            high_vel_pid.short_circuit_flag = 1;
        }
        //����̬������
        attitude_self_stabilization_control();
    } else if (controller_state == 2) {
        //��һ�����ж��߿�����
        if (controller_last_state != 2) {
            high_vel_pid.short_circuit_flag = 0;
            high_pos_pid_integrate_reset();
            high_vel_pid_integrate_reset();
        }
        //���߿�����
        high_attitude_stabilization_control();
    }
    
    //�߶Ȼ�������
    high_control();
    //�ǶȻ�������
    angle_control();
    //���ٶȿ�����
    gyro_control();
    
    
    if (controller_state == 1) {
        //���Ų���
        throttle_motor_output = throttle_angle_compensate(high_vel_pid.control_output + 1000);
    } else if (controller_state == 2) {
        //���Ų���
        throttle_motor_output = throttle_angle_compensate(high_vel_pid.control_output + 1100);
    }
    
}