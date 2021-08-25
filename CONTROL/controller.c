#include "controller.h"
#include "remote_control.h"
#include "ahrs_aux.h"
#include "attitude_self_stabilization.h"
#include "high_attitude_stabilization.h"
#include "horizontal_attitude_stabilization.h"
#include "route_plan_attitude_stabilization.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "high_control.h"
#include "horizontal_control.h"
#include "motor_output.h"
#include "route_plan_task.h"
#include "beep_task.h"
#include "navigation.h"
#include "main_task.h"

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
    horizontal_control_init();
    controller_state = 0;
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
uint16_t throttle_angle_compensate(uint16_t throttle)
{
	uint16_t throttle_output;
	float CosPitch_CosRoll = Cos_Pitch * Cos_Roll;
	//��������
	if(CosPitch_CosRoll <= 0.50f)
		CosPitch_CosRoll = 0.50f;
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
    if (fly_task_num && controller_state != 5) {
        route_plan_attitude_stabilization_init();
        controller_state = 5;
    } else if (!fly_task_num && Throttle_Control < 100) {
        if (rc_raw_data[6] > 1500) {
            route_plan_attitude_stabilization_init();
            controller_state = 4;
        } else if (rc_raw_data[4] < 1250) {
            attitude_self_stabilization_init();
            controller_state = 1;
        } else if (rc_raw_data[4] < 1750) {
            high_attitude_stabilization_init();
            controller_state = 2;
        } else {
            horizontal_attitude_stabilization_init();
            controller_state = 3;
        }
        gyro_pid_integrate_reset();
        angle_pid_integrate_reset();
    }
    
    //����̬ģʽ
    if (controller_state == 1) {
        //����̬������
        attitude_self_stabilization_control();
    //����ģʽ
    } else if (controller_state == 2) {
        if (Throttle_Control < HOLD_THROTTLE)
            attitude_self_stabilization_control();
        else
            high_attitude_stabilization_control();
    //����ģʽ
    } else if (controller_state == 3) {
        if (Throttle_Control < HOLD_THROTTLE)
            attitude_self_stabilization_control();
        else
            horizontal_attitude_stabilization_control();
    } else if (controller_state == 4) {
        if (Throttle_Control < 100) {   
            pitch_gyro_pid_data.control_output = 0;
            roll_gyro_pid_data.control_output = 0;
            yaw_gyro_pid_data.control_output = 0;
            throttle_motor_output = 1000;
        } else {
            route_plan_attitude_stabilization_control();
        }
    } else if (controller_state == 5) {
        route_plan_attitude_stabilization_control();
    }
}
