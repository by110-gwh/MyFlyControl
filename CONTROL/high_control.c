#include "high_control.h"
#include "pid.h"
#include "navigation.h"

pid_controler_t high_pos_pid;
pid_controler_t high_vel_pid;

/**********************************************************************************************************
*�� �� ��: high_pos_control_pid_set
*����˵��: �߶�λ�û�pid��������
*��    ��: 10p���� 10i���� 10d����
*�� �� ֵ: ��
**********************************************************************************************************/
void high_pos_control_pid_set(uint8_t p, uint8_t i, uint8_t d)
{
    high_pos_pid.kp = p / 10.;
    high_pos_pid.ki = i / 10.;
    high_pos_pid.kd = d / 10.;
}

/**********************************************************************************************************
*�� �� ��: high_pos_control_init
*����˵��: �߶�pid�ṹ���ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void high_control_init()
{
    //�߶�λ��pid������ʼ��
    high_pos_pid.last_expect = 0;
    high_pos_pid.expect = 0;
    high_pos_pid.feedback = 0;

    high_pos_pid.err = 0;
    high_pos_pid.last_err = 0;
    high_pos_pid.err_max = 0;

    high_pos_pid.integrate_separation_err = 0;
    high_pos_pid.integrate = 0;
    high_pos_pid.integrate_max = 0;

    high_pos_pid.dis_err = 0;

    high_pos_pid.kp = 1;
    high_pos_pid.ki = 0;
    high_pos_pid.kd = 0;
    
    high_pos_pid.feedforward_kp = 0;
    high_pos_pid.feedforward_kd = 0;

    high_pos_pid.control_output = 0;
    high_pos_pid.control_output_limit = 0;

    high_pos_pid.short_circuit_flag = 0;
    high_pos_pid.err_callback = NULL;
    high_pos_pid.pri_data = NULL;
    
    
    //�߶��ٶ�pid������ʼ��
    high_vel_pid.last_expect = 0;
    high_vel_pid.expect = 0;
    high_vel_pid.feedback = 0;

    high_vel_pid.err = 0;
    high_vel_pid.last_err = 0;
    high_vel_pid.err_max = 0;

    high_vel_pid.integrate_separation_err = 0;
    high_vel_pid.integrate = 0;
    high_vel_pid.integrate_max = 0;

    high_vel_pid.dis_err = 0;

    high_vel_pid.kp = 1;
    high_vel_pid.ki = 0;
    high_vel_pid.kd = 0;
    
    high_vel_pid.feedforward_kp = 0;
    high_vel_pid.feedforward_kd = 0;

    high_vel_pid.control_output = 0;
    high_vel_pid.control_output_limit = 800;

    high_vel_pid.short_circuit_flag = 0;
    high_vel_pid.err_callback = NULL;
    high_vel_pid.pri_data = NULL;
}

/**********************************************************************************************************
*�� �� ��: high_pos_pid_integrate_reset
*����˵��: �߶�λ�û�pid��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void high_pos_pid_integrate_reset()
{
	high_pos_pid.integrate = 0;
	high_pos_pid.pid_controller_dt.inited = 0;
	high_pos_pid.last_err = 0;
	high_pos_pid.pre_last_err = 0;
	high_pos_pid.last_expect = 0;
	high_pos_pid.control_output = 0;
}

/**********************************************************************************************************
*�� �� ��: high_vel_pid_integrate_reset
*����˵��: �߶��ٶȻ�pid��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void high_vel_pid_integrate_reset()
{
	high_vel_pid.integrate = 0;
	high_vel_pid.pid_controller_dt.inited = 0;
	high_vel_pid.last_err = 0;
	high_vel_pid.pre_last_err = 0;
	high_vel_pid.last_expect = 0;
	high_vel_pid.control_output = 0;
}

/**********************************************************************************************************
*�� �� ��: high_control
*����˵��: �߶Ȼ�pid����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void high_control()
{
    //�߶�λ�ÿ��Ƽ�����
    static uint16_t high_pos_control_cnt;
    
    //��ֱ�߶ȿ�������2*5=10ms
    if (high_pos_control_cnt >= 2) {
        //�߶ȷ���
        high_pos_pid.feedback = high_pos;
        //�߶�λ��pid����
        pid_control(&high_pos_pid);
        
        high_pos_control_cnt = 0;
    }
    high_pos_control_cnt++;
    
    //�߶��ٶ�pid����
    high_vel_pid.expect = high_pos_pid.control_output;
    high_vel_pid.feedback = high_vel;
    pid_control(&high_vel_pid);
}
