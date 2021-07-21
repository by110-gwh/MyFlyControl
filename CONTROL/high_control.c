#include "high_control.h"
#include "pid.h"
#include "navigation.h"

pid_paramer_t high_pos_pid_para = {
    .err_max = 0,
    .integrate_separation_err = 0,
    .integrate_max = 0,
    .kp = 4,
    .ki = 0,
    .kd = 0,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 50
};

pid_paramer_t high_speed_pid_para = {
    .err_max = 0,
    .integrate_separation_err = 0,
    .integrate_max = 0,
    .kp = 4,
    .ki = 0.2,
    .kd = 0,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 800
};

pid_data_t high_pos_pid_data;
pid_data_t high_speed_pid_data;

/**********************************************************************************************************
*�� �� ��: high_pos_control_pid_set
*����˵��: �߶�λ�û�pid��������
*��    ��: 10p���� 10i���� 10d����
*�� �� ֵ: ��
**********************************************************************************************************/
void high_pos_control_pid_set(uint8_t p, uint8_t i, uint8_t d)
{
    high_pos_pid_para.kp = p / 10.;
    high_pos_pid_para.ki = i / 10.;
    high_pos_pid_para.kd = d / 10.;
}

/**********************************************************************************************************
*�� �� ��: high_pos_control_init
*����˵��: �߶�pid�ṹ���ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void high_control_init()
{
    high_pos_pid_data.last_expect = 0;
	high_pos_pid_data.expect = 0;
    high_pos_pid_data.feedback = 0;
	high_pos_pid_data.last_err = 0;
	high_pos_pid_data.pre_last_err = 0;
    high_pos_pid_data.integrate = 0;
    high_pos_pid_data.dis_err = 0;
	high_pos_pid_data.control_output = 0;
	high_pos_pid_data.pid_controller_dt.inited = 0;
    high_pos_pid_data.err_callback = NULL;
    high_pos_pid_data.pri_data = NULL;
    high_pos_pid_data.short_circuit_flag = 0;
    
    high_speed_pid_data.last_expect = 0;
	high_speed_pid_data.expect = 0;
    high_speed_pid_data.feedback = 0;
	high_speed_pid_data.last_err = 0;
	high_speed_pid_data.pre_last_err = 0;
    high_speed_pid_data.integrate = 0;
    high_speed_pid_data.dis_err = 0;
	high_speed_pid_data.control_output = 0;
	high_speed_pid_data.pid_controller_dt.inited = 0;
    high_speed_pid_data.err_callback = NULL;
    high_speed_pid_data.pri_data = NULL;
    high_speed_pid_data.short_circuit_flag = 0;
}

/**********************************************************************************************************
*�� �� ��: high_pos_pid_integrate_reset
*����˵��: �߶�λ�û�pid��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void high_pos_pid_integrate_reset()
{
	high_pos_pid_data.integrate = 0;
	high_pos_pid_data.pid_controller_dt.inited = 0;
	high_pos_pid_data.last_err = 0;
	high_pos_pid_data.pre_last_err = 0;
	high_pos_pid_data.last_expect = 0;
	high_pos_pid_data.control_output = 0;
}

/**********************************************************************************************************
*�� �� ��: high_speed_pid_integrate_reset
*����˵��: �߶��ٶȻ�pid��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void high_speed_pid_integrate_reset()
{
	high_speed_pid_data.integrate = 0;
	high_speed_pid_data.pid_controller_dt.inited = 0;
	high_speed_pid_data.last_err = 0;
	high_speed_pid_data.pre_last_err = 0;
	high_speed_pid_data.last_expect = 0;
	high_speed_pid_data.control_output = 0;
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
        high_pos_pid_data.feedback = pos_z;
        //�߶�λ��pid����
        pid_control(&high_pos_pid_data, &high_pos_pid_para);
        
        high_pos_control_cnt = 0;
    }
    high_pos_control_cnt++;
    
    //�߶��ٶ�pid����
    high_speed_pid_data.expect = high_pos_pid_data.control_output;
    high_speed_pid_data.feedback = speed_z;
    pid_control(&high_speed_pid_data, &high_speed_pid_para);
}
