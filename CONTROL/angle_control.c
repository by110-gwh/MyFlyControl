#include "angle_control.h"
#include "pid.h"
#include "ahrs_aux.h"
#include "controller.h"

pid_paramer_t pitch_angle_pid_para = {
    .err_max = 30,
    .integrate_separation_err = 0,
    .integrate_max = 80,
    .kp = 5,
    .ki = 0,
    .kd = 0,
    .control_output_limit = 450
};

pid_paramer_t roll_angle_pid_para = {
    .err_max = 30,
    .integrate_separation_err = 0,
    .integrate_max = 80,
    .kp = 5,
    .ki = 0,
    .kd = 0,
    .control_output_limit = 450
};

pid_paramer_t yaw_angle_pid_para = {
    .err_max = 45,
    .integrate_separation_err = 0,
    .integrate_max = 150,
    .kp = 5,
    .ki = 0,
    .kd = 0,
    .control_output_limit = 20
};

pid_data_t pitch_angle_pid_data;
pid_data_t roll_angle_pid_data;
pid_data_t yaw_angle_pid_data;

/**********************************************************************************************************
*�� �� ��: yaw_err_correct
*����˵��: ƫ�������Ƕ�У��
*��    ��: ppid���������ݽṹ�� pid����������
*�� �� ֵ: ��
**********************************************************************************************************/
static void yaw_err_correct(pid_data_t *data, pid_paramer_t *para)
{
    if(data->err < -180)
        data->err = data->err + 360;
    if(data->err > 180)
        data->err = data->err - 360;
}

/**********************************************************************************************************
*�� �� ��: angle_control_pid_set
*����˵��: �ǶȻ�pid��������
*��    ��: 10p���� 10i���� 10d����
*�� �� ֵ: ��
**********************************************************************************************************/
void angle_control_pid_set(uint8_t p, uint8_t i, uint8_t d)
{
	pitch_angle_pid_para.kp = p / 10.;
	pitch_angle_pid_para.ki = i / 10.;
	pitch_angle_pid_para.kd = d / 10.;
	angle_pid_integrate_reset();
}

/**********************************************************************************************************
*�� �� ��: angle_control_init
*����˵��: �ǶȻ�pid�ṹ���ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void angle_control_init()
{
	yaw_angle_pid_data.last_expect = 0;
	yaw_angle_pid_data.expect = 0;
    yaw_angle_pid_data.feedback = 0;
	yaw_angle_pid_data.last_err = 0;
	yaw_angle_pid_data.pre_last_err = 0;
    yaw_angle_pid_data.integrate = 0;
    yaw_angle_pid_data.dis_err = 0;
	yaw_angle_pid_data.control_output = 0;
	yaw_angle_pid_data.pid_controller_dt.inited = 0;
    yaw_angle_pid_data.err_callback = yaw_err_correct;
    yaw_angle_pid_data.pri_data = NULL;
    yaw_angle_pid_data.short_circuit_flag = 0;
    
	pitch_angle_pid_data.last_expect = 0;
	pitch_angle_pid_data.expect = 0;
    pitch_angle_pid_data.feedback = 0;
	pitch_angle_pid_data.last_err = 0;
	pitch_angle_pid_data.pre_last_err = 0;
    pitch_angle_pid_data.integrate = 0;
    pitch_angle_pid_data.dis_err = 0;
	pitch_angle_pid_data.control_output = 0;
	pitch_angle_pid_data.pid_controller_dt.inited = 0;
    pitch_angle_pid_data.err_callback = NULL;
    pitch_angle_pid_data.pri_data = NULL;
    pitch_angle_pid_data.short_circuit_flag = 0;
    
	roll_angle_pid_data.last_expect = 0;
	roll_angle_pid_data.expect = 0;
    roll_angle_pid_data.feedback = 0;
	roll_angle_pid_data.last_err = 0;
	roll_angle_pid_data.pre_last_err = 0;
    roll_angle_pid_data.integrate = 0;
    roll_angle_pid_data.dis_err = 0;
	roll_angle_pid_data.control_output = 0;
	roll_angle_pid_data.pid_controller_dt.inited = 0;
    roll_angle_pid_data.err_callback = NULL;
    roll_angle_pid_data.pri_data = NULL;
    roll_angle_pid_data.short_circuit_flag = 0;
}

/**********************************************************************************************************
*�� �� ��: angle_pid_integrate_reset
*����˵��: �ǶȻ�pid��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void angle_pid_integrate_reset()
{
	yaw_angle_pid_data.integrate = 0;
	yaw_angle_pid_data.pid_controller_dt.inited = 0;
	yaw_angle_pid_data.last_err = 0;
	yaw_angle_pid_data.pre_last_err = 0;
	yaw_angle_pid_data.last_expect = 0;
	yaw_angle_pid_data.control_output = 0;
	
	pitch_angle_pid_data.integrate = 0;
	pitch_angle_pid_data.pid_controller_dt.inited = 0;
	pitch_angle_pid_data.last_err = 0;
	pitch_angle_pid_data.pre_last_err = 0;
	pitch_angle_pid_data.last_expect = 0;
	pitch_angle_pid_data.control_output = 0;
	
	roll_angle_pid_data.integrate = 0;
	roll_angle_pid_data.pid_controller_dt.inited = 0;
	roll_angle_pid_data.last_err = 0;
	roll_angle_pid_data.pre_last_err = 0;
	roll_angle_pid_data.last_expect = 0;
	roll_angle_pid_data.control_output = 0;
}

/**********************************************************************************************************
*�� �� ��: angle_control
*����˵��: �ǶȻ�pid����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void angle_control()
{
    pitch_angle_pid_data.feedback = Pitch + 0;
    pid_control(&pitch_angle_pid_data, &pitch_angle_pid_para);

    roll_angle_pid_data.feedback = Roll + 0;
    pid_control(&roll_angle_pid_data, &roll_angle_pid_para);

    yaw_angle_pid_data.feedback = Yaw;
    pid_control(&yaw_angle_pid_data, &yaw_angle_pid_para);
}
