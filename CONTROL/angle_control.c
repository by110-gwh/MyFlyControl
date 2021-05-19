#include "angle_control.h"
#include "pid.h"
#include "ahrs_aux.h"

pid_controler_t pitch_angle_pid;
pid_controler_t roll_angle_pid;
pid_controler_t yaw_angle_pid;

/**********************************************************************************************************
*�� �� ��: yaw_err_correct
*����˵��: ƫ�������Ƕ�У��
*��    ��: pid�������ṹ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void yaw_err_correct(pid_controler_t *controler)
{
    if(controler->err < -180)
        controler->err = controler->err + 360;
    if(controler->err > 180)
        controler->err = controler->err - 360;
}

/**********************************************************************************************************
*�� �� ��: angle_control_pid_set
*����˵��: �ǶȻ�pid��������
*��    ��: 10p���� 10i���� 10d����
*�� �� ֵ: ��
**********************************************************************************************************/
void angle_control_pid_set(uint8_t p, uint8_t i, uint8_t d)
{
	pitch_angle_pid.kp = p / 10.;
	pitch_angle_pid.ki = i / 10.;
	pitch_angle_pid.kd = d / 10.;
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
    //������pid������ʼ��
    pitch_angle_pid.last_expect = 0;
    pitch_angle_pid.expect = 0;
    pitch_angle_pid.feedback = 0;

    pitch_angle_pid.err = 0;
    pitch_angle_pid.last_err = 0;
    pitch_angle_pid.err_max = 30;

    pitch_angle_pid.integrate_separation_err = 0;
    pitch_angle_pid.integrate = 0;
    pitch_angle_pid.integrate_max = 80;

    pitch_angle_pid.dis_err = 0;

    pitch_angle_pid.kp = 3;
    pitch_angle_pid.ki = 0;
    pitch_angle_pid.kd = 0;
    
    pitch_angle_pid.feedforward_kp = 0;
    pitch_angle_pid.feedforward_kd = 0;

    pitch_angle_pid.control_output = 0;
    pitch_angle_pid.control_output_limit = 450;

    pitch_angle_pid.short_circuit_flag = 0;
    pitch_angle_pid.err_callback = NULL;
    pitch_angle_pid.pri_data = NULL;

    //�����pid������ʼ��
    roll_angle_pid.last_expect = 0;
    roll_angle_pid.expect = 0;
    roll_angle_pid.feedback = 0;

    roll_angle_pid.err = 0;
    roll_angle_pid.last_err = 0;
    roll_angle_pid.err_max = 30;

    roll_angle_pid.integrate_separation_err = 0;
    roll_angle_pid.integrate = 0;
    roll_angle_pid.integrate_max = 80;

    roll_angle_pid.dis_err = 0;

    roll_angle_pid.kp = 3;
    roll_angle_pid.ki = 0;
    roll_angle_pid.kd = 0;

    roll_angle_pid.feedforward_kp = 0;
    roll_angle_pid.feedforward_kd = 0;

    roll_angle_pid.control_output = 0;
    roll_angle_pid.control_output_limit = 450;

    roll_angle_pid.short_circuit_flag = 0;
    roll_angle_pid.err_callback = NULL;
    roll_angle_pid.pri_data = NULL;

    //ƫ��pid������ʼ��
    yaw_angle_pid.last_expect = 0;
    yaw_angle_pid.expect = 0;
    yaw_angle_pid.feedback = 0;

    yaw_angle_pid.err = 0;
    yaw_angle_pid.last_err = 0;
    yaw_angle_pid.err_max = 45;

    yaw_angle_pid.integrate_separation_err = 0;
    yaw_angle_pid.integrate = 0;
    yaw_angle_pid.integrate_max = 150;

    yaw_angle_pid.dis_err = 0;

    yaw_angle_pid.kp = 3;
    yaw_angle_pid.ki = 0;
    yaw_angle_pid.kd = 0;

    yaw_angle_pid.feedforward_kp = 0;
    yaw_angle_pid.feedforward_kd = 0;

    yaw_angle_pid.control_output = 0;
    yaw_angle_pid.control_output_limit = 20;

    yaw_angle_pid.short_circuit_flag = 0;
    yaw_angle_pid.err_callback = yaw_err_correct;
    yaw_angle_pid.pri_data = NULL;
}

/**********************************************************************************************************
*�� �� ��: angle_pid_integrate_reset
*����˵��: �ǶȻ�pid��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void angle_pid_integrate_reset()
{
	yaw_angle_pid.integrate = 0;
	yaw_angle_pid.pid_controller_dt.inited = 0;
	yaw_angle_pid.last_err = 0;
	yaw_angle_pid.pre_last_err = 0;
	yaw_angle_pid.last_expect = 0;
	yaw_angle_pid.control_output = 0;
	
	pitch_angle_pid.integrate = 0;
	pitch_angle_pid.pid_controller_dt.inited = 0;
	pitch_angle_pid.last_err = 0;
	pitch_angle_pid.pre_last_err = 0;
	pitch_angle_pid.last_expect = 0;
	pitch_angle_pid.control_output = 0;
	
	roll_angle_pid.integrate = 0;
	roll_angle_pid.pid_controller_dt.inited = 0;
	roll_angle_pid.last_err = 0;
	roll_angle_pid.pre_last_err = 0;
	roll_angle_pid.last_expect = 0;
	roll_angle_pid.control_output = 0;
}

/**********************************************************************************************************
*�� �� ��: angle_control
*����˵��: �ǶȻ�pid����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void angle_control()
{
    pitch_angle_pid.feedback = Pitch;
    pid_control(&pitch_angle_pid);

    roll_angle_pid.feedback = Roll;
    pid_control(&roll_angle_pid);

    yaw_angle_pid.feedback = Yaw;
    pid_control(&yaw_angle_pid);
}
