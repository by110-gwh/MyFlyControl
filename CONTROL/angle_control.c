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
*�� �� ��: angle_control_init
*����˵��: �ǶȻ�pid�ṹ���ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void angle_control_init()
{
    //������pid������ʼ��
    pitch_angle_pid.expect = 0;
    pitch_angle_pid.feedback = 0;

    pitch_angle_pid.err = 0;
    pitch_angle_pid.last_err = 0;
    pitch_angle_pid.err_max = 30;

    pitch_angle_pid.integrate_separation_err = 0;
    pitch_angle_pid.integrate = 0;
    pitch_angle_pid.integrate_max = 80;

    pitch_angle_pid.dis_err = 0;

    pitch_angle_pid.kp = 4;
    pitch_angle_pid.ki = 0;
    pitch_angle_pid.kd = 0;

    pitch_angle_pid.control_output = 0;
    pitch_angle_pid.control_output_limit = 450;

    pitch_angle_pid.short_circuit_flag = 0;
    pitch_angle_pid.pri_data = NULL;

    //�����pid������ʼ��
    roll_angle_pid.expect = 0;
    roll_angle_pid.feedback = 0;

    roll_angle_pid.err = 0;
    roll_angle_pid.last_err = 0;
    roll_angle_pid.err_max = 30;

    roll_angle_pid.integrate_separation_err = 0;
    roll_angle_pid.integrate = 0;
    roll_angle_pid.integrate_max = 80;

    roll_angle_pid.dis_err = 0;

    roll_angle_pid.kp = 4;
    roll_angle_pid.ki = 0;
    roll_angle_pid.kd = 0;

    roll_angle_pid.control_output = 0;
    roll_angle_pid.control_output_limit = 450;

    pitch_angle_pid.short_circuit_flag = 0;
    roll_angle_pid.pri_data = NULL;

    //ƫ��pid������ʼ��
    yaw_angle_pid.expect = 0;
    yaw_angle_pid.feedback = 0;

    yaw_angle_pid.err = 0;
    yaw_angle_pid.last_err = 0;
    yaw_angle_pid.err_max = 45;

    yaw_angle_pid.integrate_separation_err = 0;
    yaw_angle_pid.integrate = 0;
    yaw_angle_pid.integrate_max = 150;

    yaw_angle_pid.dis_err = 0;

    yaw_angle_pid.kp = 4;
    yaw_angle_pid.ki = 0;
    yaw_angle_pid.kd = 0;

    yaw_angle_pid.control_output = 0;
    yaw_angle_pid.control_output_limit = 450;

    yaw_angle_pid.short_circuit_flag = 0;
    yaw_angle_pid.pri_data = yaw_err_correct;
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
