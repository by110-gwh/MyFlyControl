#include "gyro_control.h"
#include "pid.h"
#include "imu.h"
#include "angle_control.h"
#include "Filter.h"
#include "math.h"

#define ABS(X)  (((X) > 0)? (X) : -(X))

pid_controler_t pitch_gyro_pid;
pid_controler_t roll_gyro_pid;
pid_controler_t yaw_gyro_pid;

typedef struct {
    float last_dis_err;
    float pre_last_dis_err;
    Butter_BufferData buffer;
    float raw_kd;
} pitch_roll_err_correct_t;

//�����pid����
pitch_roll_err_correct_t pitch_pri_dat;
pitch_roll_err_correct_t roll_pri_dat;
pitch_roll_err_correct_t yaw_pri_dat;

Butter_Parameter gyro_filter_parameter_30Hz;
Butter_Parameter gyro_filter_parameter_20Hz;

/**********************************************************************************************************
*�� �� ��: pitch_roll_err_correct
*����˵��: ����ĸ�����ƫ�����ٶ�pid����
*��    ��: pid�������ṹ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void pitch_roll_err_correct(pid_controler_t *controler)
{
    //���ڷ������˲�
    float tempa, tempb, tempc, max, min;
    float dis_err_filter;

    //�����һ�β�����΢��
    controler->dis_err = controler->err - controler->pre_last_err;

    //��ֵ�˲�����֤�õ����ݲ����䣬����������Ծʱ��΢������쳣
    tempa = ((pitch_roll_err_correct_t *)controler->pri_data)->pre_last_dis_err;
    tempb = ((pitch_roll_err_correct_t *)controler->pri_data)->last_dis_err;
    tempc = controler->dis_err;
    max = tempa > tempb ? tempa : tempb;
    max = max > tempc ? max : tempc;
    min = tempa < tempb ? tempa : tempb;
    min = min < tempc ? min : tempc;
    if (tempa > min && tempa < max)
        controler->dis_err = tempa;
    if (tempb > min && tempb < max)
        controler->dis_err = tempb;
    if (tempc > min && tempc < max)
        controler->dis_err = tempc;
    ((pitch_roll_err_correct_t *)controler->pri_data)->pre_last_dis_err =
        ((pitch_roll_err_correct_t *)controler->pri_data)->last_dis_err;
    ((pitch_roll_err_correct_t *)controler->pri_data)->last_dis_err = controler->dis_err;

    //������˹��ͨ��õ���΢����,30hz
    dis_err_filter = Butterworth_Filter(controler->dis_err,
        &((pitch_roll_err_correct_t *)controler->pri_data)->buffer,
        &gyro_filter_parameter_30Hz);

	if (dis_err_filter >= 500)
		dis_err_filter = 500;
	if (dis_err_filter <= -500)
		dis_err_filter = -500;
    //����Ӧ΢�ֲ���
    controler->kd = ((pitch_roll_err_correct_t *)controler->pri_data)->raw_kd
        * (1 + ABS(dis_err_filter) / 500.0f);    
}

/**********************************************************************************************************
*�� �� ��: yaw_err_correct
*����˵��: �����ƫ�����ٶ�pid����
*��    ��: pid�������ṹ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void yaw_err_correct(pid_controler_t *controler)
{
    //���ڷ������˲�
    float tempa, tempb, tempc, max, min;

    //��ֵ�˲�����֤�õ����ݲ����䣬����������Ծʱ��΢������쳣
    tempa = ((pitch_roll_err_correct_t *)controler->pri_data)->pre_last_dis_err;
    tempb = ((pitch_roll_err_correct_t *)controler->pri_data)->last_dis_err;
    tempc = controler->dis_err;
    max = tempa > tempb ? tempa : tempb;
    max = max > tempc ? max : tempc;
    min = tempa < tempb ? tempa : tempb;
    min = min < tempc ? min : tempc;
    if (tempa > min && tempa < max)
        controler->dis_err = tempa;
    if (tempb > min && tempb < max)
        controler->dis_err = tempb;
    if (tempc > min && tempc < max)
        controler->dis_err = tempc;
    ((pitch_roll_err_correct_t *)controler->pri_data)->pre_last_dis_err =
        ((pitch_roll_err_correct_t *)controler->pri_data)->last_dis_err;
    ((pitch_roll_err_correct_t *)controler->pri_data)->last_dis_err = controler->dis_err;
    //������˹��ͨ��õ���΢����,30hz
    controler->dis_err = Butterworth_Filter(controler->dis_err,
        &((pitch_roll_err_correct_t *)controler->pri_data)->buffer,
        &gyro_filter_parameter_20Hz);
}


/**********************************************************************************************************
*�� �� ��: gyro_control_init
*����˵��: ���ٶȶȻ�pid�ṹ���ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void gyro_control_init()
{
    //������pid������ʼ��
    pitch_gyro_pid.last_expect = 0;
    pitch_gyro_pid.expect = 0;
    pitch_gyro_pid.feedback = 0;

    pitch_gyro_pid.err = 0;
    pitch_gyro_pid.last_err = 0;
    pitch_gyro_pid.err_max = 500;

    pitch_gyro_pid.integrate_separation_err = 0;
    pitch_gyro_pid.integrate = 0;
    pitch_gyro_pid.integrate_max = 300;

    pitch_gyro_pid.dis_err = 0;

    pitch_gyro_pid.kp = 0.75;
    pitch_gyro_pid.ki = 2.5;
    pitch_gyro_pid.kd = 2;
    
    pitch_gyro_pid.feedforward_kp = 0.1;
    pitch_gyro_pid.feedforward_kd = 0;

    pitch_gyro_pid.control_output = 0;
    pitch_gyro_pid.control_output_limit = 500;

    pitch_gyro_pid.short_circuit_flag = 0;
    pitch_gyro_pid.err_callback = pitch_roll_err_correct;
    pitch_gyro_pid.pri_data = &pitch_pri_dat;

    //�����pid������ʼ��
    roll_gyro_pid.last_expect = 0;
    roll_gyro_pid.expect = 0;
    roll_gyro_pid.feedback = 0;

    roll_gyro_pid.err = 0;
    roll_gyro_pid.last_err = 0;
    roll_gyro_pid.err_max = 500;

    roll_gyro_pid.integrate_separation_err = 0;
    roll_gyro_pid.integrate = 0;
    roll_gyro_pid.integrate_max = 300;

    roll_gyro_pid.dis_err = 0;

    roll_gyro_pid.kp = 0.75;
    roll_gyro_pid.ki = 2.5;
    roll_gyro_pid.kd = 2;

    roll_gyro_pid.feedforward_kp = 0.1;
    roll_gyro_pid.feedforward_kd = 0;

    roll_gyro_pid.control_output = 0;
    roll_gyro_pid.control_output_limit = 500;

    roll_gyro_pid.short_circuit_flag = 0;
    roll_gyro_pid.err_callback = pitch_roll_err_correct;
    roll_gyro_pid.pri_data = &roll_pri_dat;

    //ƫ��pid������ʼ��
    yaw_gyro_pid.last_expect = 0;
    yaw_gyro_pid.expect = 0;
    yaw_gyro_pid.feedback = 0;

    yaw_gyro_pid.err = 0;
    yaw_gyro_pid.last_err = 0;
    yaw_gyro_pid.err_max = 300;

    yaw_gyro_pid.integrate_separation_err = 0;
    yaw_gyro_pid.integrate = 0;
    yaw_gyro_pid.integrate_max = 200;

    yaw_gyro_pid.dis_err = 0;

    yaw_gyro_pid.kp = 1.2;
    yaw_gyro_pid.ki = 0.5;
    yaw_gyro_pid.kd = 0;

    yaw_gyro_pid.feedforward_kp = 0;
    yaw_gyro_pid.feedforward_kd = 0.05;

    yaw_gyro_pid.control_output = 0;
    yaw_gyro_pid.control_output_limit = 500;

    yaw_gyro_pid.short_circuit_flag = 0;
    yaw_gyro_pid.err_callback = yaw_err_correct;
    yaw_gyro_pid.pri_data = &yaw_pri_dat;

    pitch_pri_dat.raw_kd = 2;
    roll_pri_dat.raw_kd = 2;
    yaw_pri_dat.raw_kd = 0;

    Set_Cutoff_Frequency(Sampling_Freq, 30, &gyro_filter_parameter_30Hz);
    Set_Cutoff_Frequency(Sampling_Freq, 20, &gyro_filter_parameter_20Hz);
}

/**********************************************************************************************************
*�� �� ��: gyro_control
*����˵��: ���ٶȶȻ�pid����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void gyro_control()
{
    pitch_gyro_pid.feedback = gyroDataFilter.x * GYRO_CALIBRATION_COFF;
    pitch_gyro_pid.expect = pitch_angle_pid.control_output;
    pid_control(&pitch_gyro_pid);

    roll_gyro_pid.feedback = gyroDataFilter.y * GYRO_CALIBRATION_COFF;
    roll_gyro_pid.expect = roll_angle_pid.control_output;
    pid_control(&roll_gyro_pid);

    yaw_gyro_pid.feedback = gyroDataFilter.z * GYRO_CALIBRATION_COFF;
    yaw_gyro_pid.expect = yaw_angle_pid.control_output;
    pid_control(&yaw_gyro_pid);
}
