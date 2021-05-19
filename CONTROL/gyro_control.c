#include "gyro_control.h"
#include "pid.h"
#include "imu.h"
#include "angle_control.h"
#include "Filter.h"
#include "math.h"
#include "string.h"

pid_controler_t pitch_gyro_pid;
pid_controler_t roll_gyro_pid;
pid_controler_t yaw_gyro_pid;

typedef struct {
    Butter_BufferData buffer;
} pitch_roll_err_correct_t;

//�����pid����
static pitch_roll_err_correct_t pitch_pri_dat;
static pitch_roll_err_correct_t roll_pri_dat;
static pitch_roll_err_correct_t yaw_pri_dat;

static Butter_BufferData pitch_pri_dat2;
static Butter_BufferData roll_pri_dat2;
static Butter_BufferData yaw_pri_dat2;

static Butter_Parameter gyro_filter_parameter_30Hz;

/**********************************************************************************************************
*�� �� ��: pitch_roll_err_correct
*����˵��: ����ĸ�����ƫ�����ٶ�pid����
*��    ��: pid�������ṹ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void pitch_roll_err_correct(pid_controler_t *controler)
{
    //������˹��ͨ��õ���΢����,30hz
    controler->dis_err = Butterworth_Filter(controler->dis_err,
        &((pitch_roll_err_correct_t *)controler->pri_data)->buffer,
        &gyro_filter_parameter_30Hz);
}

/**********************************************************************************************************
*�� �� ��: gyro_control_pid_set
*����˵��: ���ٶȻ�pid��������
*��    ��: 10p���� 10i���� 10d����
*�� �� ֵ: ��
**********************************************************************************************************/
void gyro_control_pid_set(uint8_t p, uint8_t i, uint8_t d)
{
    pitch_gyro_pid.kp = p / 10.;
    pitch_gyro_pid.ki = i / 10.;
    pitch_gyro_pid.kd = d / 10.;
	
	gyro_pid_integrate_reset();
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
    pitch_gyro_pid.integrate_max = 100;

    pitch_gyro_pid.dis_err = 0;

    pitch_gyro_pid.kp = 1.8;
    pitch_gyro_pid.ki = 5;
    pitch_gyro_pid.kd = 9;
    
    pitch_gyro_pid.feedforward_kp = 0;
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
    roll_gyro_pid.integrate_max = 100;

    roll_gyro_pid.dis_err = 0;

    roll_gyro_pid.kp = 1.8;
    roll_gyro_pid.ki = 5;
    roll_gyro_pid.kd = 9;

    roll_gyro_pid.feedforward_kp = 0;
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

    yaw_gyro_pid.kp = 5;
    yaw_gyro_pid.ki = 1.5;
    yaw_gyro_pid.kd = 0;

    yaw_gyro_pid.feedforward_kp = 0;
    yaw_gyro_pid.feedforward_kd = 0;

    yaw_gyro_pid.control_output = 0;
    yaw_gyro_pid.control_output_limit = 500;

    yaw_gyro_pid.short_circuit_flag = 0;
    yaw_gyro_pid.err_callback = pitch_roll_err_correct;
    yaw_gyro_pid.pri_data = &yaw_pri_dat;
	
	Set_Cutoff_Frequency(Sampling_Freq, 30, &gyro_filter_parameter_30Hz);
}

/**********************************************************************************************************
*�� �� ��: gyro_pid_integrate_reset
*����˵��: ���ٶȻ�pid��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void gyro_pid_integrate_reset()
{
	yaw_gyro_pid.integrate = 0;
	yaw_gyro_pid.pid_controller_dt.inited = 0;
	yaw_gyro_pid.last_err = 0;
	yaw_gyro_pid.pre_last_err = 0;
	yaw_gyro_pid.last_expect = 0;
	yaw_gyro_pid.control_output = 0;
	
	pitch_gyro_pid.integrate = 0;
	pitch_gyro_pid.pid_controller_dt.inited = 0;
	pitch_gyro_pid.last_err = 0;
	pitch_gyro_pid.pre_last_err = 0;
	pitch_gyro_pid.last_expect = 0;
	pitch_gyro_pid.control_output = 0;
	
	roll_gyro_pid.integrate = 0;
	roll_gyro_pid.pid_controller_dt.inited = 0;
	roll_gyro_pid.last_err = 0;
	roll_gyro_pid.pre_last_err = 0;
	roll_gyro_pid.last_expect = 0;
	roll_gyro_pid.control_output = 0;
}


/**********************************************************************************************************
*�� �� ��: gyro_control
*����˵��: ���ٶȶȻ�pid����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void gyro_control()
{
	
    //������˹��ͨ��õ���΢����,30hz
    //pitch_gyro_pid.feedback = Butterworth_Filter(gyroDataFilter.x * GYRO_CALIBRATION_COFF, &pitch_pri_dat2, &gyro_filter_parameter_30Hz);
    pitch_gyro_pid.feedback = gyroDataFilter.x * GYRO_CALIBRATION_COFF;
    pitch_gyro_pid.expect = pitch_angle_pid.control_output;
    pid_control(&pitch_gyro_pid);

    //roll_gyro_pid.feedback = Butterworth_Filter(gyroDataFilter.y * GYRO_CALIBRATION_COFF, &roll_pri_dat2, &gyro_filter_parameter_30Hz);
    roll_gyro_pid.feedback = gyroDataFilter.y * GYRO_CALIBRATION_COFF;
    roll_gyro_pid.expect = roll_angle_pid.control_output;
    pid_control(&roll_gyro_pid);

    yaw_gyro_pid.feedback = Butterworth_Filter(gyroDataFilter.z * GYRO_CALIBRATION_COFF, &yaw_pri_dat2, &gyro_filter_parameter_30Hz);
    //yaw_gyro_pid.feedback = gyroDataFilter.z * GYRO_CALIBRATION_COFF;
    yaw_gyro_pid.expect = yaw_angle_pid.control_output;
    pid_control(&yaw_gyro_pid);
}
