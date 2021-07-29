#include "horizontal_control.h"
#include "pid.h"
#include "navigation.h"
#include "ahrs_aux.h"
#include "angle_control.h"

//�������ٶ�
#define GRAVITY_MSS 9.80665f

#define PI 3.1415926f
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)

pid_paramer_t horizontal_pos_x_pid_para = {
    .err_max = 0,
    .integrate_separation_err = 0,
    .integrate_max = 0,
    .kp = 1.3,
    .ki = 0,
    .kd = 0,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 200
};

pid_paramer_t horizontal_pos_y_pid_para = {
    .err_max = 0,
    .integrate_separation_err = 0,
    .integrate_max = 0,
    .kp = 1.3,
    .ki = 0,
    .kd = 0,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 200
};

pid_paramer_t horizontal_speed_x_pid_para = {
    .err_max = 0,
    .integrate_separation_err = 0,
    .integrate_max = 0,
    .kp = 2.6,
    .ki = 0,
    .kd = 0.1,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 500
};

pid_paramer_t horizontal_speed_y_pid_para = {
    .err_max = 0,
    .integrate_separation_err = 0,
    .integrate_max = 0,
    .kp = 2.6,
    .ki = 0,
    .kd = 0.1,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 500
};

pid_data_t horizontal_pos_x_pid_data;
pid_data_t horizontal_speed_x_pid_data;
pid_data_t horizontal_pos_y_pid_data;
pid_data_t horizontal_speed_y_pid_data;

/**********************************************************************************************************
*�� �� ��: horizontal_pos_control_pid_set
*����˵��: ˮƽλ�û�x����pid��������
*��    ��: 10p���� 10i���� 10d����
*�� �� ֵ: ��
**********************************************************************************************************/
void horizontal_pos_x_control_pid_set(uint8_t p, uint8_t i, uint8_t d)
{
    horizontal_pos_x_pid_para.kp = p / 10.;
    horizontal_pos_x_pid_para.ki = i / 10.;
    horizontal_pos_x_pid_para.kd = d / 10.;
}

/**********************************************************************************************************
*�� �� ��: horizontal_pos_control_pid_set
*����˵��: ˮƽλ�û�x����pid��������
*��    ��: 10p���� 10i���� 10d����
*�� �� ֵ: ��
**********************************************************************************************************/
void horizontal_speed_x_control_pid_set(uint8_t p, uint8_t i, uint8_t d)
{
    horizontal_speed_x_pid_para.kp = p / 10.;
    horizontal_speed_x_pid_para.ki = i / 10.;
    horizontal_speed_x_pid_para.kd = d / 10.;
}

/**********************************************************************************************************
*�� �� ��: horizontal_pos_control_init
*����˵��: ˮƽpid�ṹ���ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void horizontal_control_init()
{
    horizontal_pos_x_pid_data.last_expect = 0;
	horizontal_pos_x_pid_data.expect = 0;
    horizontal_pos_x_pid_data.feedback = 0;
	horizontal_pos_x_pid_data.last_err = 0;
	horizontal_pos_x_pid_data.pre_last_err = 0;
    horizontal_pos_x_pid_data.integrate = 0;
    horizontal_pos_x_pid_data.dis_err = 0;
	horizontal_pos_x_pid_data.control_output = 0;
	horizontal_pos_x_pid_data.pid_controller_dt.inited = 0;
    horizontal_pos_x_pid_data.err_callback = NULL;
    horizontal_pos_x_pid_data.pri_data = NULL;
    horizontal_pos_x_pid_data.short_circuit_flag = 1;
    
    horizontal_speed_x_pid_data.last_expect = 0;
	horizontal_speed_x_pid_data.expect = 0;
    horizontal_speed_x_pid_data.feedback = 0;
	horizontal_speed_x_pid_data.last_err = 0;
	horizontal_speed_x_pid_data.pre_last_err = 0;
    horizontal_speed_x_pid_data.integrate = 0;
    horizontal_speed_x_pid_data.dis_err = 0;
	horizontal_speed_x_pid_data.control_output = 0;
	horizontal_speed_x_pid_data.pid_controller_dt.inited = 0;
    horizontal_speed_x_pid_data.err_callback = NULL;
    horizontal_speed_x_pid_data.pri_data = NULL;
    horizontal_speed_x_pid_data.short_circuit_flag = 0;
    
    horizontal_pos_y_pid_data.last_expect = 0;
	horizontal_pos_y_pid_data.expect = 0;
    horizontal_pos_y_pid_data.feedback = 0;
	horizontal_pos_y_pid_data.last_err = 0;
	horizontal_pos_y_pid_data.pre_last_err = 0;
    horizontal_pos_y_pid_data.integrate = 0;
    horizontal_pos_y_pid_data.dis_err = 0;
	horizontal_pos_y_pid_data.control_output = 0;
	horizontal_pos_y_pid_data.pid_controller_dt.inited = 0;
    horizontal_pos_y_pid_data.err_callback = NULL;
    horizontal_pos_y_pid_data.pri_data = NULL;
    horizontal_pos_y_pid_data.short_circuit_flag = 1;
    
    horizontal_speed_y_pid_data.last_expect = 0;
	horizontal_speed_y_pid_data.expect = 0;
    horizontal_speed_y_pid_data.feedback = 0;
	horizontal_speed_y_pid_data.last_err = 0;
	horizontal_speed_y_pid_data.pre_last_err = 0;
    horizontal_speed_y_pid_data.integrate = 0;
    horizontal_speed_y_pid_data.dis_err = 0;
	horizontal_speed_y_pid_data.control_output = 0;
	horizontal_speed_y_pid_data.pid_controller_dt.inited = 0;
    horizontal_speed_y_pid_data.err_callback = NULL;
    horizontal_speed_y_pid_data.pri_data = NULL;
    horizontal_speed_y_pid_data.short_circuit_flag = 0;
}

/**********************************************************************************************************
*�� �� ��: horizontal_pos_x_pid_integrate_reset
*����˵��: ˮƽλ�û�x����pid��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void horizontal_pos_x_pid_integrate_reset()
{
	horizontal_pos_x_pid_data.integrate = 0;
	horizontal_pos_x_pid_data.pid_controller_dt.inited = 0;
	horizontal_pos_x_pid_data.last_err = 0;
	horizontal_pos_x_pid_data.pre_last_err = 0;
	horizontal_pos_x_pid_data.last_expect = 0;
	horizontal_pos_x_pid_data.control_output = 0;
}

/**********************************************************************************************************
*�� �� ��: horizontal_pos_y_pid_integrate_reset
*����˵��: ˮƽλ�û�y����pid��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void horizontal_pos_y_pid_integrate_reset()
{
	horizontal_pos_y_pid_data.integrate = 0;
	horizontal_pos_y_pid_data.pid_controller_dt.inited = 0;
	horizontal_pos_y_pid_data.last_err = 0;
	horizontal_pos_y_pid_data.pre_last_err = 0;
	horizontal_pos_y_pid_data.last_expect = 0;
	horizontal_pos_y_pid_data.control_output = 0;
}

/**********************************************************************************************************
*�� �� ��: horizontal_speed_x_pid_integrate_reset
*����˵��: ˮƽ�ٶȻ�x����pid��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void horizontal_speed_x_pid_integrate_reset()
{
	horizontal_speed_x_pid_data.integrate = 0;
	horizontal_speed_x_pid_data.pid_controller_dt.inited = 0;
	horizontal_speed_x_pid_data.last_err = 0;
	horizontal_speed_x_pid_data.pre_last_err = 0;
	horizontal_speed_x_pid_data.last_expect = 0;
	horizontal_speed_x_pid_data.control_output = 0;
}

/**********************************************************************************************************
*�� �� ��: horizontal_speed_y_pid_integrate_reset
*����˵��: ˮƽ�ٶȻ�y����pid��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void horizontal_speed_y_pid_integrate_reset()
{
	horizontal_speed_y_pid_data.integrate = 0;
	horizontal_speed_y_pid_data.pid_controller_dt.inited = 0;
	horizontal_speed_y_pid_data.last_err = 0;
	horizontal_speed_y_pid_data.pre_last_err = 0;
	horizontal_speed_y_pid_data.last_expect = 0;
	horizontal_speed_y_pid_data.control_output = 0;
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
*�� �� ��: fast_atan
*����˵��: ����������
*��    ��: x
*�� �� ֵ: atan(x)
**********************************************************************************************************/
static float fast_atan(float v)
{
    float v2 = v*v;
    return (v*(1.6867629106f+v2*0.4378497304f)/(1.6867633134f+v2));
}

/**********************************************************************************************************
*�� �� ��: horizontal_speed_to_angles
*����˵��: ˮƽ�ٶȿ�����������Ƕȿ�����
*��    ��: ˮƽ�ٶ�x��� ˮƽ�ٶ�y��� ������ָ�� �����ָ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void horizontal_speed_to_angles(float horizontal_speed_x, float horizontal_speed_y, volatile float *pitch_angle, volatile float *roll_angle)
{
    float accel_right, accel_forward;
    float lean_angle_max = 30;
    accel_forward = horizontal_speed_y;//cm/s^2
    accel_right = horizontal_speed_x;//cm/s^2

    *pitch_angle = -constrain_float(fast_atan(accel_forward*Cos_Roll/(GRAVITY_MSS*100))*RAD2DEG,-lean_angle_max,lean_angle_max);//pitch
    *roll_angle = constrain_float(fast_atan(accel_right/(GRAVITY_MSS*100))*RAD2DEG,-lean_angle_max,lean_angle_max);//roll
}

/**********************************************************************************************************
*�� �� ��: horizontal_control
*����˵��: �߶Ȼ�pid����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void horizontal_control()
{
    //ˮƽλ�ÿ��Ƽ�����
    static uint16_t horizontal_pos_control_cnt;
    //ˮƽ�ٶȿ��Ƽ�����
    static uint16_t horizontal_speed_control_cnt;
    
    //ˮƽλ�ÿ�������10*5=50ms
    if (horizontal_pos_control_cnt >= 10) {
        //ˮƽλ�÷���
        horizontal_pos_x_pid_data.feedback = pos_x;
        horizontal_pos_y_pid_data.feedback = pos_y;
        //ˮƽλ��pid����
        pid_control(&horizontal_pos_x_pid_data, &horizontal_pos_x_pid_para);
        pid_control(&horizontal_pos_y_pid_data, &horizontal_pos_y_pid_para);
        
        horizontal_pos_control_cnt = 0;
    }
    horizontal_pos_control_cnt++;
    
    
    //ˮƽ�ٶȿ�������4*5=50ms
    if (horizontal_speed_control_cnt >= 4) {
        //ˮƽ�ٶ��ڴ�
        horizontal_speed_x_pid_data.expect = horizontal_pos_x_pid_data.control_output;
        horizontal_speed_y_pid_data.expect = horizontal_pos_y_pid_data.control_output;
        //ˮƽ�ٶȷ���
        horizontal_speed_x_pid_data.feedback = speed_x;
        horizontal_speed_y_pid_data.feedback = speed_y;
        //ˮƽ�ٶ�pid����
        pid_control(&horizontal_speed_x_pid_data, &horizontal_speed_x_pid_para);
        pid_control(&horizontal_speed_y_pid_data, &horizontal_speed_y_pid_para);
        
        horizontal_speed_control_cnt = 0;
    }
    horizontal_speed_control_cnt++;
    
    horizontal_speed_to_angles(horizontal_speed_x_pid_data.control_output, horizontal_speed_y_pid_data.control_output, &pitch_angle_pid_data.expect, &roll_angle_pid_data.expect);
}
