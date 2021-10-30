#include "gyro_control.h"
#include "pid.h"
#include "imu.h"
#include "angle_control.h"
#include "Filter.h"
#include "math.h"
#include "string.h"
#include "navigation.h"
#include "controller.h"

pid_paramer_t pitch_gyro_pid_para = {
    .err_max = 500,
    .integrate_separation_err = 0,
    .integrate_max = 100,
    .kp = 1,
    .ki = 0,
    .kd = 0.225,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 500
};

pid_paramer_t roll_gyro_pid_para = {
    .err_max = 500,
    .integrate_separation_err = 0,
    .integrate_max = 100,
    .kp = 1,
    .ki = 0,
    .kd = 0.225,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 500
};

pid_paramer_t yaw_gyro_pid_para = {
    .err_max = 300,
    .integrate_separation_err = 0,
    .integrate_max = 200,
    .kp = 4,
    .ki = 0,
    .kd = 0,
    .feedforward_kp = 0,
    .feedforward_kd = 0,
    .control_output_limit = 500
};

pid_data_t pitch_gyro_pid_data;
pid_data_t roll_gyro_pid_data;
pid_data_t yaw_gyro_pid_data;

typedef struct {
    Butter_BufferData buffer;
} pitch_roll_err_correct_t;

//额外的pid参数
static pitch_roll_err_correct_t pitch_pri_dat;
static pitch_roll_err_correct_t roll_pri_dat;
static pitch_roll_err_correct_t yaw_pri_dat;

static Butter_BufferData pitch_pri_dat2;
static Butter_BufferData roll_pri_dat2;
static Butter_BufferData yaw_pri_dat2;

static Butter_Parameter gyro_filter_parameter_30Hz;

/**********************************************************************************************************
*函 数 名: pitch_roll_err_correct
*功能说明: 额外的俯仰和偏航角速度pid计算
*形    参: pid控制器结构体
*返 回 值: 无
**********************************************************************************************************/
static void pitch_roll_err_correct(pid_data_t *data, pid_paramer_t *para)
{
    //巴特沃斯低通后得到的微分项,30hz
    data->dis_err = Butterworth_Filter(data->dis_err,
        &((pitch_roll_err_correct_t *)data->pri_data)->buffer,
        &gyro_filter_parameter_30Hz);
}

/**********************************************************************************************************
*函 数 名: gyro_control_pid_set
*功能说明: 角速度环pid调整函数
*形    参: 10p参数 10i参数 10d参数
*返 回 值: 无
**********************************************************************************************************/
void gyro_control_pid_set(uint8_t p, uint8_t i, uint8_t d)
{
    pitch_gyro_pid_para.kp = p / 10.;
    pitch_gyro_pid_para.ki = i / 10.;
    pitch_gyro_pid_para.kd = d / 10.;
	
	gyro_pid_integrate_reset();
}

/**********************************************************************************************************
*函 数 名: gyro_control_init
*功能说明: 角速度度环pid结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void gyro_control_init()
{
    pitch_gyro_pid_data.last_expect = 0;
	pitch_gyro_pid_data.expect = 0;
    pitch_gyro_pid_data.feedback = 0;
	pitch_gyro_pid_data.last_err = 0;
	pitch_gyro_pid_data.pre_last_err = 0;
    pitch_gyro_pid_data.integrate = 0;
    pitch_gyro_pid_data.dis_err = 0;
	pitch_gyro_pid_data.control_output = 0;
	pitch_gyro_pid_data.pid_controller_dt.inited = 0;
    pitch_gyro_pid_data.err_callback = pitch_roll_err_correct;
    pitch_gyro_pid_data.pri_data = &pitch_pri_dat;
    pitch_gyro_pid_data.short_circuit_flag = 0;
    
	roll_gyro_pid_data.last_expect = 0;
	roll_gyro_pid_data.expect = 0;
    roll_gyro_pid_data.feedback = 0;
	roll_gyro_pid_data.last_err = 0;
	roll_gyro_pid_data.pre_last_err = 0;
    roll_gyro_pid_data.integrate = 0;
    roll_gyro_pid_data.dis_err = 0;
	roll_gyro_pid_data.control_output = 0;
	roll_gyro_pid_data.pid_controller_dt.inited = 0;
    roll_gyro_pid_data.err_callback = pitch_roll_err_correct;
    roll_gyro_pid_data.pri_data = &roll_pri_dat;
    roll_gyro_pid_data.short_circuit_flag = 0;
    
	yaw_gyro_pid_data.last_expect = 0;
	yaw_gyro_pid_data.expect = 0;
    yaw_gyro_pid_data.feedback = 0;
	yaw_gyro_pid_data.last_err = 0;
	yaw_gyro_pid_data.pre_last_err = 0;
    yaw_gyro_pid_data.integrate = 0;
    yaw_gyro_pid_data.dis_err = 0;
	yaw_gyro_pid_data.control_output = 0;
	yaw_gyro_pid_data.pid_controller_dt.inited = 0;
    yaw_gyro_pid_data.err_callback = pitch_roll_err_correct;
    yaw_gyro_pid_data.pri_data = &yaw_pri_dat;
    yaw_gyro_pid_data.short_circuit_flag = 0;
	
	Set_Cutoff_Frequency(Sampling_Freq, 30, &gyro_filter_parameter_30Hz);
}

/**********************************************************************************************************
*函 数 名: gyro_pid_integrate_reset
*功能说明: 角速度环pid积分清零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void gyro_pid_integrate_reset()
{
	yaw_gyro_pid_data.integrate = 0;
	yaw_gyro_pid_data.pid_controller_dt.inited = 0;
	yaw_gyro_pid_data.last_err = 0;
	yaw_gyro_pid_data.pre_last_err = 0;
	yaw_gyro_pid_data.last_expect = 0;
	yaw_gyro_pid_data.control_output = 0;
	
	pitch_gyro_pid_data.integrate = 0;
	pitch_gyro_pid_data.pid_controller_dt.inited = 0;
	pitch_gyro_pid_data.last_err = 0;
	pitch_gyro_pid_data.pre_last_err = 0;
	pitch_gyro_pid_data.last_expect = 0;
	pitch_gyro_pid_data.control_output = 0;
	
	roll_gyro_pid_data.integrate = 0;
	roll_gyro_pid_data.pid_controller_dt.inited = 0;
	roll_gyro_pid_data.last_err = 0;
	roll_gyro_pid_data.pre_last_err = 0;
	roll_gyro_pid_data.last_expect = 0;
	roll_gyro_pid_data.control_output = 0;
}


/**********************************************************************************************************
*函 数 名: gyro_control
*功能说明: 角速度度环pid控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void gyro_control()
{
    //只有起飞之后，高度大于30cm，积分才介入
    if (pos_z > 30 || controller_state > 1) {
        roll_gyro_pid_para.ki = 2.5;
        pitch_gyro_pid_para.ki = 2.5;
    } else {
        roll_gyro_pid_para.ki = 0;
        pitch_gyro_pid_para.ki = 0;
    }
	
    //巴特沃斯低通后得到的微分项,30hz
    //pitch_gyro_pid.feedback = Butterworth_Filter(gyroDataFilter.x * GYRO_CALIBRATION_COFF, &pitch_pri_dat2, &gyro_filter_parameter_30Hz);
    pitch_gyro_pid_data.feedback = gyroDataFilter.x * GYRO_CALIBRATION_COFF;
    pitch_gyro_pid_data.expect = pitch_angle_pid_data.control_output;
    pid_control(&pitch_gyro_pid_data, &pitch_gyro_pid_para);

    //roll_gyro_pid.feedback = Butterworth_Filter(gyroDataFilter.y * GYRO_CALIBRATION_COFF, &roll_pri_dat2, &gyro_filter_parameter_30Hz);
    roll_gyro_pid_data.feedback = gyroDataFilter.y * GYRO_CALIBRATION_COFF;
    roll_gyro_pid_data.expect = roll_angle_pid_data.control_output;
    pid_control(&roll_gyro_pid_data, &roll_gyro_pid_para);

    yaw_gyro_pid_data.feedback = Butterworth_Filter(gyroDataFilter.z * GYRO_CALIBRATION_COFF, &yaw_pri_dat2, &gyro_filter_parameter_30Hz);
    //yaw_gyro_pid.feedback = gyroDataFilter.z * GYRO_CALIBRATION_COFF;
    yaw_gyro_pid_data.expect = yaw_angle_pid_data.control_output;
    pid_control(&yaw_gyro_pid_data, &yaw_gyro_pid_para);
}
