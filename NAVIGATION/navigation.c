#include "navigation.h"
#include "ahrs_aux.h"
#include "vector3.h"
#include "Filter.h"
#include "imu.h"
#include "math.h"
#include "time_cnt.h"
#include "vl53l1x.h"
#include "optical_flow_task.h"

#include "FreeRTOS.h"
#include "task.h"

#define AcceGravity 9.80665f

//惯性导航加速度
Vector3f_t navigation_acce;
//惯性导航加速度模长
float navigation_acce_length;

//用于惯性导航的加速度滤波数据
static Butter_BufferData Butter_Buffer_Navigation[3];
//用于惯性导航的加速度滤波参数
static Butter_Parameter Butter_15HZ_Parameter_Navigation;

static Butter_Parameter tof_err_filter_prarameter;
static Butter_Parameter optical_err_filter_prarameter;
static Butter_BufferData tof_err_filter_data[2];
static Butter_BufferData optical_err_filter_data[4];

float pos_x, pos_y, pos_z;
float speed_x, speed_y, speed_z;
float acce_x, acce_y, acce_z;
static float filter_weight_speed = 0.03;
static float filter_weight_pos = 0.01;

/**********************************************************************************************************
*函 数 名: navigation_init
*功能说明: 惯性导航参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void navigation_init(void)
{
    //初始化滤波器参数
	Set_Cutoff_Frequency(Sampling_Freq, 15, &Butter_15HZ_Parameter_Navigation);
	Set_Cutoff_Frequency(Sampling_Freq, 2.5, &optical_err_filter_prarameter);
	Set_Cutoff_Frequency(Sampling_Freq, 1.4, &tof_err_filter_prarameter);
}

/**********************************************************************************************************
*函 数 名: navigation_prepare
*功能说明: 惯性导航准备，计算用于惯性导航的加速度
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void navigation_prepare(void)
{
	Vector3f_t Body_Frame, Earth_Frame;
	
    //导航加速度低通滤波
	Body_Frame.x = Butterworth_Filter(accDataFilter.x, &Butter_Buffer_Navigation[0], &Butter_15HZ_Parameter_Navigation);
	Body_Frame.y = Butterworth_Filter(accDataFilter.y, &Butter_Buffer_Navigation[1], &Butter_15HZ_Parameter_Navigation);
	Body_Frame.z = Butterworth_Filter(accDataFilter.z, &Butter_Buffer_Navigation[2], &Butter_15HZ_Parameter_Navigation);
	
    //载体加速度转换到导航加速度
	Vector_From_BodyFrame2EarthFrame(&Body_Frame, &Earth_Frame);
	navigation_acce = Earth_Frame;

    //转为m/s2
    navigation_acce.z -= 0xA0;
	navigation_acce.z *= ACCEL_SCALE;
	//减去重力加速度
	navigation_acce.z -= AcceGravity;
	//转换为加速度cm/s^2
	navigation_acce.z *= 100;

    //转为m/s2
	navigation_acce.x *= ACCEL_SCALE;
	//转换为加速度cm/s^2
	navigation_acce.x *= 100;

    //转为m/s2
	navigation_acce.y *= ACCEL_SCALE;
	//转换为加速度cm/s^2
	navigation_acce.y *= 100;

    //计算导航加速度模长
	navigation_acce_length = sqrt(navigation_acce.z * navigation_acce.z + navigation_acce.x * navigation_acce.x + navigation_acce.y * navigation_acce.y);
}

/**********************************************************************************************************
*函 数 名: high_filter
*功能说明: 高度位置互补滤波
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void high_filter()
{
    static float acc_correction;
    static float last_acce_z;
    float dt = 0.005f;
    
    //误差
    float tof_speed_err_z = high_speed_raw_data - Butterworth_Filter(speed_z, &tof_err_filter_data[0], &tof_err_filter_prarameter);
    float tof_pos_err_z = high_raw_data - Butterworth_Filter(pos_z, &tof_err_filter_data[1], &tof_err_filter_prarameter);
    
    acc_correction += tof_speed_err_z * 0.01f;
    
    last_acce_z = acce_z;
    acce_z = navigation_acce.z + acc_correction;
    speed_z += (last_acce_z + acce_z) / 2.0f * dt + 0.03f * tof_speed_err_z;
    pos_z += speed_z * dt + 0.5f * (last_acce_z + acce_z) / 2.0f * dt * dt + 0.01f * tof_pos_err_z;   
}

/**********************************************************************************************************
*函 数 名: pos_filter
*功能说明: 水平位置互补滤波
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void pos_filter(void)
{
    float dt = 0.005f;
    //误差
    float optical_flow_speed_err_x = optical_flow_speed_x - Butterworth_Filter(speed_x, &optical_err_filter_data[0], &optical_err_filter_prarameter);
    float optical_flow_speed_err_y = optical_flow_speed_y - Butterworth_Filter(speed_y, &optical_err_filter_data[1], &optical_err_filter_prarameter);
    float optical_flow_pos_err_x = optical_flow_pos_x - Butterworth_Filter(pos_x, &optical_err_filter_data[2], &optical_err_filter_prarameter);
    float optical_flow_pos_err_y = optical_flow_pos_y - Butterworth_Filter(pos_y, &optical_err_filter_data[3], &optical_err_filter_prarameter);
    
    //互补滤波
    acce_x = navigation_acce.x;
    acce_y = navigation_acce.y;
    speed_x += acce_x * dt + filter_weight_speed * optical_flow_speed_err_x;
    speed_y += acce_y * dt + filter_weight_speed * optical_flow_speed_err_y;
    pos_x += speed_x * dt + 0.5f * acce_x * dt * dt + filter_weight_pos * optical_flow_pos_err_x;
    pos_y += speed_y * dt + 0.5f * acce_y * dt * dt + filter_weight_pos * optical_flow_pos_err_y;
}
