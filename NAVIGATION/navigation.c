#include "navigation.h"
#include "ahrs_aux.h"
#include "vector3.h"
#include "Filter.h"
#include "imu.h"
#include "math.h"

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

/**********************************************************************************************************
*函 数 名: navigation_init
*功能说明: 惯性导航参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void navigation_init(void)
{
	Set_Cutoff_Frequency(Sampling_Freq, 15,&Butter_15HZ_Parameter_Navigation);
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
	
	Body_Frame.x = Butterworth_Filter(accDataFilter.x, &Butter_Buffer_Navigation[0], &Butter_15HZ_Parameter_Navigation);
	Body_Frame.y = Butterworth_Filter(accDataFilter.y, &Butter_Buffer_Navigation[1], &Butter_15HZ_Parameter_Navigation);
	Body_Frame.z = Butterworth_Filter(accDataFilter.z, &Butter_Buffer_Navigation[2], &Butter_15HZ_Parameter_Navigation);
	
	Vector_From_BodyFrame2EarthFrame(&Body_Frame, &Earth_Frame);
	
	navigation_acce = Earth_Frame;

	navigation_acce.z *= ACCEL_SCALE;
	//减去重力加速度
	navigation_acce.z -= AcceGravity;
	//转换为加速度cm/s^2
	navigation_acce.z *= 100;

	navigation_acce.x *= ACCEL_SCALE;
	//转换为加速度cm/s^2
	navigation_acce.x *= 100;

	navigation_acce.y *= ACCEL_SCALE;
	//转换为加速度cm/s^2
	navigation_acce.y *= 100;

	navigation_acce_length = sqrt(navigation_acce.z * navigation_acce.z + navigation_acce.x * navigation_acce.x + navigation_acce.y * navigation_acce.y);
}
