#include "navigation.h"
#include "ahrs_aux.h"
#include "vector3.h"
#include "Filter.h"
#include "imu.h"
#include "math.h"
#include "time_cnt.h"
#include "vl53l1x.h"

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

//高度惯性导航加速度
float high_acce;
//高度惯性导航位置
float high_pos;
//高度惯性导航速度
float high_vel;

/**********************************************************************************************************
*函 数 名: constrain_float
*功能说明: 限制最大最小值
*形    参: 要限制的值 最小值 最大值
*返 回 值: 输出值
**********************************************************************************************************/
static float constrain_float(float amt, float low, float high)
{
	return ((amt) < (low) ? (low) : ((amt) > (high)? (high) : (amt)));
}

/**********************************************************************************************************
*函 数 名: navigation_init
*功能说明: 惯性导航参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void navigation_init(void)
{
    //初始化滤波器参数
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
	
    //导航加速度低通滤波
	Body_Frame.x = Butterworth_Filter(accDataFilter.x, &Butter_Buffer_Navigation[0], &Butter_15HZ_Parameter_Navigation);
	Body_Frame.y = Butterworth_Filter(accDataFilter.y, &Butter_Buffer_Navigation[1], &Butter_15HZ_Parameter_Navigation);
	Body_Frame.z = Butterworth_Filter(accDataFilter.z, &Butter_Buffer_Navigation[2], &Butter_15HZ_Parameter_Navigation);
	
    //载体加速度转换到导航加速度
	Vector_From_BodyFrame2EarthFrame(&Body_Frame, &Earth_Frame);
	navigation_acce = Earth_Frame;

    //转为m/s2
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
*函 数 名: high_kalman_filter
*功能说明: 高度卡尔曼滤波
*形    参: 高度观测量 观测传感器延时量 惯导加速度 
*返 回 值: 无
**********************************************************************************************************/
void high_kalman_filter()
{
    static uint8_t cnt;
    //加速度误差
    static float acce_bias;
    //用于计算时间差的结构体
    static Testime Time_Delta;
    //时间差
    float dt;
    static float pos_history[20];
    //后验协方差
    static float pos_conv[2][2] = {
        {0.18, 0.1},
        {0.1, 0.18}
    };
    //先验协方差
    float prior_conv[2][2];
    //系统过程协方差
    float Q[2] = {5.0e-3f, 6.0e-3f};
    float R = 100;
    //卡尔曼增益矩阵
    float K[2];
    float temp;

    //更新计算时间差
    Get_Time_Period(&Time_Delta);
    dt = Time_Delta.Time_Delta / 1000000.0;

    //先验状态
    high_acce = navigation_acce.z;
    high_acce += acce_bias;
    high_pos += high_vel * dt + (high_acce * dt * dt) / 2.0f;
    high_vel += high_acce * dt;
	
    //当观测值更新时才进行融合
    if(high_raw_data) {
		//先验协方差
		temp = pos_conv[0][1] + pos_conv[1][1] * dt;
		prior_conv[0][0] = pos_conv[0][0] + pos_conv[1][0] * dt + temp * dt + Q[0];
		prior_conv[0][1] = temp;
		prior_conv[1][0] = pos_conv[1][0] + pos_conv[1][1] * dt;;
		prior_conv[1][1] = pos_conv[1][1] + Q[1];
        
		//计算卡尔曼增益
		K[0] = prior_conv[0][0] / (prior_conv[0][0] + R);
		K[1] = prior_conv[1][0] / (prior_conv[0][0] + R);
        
		//融合数据输出
		temp = high_raw_data * Cos_Roll * Cos_Pitch / 10 - pos_history[20 - 1];
		high_pos += K[0] * temp;
		high_vel += K[1] * temp;
		acce_bias += 0.0005f * temp;
		acce_bias = constrain_float(acce_bias, -200, 200);
        
		//更新状态协方差矩阵
		pos_conv[0][0] = (1 - K[0]) * prior_conv[0][0];
		pos_conv[0][1] = (1 - K[0]) * prior_conv[0][1];
		pos_conv[1][0] = prior_conv[1][0] - K[1] * prior_conv[0][0];
		pos_conv[1][1] = prior_conv[1][1] - K[1] * prior_conv[0][1];
	}
    
    //5ms滑动一次
    for(cnt = 20 - 1; cnt > 0; cnt--) {
       pos_history[cnt] = pos_history[cnt - 1];
    }
    pos_history[0] = high_pos;
}
