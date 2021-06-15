#include "remote_control.h"
#include "key.h"
#include "ppm.h"
#include "paramer_save.h"
#include <string.h>
#include "Filter.h"

#include "FreeRTOS.h"
#include "task.h"

//最大俯仰、横滚期望
#define Pit_Rol_Max 20
//最大偏航期望
#define Yaw_Max     200
//油门底部安全死区
#define THROTTLE_BUTTOM_SAFE_DEADBAND 50
//触发时间，单位ms
#define TOUCH_TIME 1000

//遥控器矫正数据
rc_calibration_data_t rc_calibration_data[RC_DEADBAND_CHANNEL];
//遥控器原始数据
uint16_t rc_raw_data[RC_DEADBAND_CHANNEL];
//遥控器滤波参数
static Butter_Parameter Butter_5HZ_Parameter_RC;
//遥控输出期待
uint16_t Throttle_Control;
int16_t Pitch_Control, Roll_Control, Yaw_Control, High_Control;
//遥控器更新标志，用于安全检查
volatile uint8_t remote_control_updata;

/**********************************************************************************************************
*函 数 名: constrain_int16_t
*功能说明: 限制最大最小值
*形    参: 要限制的值 最小值 最大值
*返 回 值: 输出值
**********************************************************************************************************/
int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high)
{
	return ((amt) < (low) ? (low) : ((amt) > (high)? (high) : (amt)));
}

/**********************************************************************************************************
*函 数 名: rc_init
*功能说明: 遥控器初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void rc_init()
{
	uint8_t i;
	Throttle_Control = 1000;
	PPM_Init();
	rc_calibration_data[0].max = paramer_save_data.rc_ch1_max;
	rc_calibration_data[0].min = paramer_save_data.rc_ch1_min;
	rc_calibration_data[1].max = paramer_save_data.rc_ch2_max;
	rc_calibration_data[1].min = paramer_save_data.rc_ch2_min;
	rc_calibration_data[2].max = paramer_save_data.rc_ch3_max;
	rc_calibration_data[2].min = paramer_save_data.rc_ch3_min;
	rc_calibration_data[3].max = paramer_save_data.rc_ch4_max;
	rc_calibration_data[3].min = paramer_save_data.rc_ch4_min;
	rc_calibration_data[4].max = paramer_save_data.rc_ch5_max;
	rc_calibration_data[4].min = paramer_save_data.rc_ch5_min;
	rc_calibration_data[5].max = paramer_save_data.rc_ch6_max;
	rc_calibration_data[5].min = paramer_save_data.rc_ch6_min;
	rc_calibration_data[6].max = paramer_save_data.rc_ch7_max;
	rc_calibration_data[6].min = paramer_save_data.rc_ch7_min;
	rc_calibration_data[7].max = paramer_save_data.rc_ch8_max;
	rc_calibration_data[7].min = paramer_save_data.rc_ch8_min;
	for(i = 0; i < 8; i++) {
		//行程中位
		rc_calibration_data[i].middle = (uint16_t)((rc_calibration_data[i].max + rc_calibration_data[i].min) / 2);
		//设置满量程的百分之RC_DEADBAND_PERCENT为中位死区
		rc_calibration_data[i].deadband = (uint16_t)((rc_calibration_data[i].max - rc_calibration_data[i].min)*RC_DEADBAND_PERCENT);
		rc_calibration_data[i].deadband_top = rc_calibration_data[i].middle + rc_calibration_data[i].deadband / 2;
		rc_calibration_data[i].deadband_buttom = rc_calibration_data[i].middle - rc_calibration_data[i].deadband / 2;
	}
	//初始化遥控器滤波参数
	Set_Cutoff_Frequency(50, 10, &Butter_5HZ_Parameter_RC);
}

/**********************************************************************************************************
*函 数 名: rc_save_paramer
*功能说明: 遥控器校准参数保存
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void rc_save_paramer()
{
	paramer_save_data.rc_ch1_max = rc_calibration_data[0].max;
	paramer_save_data.rc_ch1_min = rc_calibration_data[0].min;
	paramer_save_data.rc_ch2_max = rc_calibration_data[1].max;
	paramer_save_data.rc_ch2_min = rc_calibration_data[1].min;
	paramer_save_data.rc_ch3_max = rc_calibration_data[2].max;
	paramer_save_data.rc_ch3_min = rc_calibration_data[2].min;
	paramer_save_data.rc_ch4_max = rc_calibration_data[3].max;
	paramer_save_data.rc_ch4_min = rc_calibration_data[3].min;
	paramer_save_data.rc_ch5_max = rc_calibration_data[4].max;
	paramer_save_data.rc_ch5_min = rc_calibration_data[4].min;
	paramer_save_data.rc_ch6_max = rc_calibration_data[5].max;
	paramer_save_data.rc_ch6_min = rc_calibration_data[5].min;
	paramer_save_data.rc_ch7_max = rc_calibration_data[6].max;
	paramer_save_data.rc_ch7_min = rc_calibration_data[6].min;
	paramer_save_data.rc_ch8_max = rc_calibration_data[7].max;
	paramer_save_data.rc_ch8_min = rc_calibration_data[7].min;
	write_save_paramer();
}

/**********************************************************************************************************
*函 数 名: rc_calibration_reset
*功能说明: 遥控器较准数据复位
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void rc_calibration_reset()
{
	uint8_t i = 0;
	for(i = 0; i < 8; i++) {
		rc_calibration_data[i].max = RC_RESET_DEFAULT;
		rc_calibration_data[i].min = RC_RESET_DEFAULT;
		rc_raw_data[i] = RC_RESET_DEFAULT;
	}
}

/**********************************************************************************************************
*函 数 名: rc_is_on
*功能说明: 判断遥控器是否正常
*形    参: 无
*返 回 值: 1：正常 0：不正常
**********************************************************************************************************/
uint8_t rc_is_on()
{
	uint8_t i = 0;
	for(i = 0; i < 8; i++) {
		if (rc_raw_data[i] == 0)
			return 0;
	}
	return 1;
}

/**********************************************************************************************************
*函 数 名: rc_callback
*功能说明: 底层提交数据回调函数
*形    参: 底层遥控器数据
*返 回 值: 无
**********************************************************************************************************/
void rc_callback(uint16_t buf[8])
{
	uint16_t rc_data[4];
	uint8_t i=0;
	static Butter_BufferData RC_LPF_Buffer[4];
	memcpy(rc_raw_data, buf, sizeof(rc_raw_data));
	//对前四通道进行滤波
	for(i = 0; i < 4; i++) {
		rc_data[i] = Butterworth_Filter(buf[i], &RC_LPF_Buffer[i], &Butter_5HZ_Parameter_RC);
	}
	//计算缩放后的横滚期待
	if(rc_raw_data[0] <= rc_calibration_data[0].deadband_buttom)
		Roll_Control = (rc_calibration_data[0].deadband_buttom - rc_data[0])
			* Pit_Rol_Max /(rc_calibration_data[0].deadband_buttom - rc_calibration_data[0].min);
	else if(rc_raw_data[0] >= rc_calibration_data[0].deadband_top)
		Roll_Control = -(rc_calibration_data[0].deadband_top - rc_data[0])
			* Pit_Rol_Max /(rc_calibration_data[0].deadband_top - rc_calibration_data[0].max);
	else
		Roll_Control = 0;
	Roll_Control = constrain_int16_t(Roll_Control, -Pit_Rol_Max, Pit_Rol_Max);
	Roll_Control = -Roll_Control;
	//计算缩放后的俯仰期待
	if(rc_raw_data[1] <= rc_calibration_data[1].deadband_buttom)
		Pitch_Control = (rc_calibration_data[1].deadband_buttom - rc_data[1])
			* Pit_Rol_Max /(rc_calibration_data[1].deadband_buttom - rc_calibration_data[1].min);
	else if(rc_raw_data[1] >= rc_calibration_data[1].deadband_top)
		Pitch_Control = -(rc_calibration_data[1].deadband_top - rc_data[1])
			* Pit_Rol_Max /(rc_calibration_data[1].deadband_top - rc_calibration_data[1].max);
	else
		Pitch_Control = 0;
	Pitch_Control = constrain_int16_t(Pitch_Control, -Pit_Rol_Max, Pit_Rol_Max);
	//计算缩放后的油门期待
	//为了安全，油门杆低位死区为Buttom_Safe_Deadband
	Throttle_Control = (rc_data[2] - (rc_calibration_data[2].min + THROTTLE_BUTTOM_SAFE_DEADBAND));
	Throttle_Control = constrain_int16_t(Throttle_Control, 0, 1000);
    //计算缩放后的偏航期待
	if(rc_raw_data[2] <= rc_calibration_data[2].deadband_buttom)
		High_Control = -(rc_calibration_data[2].deadband_buttom - rc_data[2])
			* 40 /(rc_calibration_data[2].deadband_buttom - rc_calibration_data[2].min);
	else if(rc_raw_data[2] >= rc_calibration_data[2].deadband_top)
		High_Control = (rc_calibration_data[2].deadband_top - rc_data[2])
			* 40 /(rc_calibration_data[2].deadband_top - rc_calibration_data[2].max);
	else
		High_Control = 0;
	High_Control = constrain_int16_t(High_Control, -40, 40);
	//计算缩放后的偏航期待
	if(rc_raw_data[3] <= rc_calibration_data[3].deadband_buttom)
		Yaw_Control = (rc_calibration_data[3].deadband_buttom - rc_data[3])
			* 30 /(rc_calibration_data[3].deadband_buttom - rc_calibration_data[3].min);
	else if(rc_raw_data[3] >= rc_calibration_data[3].deadband_top)
		Yaw_Control = -(rc_calibration_data[3].deadband_top - rc_data[3])
			* 30 /(rc_calibration_data[3].deadband_top - rc_calibration_data[3].max);
	else
		Yaw_Control = 0;
	Yaw_Control = constrain_int16_t(Yaw_Control, -Pit_Rol_Max, Pit_Rol_Max);
    //遥控器工作正常
    remote_control_updata = 1;
}

/**********************************************************************************************************
*函 数 名: rc_is_reset
*功能说明: 遥控器方向感摇杆归零检测
*形    参: 无
*返 回 值: 1：已归位 0：未归位
**********************************************************************************************************/
uint8_t rc_direct_is_reset()
{
	if (Throttle_Control == 0 && Pitch_Control == 0 && Roll_Control == 0 && Yaw_Control == 0) {
		return 1;
	}
	return 0;
}

/**********************************************************************************************************
*函 数 名: rc_scan
*功能说明: 遥控器摇杆触发扫描
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint8_t rc_scan(void)
{
	static uint8_t touch_flag;
	//时间记录
	uint32_t press_time;
	//如果四个杆都移到最大或最小值
	if ((rc_raw_data[0] < rc_calibration_data[0].min * 1.1 || rc_raw_data[0] > rc_calibration_data[0].max * 0.9)
		&& (rc_raw_data[1] < rc_calibration_data[1].min * 1.1 || rc_raw_data[1] > rc_calibration_data[1].max * 0.9)
		&& (rc_raw_data[2] < rc_calibration_data[2].min * 1.1 || rc_raw_data[2] > rc_calibration_data[2].max * 0.9)
		&& (rc_raw_data[3] < rc_calibration_data[3].min * 1.1 || rc_raw_data[3] > rc_calibration_data[3].max * 0.9)
		&& touch_flag == 0) {
		press_time = 0;
		touch_flag = 1;
		//四个杆都移到最大或最小值保持一段时间
		while((rc_raw_data[0] < rc_calibration_data[0].min * 1.1 || rc_raw_data[0] > rc_calibration_data[0].max * 0.9)
			&& (rc_raw_data[1] < rc_calibration_data[1].min * 1.1 || rc_raw_data[1] > rc_calibration_data[1].max * 0.9)
			&& (rc_raw_data[2] < rc_calibration_data[2].min * 1.1 || rc_raw_data[2] > rc_calibration_data[2].max * 0.9)
			&& (rc_raw_data[3] < rc_calibration_data[3].min * 1.1 || rc_raw_data[3] > rc_calibration_data[3].max * 0.9)) {
			vTaskDelay(100);
			press_time += 100;
			if (press_time >= TOUCH_TIME) {
				uint8_t ret = 0;
				//判断输出四个杆的方向
				if (rc_raw_data[0] > rc_calibration_data[0].max * 0.9)
					ret |= 1;
				if (rc_raw_data[1] > rc_calibration_data[1].max * 0.9)
					ret |= 2;
				if (rc_raw_data[2] > rc_calibration_data[2].max * 0.9)
					ret |= 4;
				if (rc_raw_data[3] > rc_calibration_data[3].max * 0.9)
					ret |= 8;
				return ret;
			}
		}
	//判断四个杆是否归中
	} else if ((rc_raw_data[0] > rc_calibration_data[0].middle * 0.9 && rc_raw_data[0] < rc_calibration_data[0].middle * 1.1)
		&& (rc_raw_data[1] > rc_calibration_data[1].middle * 0.9 && rc_raw_data[1] < rc_calibration_data[1].middle * 1.1)
		/*&& (rc_raw_data[2] > rc_calibration_data[2].middle * 0.9 && rc_raw_data[2] < rc_calibration_data[2].middle * 1.1)*/
		&& (rc_raw_data[3] > rc_calibration_data[3].middle * 0.9 && rc_raw_data[3] < rc_calibration_data[3].middle * 1.1)) {
		touch_flag = 0;
	}
	return 0;
}

/**********************************************************************************************************
*函 数 名: rc_calibration_task
*功能说明: 遥控器较准进程
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void rc_calibration_task(void)
{
	uint8_t i;
	rc_calibration_reset();
    while (1) {
        uint8_t key;
		key = key_scan();
		if ((key & KEY0) << 1 && key & 1) {
			rc_save_paramer();
			break;
		}
		for(i = 0; i < RC_DEADBAND_CHANNEL; i++) {
			//获取最大行程值
			if(rc_raw_data[i] >= rc_calibration_data[i].max)
				rc_calibration_data[i].max = rc_raw_data[i];
			//获取最小行程值
			if(rc_raw_data[i] <  rc_calibration_data[i].min)
				rc_calibration_data[i].min = rc_raw_data[i];
			//行程中位
			rc_calibration_data[i].middle =
				(uint16_t)((rc_calibration_data[i].max + rc_calibration_data[i].min) / 2);
			//设置满量程的百分之RC_DEADBAND_PERCENT为中位死区
			rc_calibration_data[i].deadband = (uint16_t)
				((rc_calibration_data[i].max - rc_calibration_data[i].min)*RC_DEADBAND_PERCENT);
		}
    }
}

