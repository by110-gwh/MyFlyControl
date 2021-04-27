#include "control.h"
#include "pwm.h"
#include "remote_control.h"

#include "FreeRTOS.h"
#include "task.h"

//油门最低限度
#define Thr_Min 1000
//待机油门
#define Thr_Idle 1100

uint16_t throttle_motor_output = 1500;
uint16_t roll_motor_output = 500;
uint16_t pitch_motor_output;
uint16_t yaw_motor_output;

//四个电机输出值
static uint16_t Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4;
//电机已经被停转标志
static uint8_t urgent_stop_flag;

/**********************************************************************************************************
*函 数 名: value_limit
*功能说明: 限定data的最大值和最小值
*形    参: 最小值 最大值 数据
*返 回 值: 限定后的数据
**********************************************************************************************************/
uint16_t value_limit(uint16_t min,uint16_t max,uint16_t data)
{
	if(data >= max)
		data = max;
	else if (data<=min)
		data = min;
	return data;
}

/**********************************************************************************************************
*函 数 名: control_init
*功能说明: 控制器初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void control_init(void)
{
	//PWM初始化
	pwm_init();
	//四个电机停转
	Motor_PWM_1 = Thr_Min;
	Motor_PWM_2 = Thr_Min;
	Motor_PWM_3 = Thr_Min;
	Motor_PWM_4 = Thr_Min;
	pwm_set(Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4);
}

/**********************************************************************************************************
*函 数 名: control_unlock
*功能说明: 控制器解锁过度过程
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void control_unlock(void)
{
	uint8_t idel_cnt;
	
	//停转模式
	if (rc_raw_data[5] > rc_calibration_data[6].middle) {
		urgent_stop_flag = 1;
		Motor_PWM_1 = Thr_Min;
		Motor_PWM_2 = Thr_Min;
		Motor_PWM_3 = Thr_Min;
		Motor_PWM_4 = Thr_Min;
		pwm_set(Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4);
	} else {
		//复位标志位
		urgent_stop_flag = 0;
		//缓慢起转
		for (idel_cnt = 0; idel_cnt <= 100; idel_cnt++) {
			Motor_PWM_1 = Thr_Min + (Thr_Idle - Thr_Min) * (idel_cnt / 100.F);
			Motor_PWM_2 = Thr_Min + (Thr_Idle - Thr_Min) * (idel_cnt / 100.F);
			Motor_PWM_3 = Thr_Min + (Thr_Idle - Thr_Min) * (idel_cnt / 100.F);
			Motor_PWM_4 = Thr_Min + (Thr_Idle - Thr_Min) * (idel_cnt / 100.F);
			pwm_set(Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4);
			vTaskDelay(50);
		}
	}
}

/**********************************************************************************************************
*函 数 名: control_output
*功能说明: 控制器PWM输出
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void control_output(void)
{
	//紧急停机
	if (rc_raw_data[5] > rc_calibration_data[6].middle || urgent_stop_flag == 1) {
		urgent_stop_flag = 1;
		//四个电机停转
		Motor_PWM_1 = Thr_Min;
		Motor_PWM_2 = Thr_Min;
		Motor_PWM_3 = Thr_Min;
		Motor_PWM_4 = Thr_Min;
		pwm_set(Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4);
	} else {
		//计算四个电机输出值
		Motor_PWM_1 = throttle_motor_output - roll_motor_output + pitch_motor_output - yaw_motor_output;
		Motor_PWM_2 = throttle_motor_output + roll_motor_output - pitch_motor_output - yaw_motor_output;
		Motor_PWM_3 = throttle_motor_output + roll_motor_output + pitch_motor_output + yaw_motor_output;
		Motor_PWM_4 = throttle_motor_output - roll_motor_output - pitch_motor_output + yaw_motor_output;
		//总输出限幅
		Motor_PWM_1 = value_limit(Thr_Idle, 2000, Motor_PWM_1);
        Motor_PWM_2 = value_limit(Thr_Idle, 2000, Motor_PWM_2);
        Motor_PWM_3 = value_limit(Thr_Idle, 2000, Motor_PWM_3);
        Motor_PWM_4 = value_limit(Thr_Idle, 2000, Motor_PWM_4);
		pwm_set(Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4);
	}
}

/**********************************************************************************************************
*函 数 名: control_lock
*功能说明: 控制器停机上锁
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void control_lock(void)
{
	Motor_PWM_1 = Thr_Min;
	Motor_PWM_2 = Thr_Min;
	Motor_PWM_3 = Thr_Min;
	Motor_PWM_4 = Thr_Min;
	pwm_set(Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4);
}
