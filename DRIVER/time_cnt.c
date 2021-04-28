#include "time_cnt.h"
#include "stm32f1xx_hal.h"

static TIM_HandleTypeDef htim7;
volatile uint32_t TIME_ISR_CNT;
//系统时间
Time_t Time_Sys;

/**********************************************************************************************************
*函 数 名: Get_Time_Init
*功能说明: 时间周期计数模块初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Get_Time_Init(void)
{
	//使能定时器时钟
	__HAL_RCC_TIM7_CLK_ENABLE();
	
	//定时器初始化
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 72-1;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 10000-1;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim7);
	
	//使能定时器中断
	HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
	
	//启动定时器
	HAL_TIM_Base_Start_IT(&htim7);
}

/**********************************************************************************************************
*函 数 名: TIM2_IRQHandler
*功能说明: 定时器7中断函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void TIM7_IRQHandler(void)
{
	static uint16_t Microsecond_Cnt = 0;
	if (__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET) {
		//每10ms自加
		TIME_ISR_CNT++;
		Microsecond_Cnt += 10;
		//1秒
		if (Microsecond_Cnt >= 1000) {
			Microsecond_Cnt = 0;
			Time_Sys.second++;
			//1分钟
			if (Time_Sys.second >= 60) {
				Time_Sys.second = 0;
				Time_Sys.minute++;
				//1小时
				if (Time_Sys.minute >= 60) {
					Time_Sys.minute = 0;
					Time_Sys.hour++;
				}
			}
		}
		Time_Sys.microsecond = Microsecond_Cnt;
		__HAL_TIM_CLEAR_IT(&htim7, TIM_FLAG_UPDATE);
	}
}

/**********************************************************************************************************
*函 数 名: Get_Period
*功能说明: 获取时间周期
*形    参: 时间周期结构体
*返 回 值: 无
**********************************************************************************************************/
void Get_Time_Period(Testime *Time_Lab)
{
	//如果还未初始化
	if (Time_Lab->inited == 0) {
		Time_Lab->inited = 1;
		Time_Lab->Last_Time = Time_Lab->Now_Time = 10000 * TIME_ISR_CNT + TIM7->CNT;
		Time_Lab->Time_Delta = 0;
		Time_Lab->Time_Delta_INT = 0;
	}
	Time_Lab->Last_Time = Time_Lab->Now_Time;
	//单位us
	Time_Lab->Now_Time = 10000 * TIME_ISR_CNT + TIM7->CNT;
	Time_Lab->Time_Delta = Time_Lab->Now_Time - Time_Lab->Last_Time;
	Time_Lab->Time_Delta_INT = (uint16_t)(Time_Lab->Time_Delta);
}
