#include "time_cnt.h"
#include "stm32f1xx_hal.h"

static TIM_HandleTypeDef htim7;
volatile uint32_t TIME_ISR_CNT;
//ϵͳʱ��
Time_t Time_Sys;

/**********************************************************************************************************
*�� �� ��: Get_Time_Init
*����˵��: ʱ�����ڼ���ģ���ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Get_Time_Init(void)
{
	//ʹ�ܶ�ʱ��ʱ��
	__HAL_RCC_TIM7_CLK_ENABLE();
	
	//��ʱ����ʼ��
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 72-1;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 10000-1;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim7);
	
	//ʹ�ܶ�ʱ���ж�
	HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
	
	//������ʱ��
	HAL_TIM_Base_Start_IT(&htim7);
}

/**********************************************************************************************************
*�� �� ��: TIM2_IRQHandler
*����˵��: ��ʱ��7�жϺ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void TIM7_IRQHandler(void)
{
	static uint16_t Microsecond_Cnt = 0;
	if (__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET) {
		//ÿ10ms�Լ�
		TIME_ISR_CNT++;
		Microsecond_Cnt += 10;
		//1��
		if (Microsecond_Cnt >= 1000) {
			Microsecond_Cnt = 0;
			Time_Sys.second++;
			//1����
			if (Time_Sys.second >= 60) {
				Time_Sys.second = 0;
				Time_Sys.minute++;
				//1Сʱ
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
*�� �� ��: Get_Period
*����˵��: ��ȡʱ������
*��    ��: ʱ�����ڽṹ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Get_Time_Period(Testime *Time_Lab)
{
	//�����δ��ʼ��
	if (Time_Lab->inited == 0) {
		Time_Lab->inited = 1;
		Time_Lab->Last_Time = Time_Lab->Now_Time = 10000 * TIME_ISR_CNT + TIM7->CNT;
		Time_Lab->Time_Delta = 0;
		Time_Lab->Time_Delta_INT = 0;
	}
	Time_Lab->Last_Time = Time_Lab->Now_Time;
	//��λus
	Time_Lab->Now_Time = 10000 * TIME_ISR_CNT + TIM7->CNT;
	Time_Lab->Time_Delta = Time_Lab->Now_Time - Time_Lab->Last_Time;
	Time_Lab->Time_Delta_INT = (uint16_t)(Time_Lab->Time_Delta);
}
