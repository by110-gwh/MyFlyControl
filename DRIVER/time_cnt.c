#include "time_cnt.h"

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

volatile uint32_t TIME_ISR_CNT;
//ϵͳʱ��
Time_t Time_Sys;

void TIM0_A_IRQHandler(void);

/**********************************************************************************************************
*�� �� ��: Get_Time_Init
*����˵��: ʱ�����ڼ���ģ���ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Get_Time_Init(void)
{
	//ʹ�ܶ�ʱ��ʱ��
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    
	//���ö�ʱ��ģ��Ϊ�����Լ���ģʽ��
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC_UP);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 100 - 1);
	
	//ʹ�ܶ�ʱ���ж�
    ROM_IntPrioritySet(INT_TIMER0A, 0 << 5);
    IntRegister(INT_TIMER0A, TIM0_A_IRQHandler);
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	
	//������ʱ��
	ROM_TimerEnable(TIMER0_BASE,TIMER_A);
}

/**********************************************************************************************************
*�� �� ��: TIM2_IRQHandler
*����˵��: ��ʱ��7�жϺ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void TIM0_A_IRQHandler(void)
{
	static uint16_t Microsecond_Cnt = 0;
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
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

/**********************************************************************************************************
*�� �� ��: Get_Period
*����˵��: ��ȡʱ������
*��    ��: ʱ�����ڽṹ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t aa;
void Get_Time_Period(Testime *Time_Lab)
{
	//�����δ��ʼ��
	if (Time_Lab->inited == 0) {
		Time_Lab->inited = 1;
		Time_Lab->Last_Time = Time_Lab->Now_Time = 10000 * TIME_ISR_CNT + HWREG(TIMER0_BASE + TIMER_O_TAR) / 80;
		Time_Lab->Time_Delta = 0;
	}
	Time_Lab->Last_Time = Time_Lab->Now_Time;
	//��λus
    aa = HWREG(TIMER0_BASE + TIMER_O_TAR);
	Time_Lab->Now_Time = 10000 * TIME_ISR_CNT + aa / 80;
	Time_Lab->Time_Delta = Time_Lab->Now_Time - Time_Lab->Last_Time;
}
