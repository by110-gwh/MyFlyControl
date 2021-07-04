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
//系统时间
Time_t Time_Sys;

void TIM0_A_IRQHandler(void);

/**********************************************************************************************************
*函 数 名: Get_Time_Init
*功能说明: 时间周期计数模块初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Get_Time_Init(void)
{
	//使能定时器时钟
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    
	//配置定时器模块为周期性计数模式。
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC_UP);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 100 - 1);
	
	//使能定时器中断
    ROM_IntPrioritySet(INT_TIMER0A, 0 << 5);
    IntRegister(INT_TIMER0A, TIM0_A_IRQHandler);
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	
	//启动定时器
	ROM_TimerEnable(TIMER0_BASE,TIMER_A);
}

/**********************************************************************************************************
*函 数 名: TIM2_IRQHandler
*功能说明: 定时器7中断函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void TIM0_A_IRQHandler(void)
{
	static uint16_t Microsecond_Cnt = 0;
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
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

/**********************************************************************************************************
*函 数 名: Get_Period
*功能说明: 获取时间周期
*形    参: 时间周期结构体
*返 回 值: 无
**********************************************************************************************************/
uint32_t aa;
void Get_Time_Period(Testime *Time_Lab)
{
	//如果还未初始化
	if (Time_Lab->inited == 0) {
		Time_Lab->inited = 1;
		Time_Lab->Last_Time = Time_Lab->Now_Time = 10000 * TIME_ISR_CNT + HWREG(TIMER0_BASE + TIMER_O_TAR) / 80;
		Time_Lab->Time_Delta = 0;
	}
	Time_Lab->Last_Time = Time_Lab->Now_Time;
	//单位us
    aa = HWREG(TIMER0_BASE + TIMER_O_TAR);
	Time_Lab->Now_Time = 10000 * TIME_ISR_CNT + aa / 80;
	Time_Lab->Time_Delta = Time_Lab->Now_Time - Time_Lab->Last_Time;
}
