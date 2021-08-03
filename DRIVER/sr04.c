#include "sr04.h"
#include "time_cnt.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"

//触发引脚
#define TRIG_H GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0);
#define TRIG_L GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0);

//高度原始数据
uint16_t sr04_distance;

void sr04_irq(void);

/**********************************************************************************************************
*函 数 名: sr04_task
*功能说明: 超声波定高任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void sr04_init(void)
{
    //使能PD时钟
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    //设置PD2为输出拉
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0);
    //设置PD1为输入，上拉
    ROM_GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_DIR_MODE_IN);
    //方向为输入
    ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    //PF4配置为下降沿中断
    ROM_GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_RISING_EDGE);
    //给PF组注册一个中断函数
    GPIOIntRegister(GPIO_PORTD_BASE, sr04_irq);
    
    //开启PF4的中断
    ROM_IntPrioritySet(INT_GPIOD, 2 << 5);
	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_1);
	ROM_IntEnable(INT_GPIOD);
	ROM_IntMasterEnable();
}

void sr04_start(void)
{
    //触发开始测量
    TRIG_H;
    vTaskDelay(1);
    TRIG_L;
}

/**********************************************************************************************************
*函 数 名: sr04_irq
*功能说明: 超声波外部中断函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void sr04_irq(void)
{
    static Testime timer;
    static uint8_t wait_fall;

    //接收上升沿
    if (wait_fall == 0) {
        //记录时间
        Get_Time_Period(&timer);
        //设置为下降沿触发中断
        GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_1);
        GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_FALLING_EDGE);
        wait_fall = 1;
    //接收下降沿
    } else {
        //计算时间差
        Get_Time_Period(&timer);
        //计算距离
        sr04_distance = timer.Time_Delta * 0.0340 / 2;
        if (sr04_distance > 200)
            sr04_distance = 0;
        //设置为下降沿触发中断
        GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_1);
        GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_RISING_EDGE);
        wait_fall = 0;
    }
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_1);
}
