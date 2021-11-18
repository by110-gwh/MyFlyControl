#include "laser.h"
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

#define LASER_H GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
#define LASER_L GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);

/**********************************************************************************************************
*函 数 名: laser_init
*功能说明: 激光初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void laser_init(void)
{
    //使能PB时钟
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //设置PB2为输出拉
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);
}

void laser_on(void)
{
    LASER_H;
}

void laser_off(void)
{
    LASER_L;
}
