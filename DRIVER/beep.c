#include "beep.h"

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

/**********************************************************************************************************
*函 数 名: beep_init
*功能说明: 蜂鸣器初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void beep_init(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
    beep_control(0);
}

/**********************************************************************************************************
*函 数 名: beep_on
*功能说明: 蜂鸣器控制
*形    参: 1：蜂鸣 0：关闭
*返 回 值: 无
**********************************************************************************************************/
void beep_control(uint8_t is_on)
{
    if (is_on) {
        ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
    } else {
        ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);
    }
}
