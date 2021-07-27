#include "pwm.h"

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

//最大PWM值 400Hz
#define MAX_PWM 25000

/**********************************************************************************************************
*函 数 名: pwm_init
*功能说明: 电机输出PWM初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void pwm_init()
{
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    
    ROM_GPIOPinConfigure(GPIO_PB6_M0PWM0);
    ROM_GPIOPinConfigure(GPIO_PB7_M0PWM1);
    ROM_GPIOPinConfigure(GPIO_PB4_M0PWM2);
    ROM_GPIOPinConfigure(GPIO_PB5_M0PWM3);
    
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
    
    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    
    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, MAX_PWM - 1);
    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, MAX_PWM - 1);
    
    ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    pwm_set(1000, 1000, 1000, 1000);
}

/**********************************************************************************************************
*函 数 名: pwm_set
*功能说明: PWM输出设置
*形    参: 通道一PWM值 通道二PWM值 通道三PWM值 通道四PWM值
*返 回 值: 无
**********************************************************************************************************/
void pwm_set(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4)
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pwm1 * 10);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, pwm2 * 10);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwm3 * 10);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, pwm4 * 10);
}
