#include "ppm.h"
#include "time_cnt.h"
#include <string.h>
#include "remote_control.h"

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

#include "FreeRTOS.h"
#include "timers.h"

void PPM_IRQHandler(void);

/**********************************************************************************************************
*函 数 名: PPM_Init
*功能说明: PPM初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PPM_Init()
{
    //使能PF时钟
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //设置PF4为输入，上拉（没按就是高电平，按下就是低电平）
    ROM_GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN);
    //方向为输入
    ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    //PF4配置为下降沿中断
    ROM_GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);
    //给PF组注册一个中断函数
    GPIOIntRegister(GPIO_PORTF_BASE, PPM_IRQHandler);
    
    //开启PF4的中断
    ROM_IntPrioritySet(INT_GPIOF, 1 << 5);
	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
	ROM_IntEnable(INT_GPIOF);
	ROM_IntMasterEnable();
}

/**********************************************************************************************************
*函 数 名: PPM_IRQHandler
*功能说明: PPM中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PPM_IRQHandler(void)
{
    static uint16_t PPM_buf[8];
    static Testime ppm_time;
    static uint8_t ppm_sample_cnt;
    uint16_t ppm_time_delta;
    if (GPIOIntStatus(GPIO_PORTF_BASE, false) & GPIO_INT_PIN_4) {
        //系统运行时间获取
        Get_Time_Period(&ppm_time);
        ppm_time_delta = ppm_time.Time_Delta;
        //PPM解析开始
        if (ppm_time_delta >= 2200 || ppm_time_delta == 0) {
            ppm_sample_cnt = 0;
        } else if (ppm_time_delta >= 900 && ppm_time_delta <= 2100) {
            PPM_buf[ppm_sample_cnt++] = ppm_time_delta;
            //单次解析结束
            if (ppm_sample_cnt >= 8) {
                //由于遥控器处理占用时间较多，为防止其他中断不及时，故将遥控器处理函数放在定时器守护进程里执行
                BaseType_t xHigherPriorityTaskWoken;
                xHigherPriorityTaskWoken = pdFALSE;
                xTimerPendFunctionCallFromISR((PendedFunction_t)rc_callback, PPM_buf, 0, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                ppm_sample_cnt = 0;
            }
        }
        GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
    }
}
