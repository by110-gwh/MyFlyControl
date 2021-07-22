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
*�� �� ��: PPM_Init
*����˵��: PPM��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void PPM_Init()
{
    //ʹ��PFʱ��
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //����PF4Ϊ���룬������û�����Ǹߵ�ƽ�����¾��ǵ͵�ƽ��
    ROM_GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN);
    //����Ϊ����
    ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    //PF4����Ϊ�½����ж�
    ROM_GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);
    //��PF��ע��һ���жϺ���
    GPIOIntRegister(GPIO_PORTF_BASE, PPM_IRQHandler);
    
    //����PF4���ж�
    ROM_IntPrioritySet(INT_GPIOF, 1 << 5);
	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
	ROM_IntEnable(INT_GPIOF);
	ROM_IntMasterEnable();
}

/**********************************************************************************************************
*�� �� ��: PPM_IRQHandler
*����˵��: PPM�ж�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void PPM_IRQHandler(void)
{
    static uint16_t PPM_buf[8];
    static Testime ppm_time;
    static uint8_t ppm_sample_cnt;
    uint16_t ppm_time_delta;
    if (GPIOIntStatus(GPIO_PORTF_BASE, false) & GPIO_INT_PIN_4) {
        //ϵͳ����ʱ���ȡ
        Get_Time_Period(&ppm_time);
        ppm_time_delta = ppm_time.Time_Delta;
        //PPM������ʼ
        if (ppm_time_delta >= 2200 || ppm_time_delta == 0) {
            ppm_sample_cnt = 0;
        } else if (ppm_time_delta >= 900 && ppm_time_delta <= 2100) {
            PPM_buf[ppm_sample_cnt++] = ppm_time_delta;
            //���ν�������
            if (ppm_sample_cnt >= 8) {
                //����ң��������ռ��ʱ��϶࣬Ϊ��ֹ�����жϲ���ʱ���ʽ�ң�������������ڶ�ʱ���ػ�������ִ��
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
