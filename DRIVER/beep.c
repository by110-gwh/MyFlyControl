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
*�� �� ��: beep_init
*����˵��: ��������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void beep_init(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
    beep_control(0);
}

/**********************************************************************************************************
*�� �� ��: beep_on
*����˵��: ����������
*��    ��: 1������ 0���ر�
*�� �� ֵ: ��
**********************************************************************************************************/
void beep_control(uint8_t is_on)
{
    if (is_on) {
        ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
    } else {
        ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);
    }
}
