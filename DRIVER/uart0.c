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
#include "driverlib/uart.h"

#include "uart0.h"
#include "openmv.h"

#include "FreeRTOS.h"
#include "queue.h"

//���Ͷ���
static QueueHandle_t tx_queue;

void UART0_IRQHandler(void);

/**********************************************************************************************************
*�� �� ��: uart0_init
*����˵��: ����0��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void uart0_init()
{
	//�������Ͷ���
	tx_queue = xQueueCreate(64, sizeof(uint8_t));

	//���ʱ��ʹ��
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//IO��ʼ��
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
	//���ڳ�ʼ��
    ROM_UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    //UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX2_8);
    ROM_UARTTxIntModeSet(UART0_BASE, UART_TXINT_MODE_EOT);

	//�����жϳ�ʼ��
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX);
    ROM_IntPrioritySet(INT_UART0, 4 << 5);
    IntRegister(INT_UART0, UART0_IRQHandler);
    ROM_IntEnable(INT_UART0);
	
	//��������
    ROM_UARTEnable(UART0_BASE);
    ROM_UARTFIFODisable(UART0_BASE);
    
    //��㷢��һ���ֽڣ��Ա���������ж�
    ROM_UARTCharPutNonBlocking(UART0_BASE, 0);
}

static void uart0_rec_callback(uint8_t data)
{
    openmv_rec_callback(data);
}

/**********************************************************************************************************
*�� �� ��: UART0_IRQHandler
*����˵��: ����0�жϺ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void UART0_IRQHandler(void)
{
    uint8_t res;
    //�����ж�
    if (UARTIntStatus(UART0_BASE, true) & UART_INT_TX) {
		BaseType_t xTaskWokenByReceive = pdFALSE;
		//���Ͷ�������������Ҫ����
		if (xQueueReceiveFromISR(tx_queue, (void *) &res, &xTaskWokenByReceive) == pdPASS) {
			ROM_UARTCharPutNonBlocking(UART0_BASE, res);
            //����ж�
            UARTIntClear(UART0_BASE, UART_INT_TX);
		} else {
            //�رշ����ж�
            ROM_UARTIntDisable(UART0_BASE, UART_INT_TX);
        }
        if(xTaskWokenByReceive)
            portYIELD_FROM_ISR(xTaskWokenByReceive);
    //�����ж�
    } else if (UARTIntStatus(UART0_BASE, true) & UART_INT_RX) {
        //��ȡ�����ֽ�
        res = UARTCharGetNonBlocking(UART0_BASE);
        uart0_rec_callback(res);
    }
}

/**********************************************************************************************************
*�� �� ��: uart0_send_data
*����˵��: ����0��������
*��    ��: Ҫ���͵�����
*�� �� ֵ: ��
**********************************************************************************************************/
void uart0_send_data(uint8_t data)
{
    //��������
    xQueueSend(tx_queue, &data, 10);
    //ʹ�ܷ����ж�
    ROM_UARTIntEnable(UART0_BASE, UART_INT_TX);
}
