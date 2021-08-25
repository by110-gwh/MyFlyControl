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

#include "uart7.h"
#include "openmv.h"

#include "FreeRTOS.h"
#include "queue.h"

//���Ͷ���
static QueueHandle_t tx_queue;

void UART7_IRQHandler(void);

/**********************************************************************************************************
*�� �� ��: uart7_init
*����˵��: ����7��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void uart7_init()
{
	//�������Ͷ���
	tx_queue = xQueueCreate(64, sizeof(uint8_t));

	//���ʱ��ʹ��
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	//IO��ʼ��
    ROM_GPIOPinConfigure(GPIO_PE0_U7RX);
    ROM_GPIOPinConfigure(GPIO_PE1_U7TX);
    ROM_GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
	//���ڳ�ʼ��
    ROM_UARTClockSourceSet(UART7_BASE, UART_CLOCK_SYSTEM);
    ROM_UARTConfigSetExpClk(UART7_BASE, ROM_SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    //UARTFIFOLevelSet(UART7_BASE, UART_FIFO_TX1_8, UART_FIFO_RX2_8);
    ROM_UARTTxIntModeSet(UART7_BASE, UART_TXINT_MODE_EOT);

	//�����жϳ�ʼ��
    ROM_UARTIntEnable(UART7_BASE, UART_INT_RX);
    ROM_IntPrioritySet(INT_UART7, 4 << 5);
    IntRegister(INT_UART7, UART7_IRQHandler);
    ROM_IntEnable(INT_UART7);
	
	//��������
    ROM_UARTEnable(UART7_BASE);
    ROM_UARTFIFODisable(UART7_BASE);
    
    //��㷢��һ���ֽڣ��Ա���������ж�
    ROM_UARTCharPutNonBlocking(UART7_BASE, 0);
}

static void uart7_rec_callback(uint8_t data)
{
    openmv_rec_callback(data);
}

/**********************************************************************************************************
*�� �� ��: UART7_IRQHandler
*����˵��: ����7�жϺ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void UART7_IRQHandler(void)
{
    uint8_t res;
    //�����ж�
    if (UARTIntStatus(UART7_BASE, true) & UART_INT_TX) {
		BaseType_t xTaskWokenByReceive = pdFALSE;
		//���Ͷ�������������Ҫ����
		if (xQueueReceiveFromISR(tx_queue, (void *) &res, &xTaskWokenByReceive) == pdPASS) {
			ROM_UARTCharPutNonBlocking(UART7_BASE, res);
            //����ж�
            UARTIntClear(UART7_BASE, UART_INT_TX);
		} else {
            //�رշ����ж�
            ROM_UARTIntDisable(UART7_BASE, UART_INT_TX);
        }
        if(xTaskWokenByReceive)
            portYIELD_FROM_ISR(xTaskWokenByReceive);
    //�����ж�
    } else if (UARTIntStatus(UART7_BASE, true) & UART_INT_RX) {
        //��ȡ�����ֽ�
        res = UARTCharGetNonBlocking(UART7_BASE);
        uart7_rec_callback(res);
    }
}

/**********************************************************************************************************
*�� �� ��: uart7_send_data
*����˵��: ����7��������
*��    ��: Ҫ���͵�����
*�� �� ֵ: ��
**********************************************************************************************************/
void uart7_send_data(uint8_t data)
{
    //��������
    xQueueSend(tx_queue, &data, 10);
    //ʹ�ܷ����ж�
    ROM_UARTIntEnable(UART2_BASE, UART_INT_TX);
}
