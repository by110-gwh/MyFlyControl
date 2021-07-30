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

#include "debug_uart.h"
#include "driverlib/uart.h"
#include <stdio.h>

#include "FreeRTOS.h"
#include "queue.h"

//���Ͷ���
QueueHandle_t tx_queue;

//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
//����״̬���
uint16_t USART_RX_STA;
//���ջ���,���USART_REC_LEN���ֽ�.
uint8_t USART_RX_BUF[USART_REC_LEN];

//���������FreeRTOS��port.c�п����ҵ�
extern uint32_t vPortGetIPSR(void);

void UART2_IRQHandler(void);

/**********************************************************************************************************
*�� �� ��: debug_uart_init
*����˵��: ���Դ��ڳ�ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void debug_uart_init()
{
	//�������Ͷ���
	tx_queue = xQueueCreate(64, sizeof(uint8_t));

	//���ʱ��ʹ��
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	//IO��ʼ��
    GPIOUnlockPin(GPIO_PORTD_BASE, GPIO_PIN_7);
    ROM_GPIOPinConfigure(GPIO_PD6_U2RX);
    ROM_GPIOPinConfigure(GPIO_PD7_U2TX);
    ROM_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    
	//���ڳ�ʼ��
    ROM_UARTClockSourceSet(UART2_BASE, UART_CLOCK_SYSTEM);
    ROM_UARTConfigSetExpClk(UART2_BASE, ROM_SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    //UARTFIFOLevelSet(UART2_BASE, UART_FIFO_TX1_8, UART_FIFO_RX2_8);
    ROM_UARTTxIntModeSet(UART2_BASE, UART_TXINT_MODE_EOT);

	//�����жϳ�ʼ��
    ROM_UARTIntEnable(UART2_BASE, UART_INT_RX);
    ROM_IntPrioritySet(INT_UART2, 4 << 5);
    IntRegister(INT_UART2, UART2_IRQHandler);
    ROM_IntEnable(INT_UART2);
	
	//��������
    ROM_UARTEnable(UART2_BASE);
    ROM_UARTFIFODisable(UART2_BASE);
    
    //��㷢��һ���ֽڣ��Ա���������ж�
    ROM_UARTCharPutNonBlocking(UART2_BASE, 0);
}

void UART2_IRQHandler(void)
{
    uint8_t Res;
    //�����ж�
    if (UARTIntStatus(UART2_BASE, true) == UART_INT_TX) {
		BaseType_t xTaskWokenByReceive = pdFALSE;
		//���Ͷ�������������Ҫ����
		if (xQueueReceiveFromISR(tx_queue, (void *) &Res, &xTaskWokenByReceive) == pdPASS) {
			ROM_UARTCharPutNonBlocking(UART2_BASE, Res);
            //����ж�
            UARTIntClear(UART2_BASE, UART_INT_TX);
		} else {
            //�رշ����ж�
            ROM_UARTIntDisable(UART2_BASE, UART_INT_TX);
        }
        if(xTaskWokenByReceive)
            portYIELD_FROM_ISR(xTaskWokenByReceive);
    //�����ж�
    } else if (UARTIntStatus(UART2_BASE, true) == UART_INT_RX) {
        //��ȡ�����ֽ�
        Res = UARTCharGetNonBlocking(UART2_BASE);
        if ((USART_RX_STA & 0x8000) == 0) {
			//���յ���0x0d
			if (USART_RX_STA & 0x4000) {
				if (Res != 0x0a)
					//���մ���,���¿�ʼ
					USART_RX_STA = 0;
				else
					//���������
					USART_RX_STA |= 0x8000;
			//��û�յ�0X0D
			} else {
				if (Res == 0x0d)
					USART_RX_STA |= 0x4000;
				else {
					USART_RX_BUF[USART_RX_STA & 0X3FFF] = Res;
					USART_RX_STA++;
					if (USART_RX_STA > (USART_REC_LEN - 1))
						//�������ݴ���,���¿�ʼ����
						USART_RX_STA = 0;
				}
			}
		}
    }
}

/**********************************************************************************************************
*�� �� ��: fputc
*����˵��: �ض�����printf����
*��    ��: Ҫ���͵��ֽ� �豸�ļ�
*�� �� ֵ: ������ɵ��ֽ�
**********************************************************************************************************/
int fputc(int ch, FILE *f)
{
    if (vPortGetIPSR()) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        //���������д��Ҫ���͵�����
        while (xQueueSendFromISR(tx_queue, &ch, &xHigherPriorityTaskWoken) != pdPASS);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
        //���������д��Ҫ���͵�����
        xQueueSend(tx_queue, &ch, 10);
    }
    //ʹ�ܷ����ж�
    ROM_UARTIntEnable(UART2_BASE, UART_INT_TX);
    return ch;
}
