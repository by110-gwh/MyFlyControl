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

//发送队列
QueueHandle_t tx_queue;

//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
//接收状态标记
uint16_t USART_RX_STA;
//接收缓冲,最大USART_REC_LEN个字节.
uint8_t USART_RX_BUF[USART_REC_LEN];

//这个函数在FreeRTOS的port.c中可以找到
extern uint32_t vPortGetIPSR(void);

void UART2_IRQHandler(void);

/**********************************************************************************************************
*函 数 名: debug_uart_init
*功能说明: 调试串口初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void debug_uart_init()
{
	//创建发送队列
	tx_queue = xQueueCreate(64, sizeof(uint8_t));

	//相关时钟使能
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	//IO初始化
    GPIOUnlockPin(GPIO_PORTD_BASE, GPIO_PIN_7);
    ROM_GPIOPinConfigure(GPIO_PD6_U2RX);
    ROM_GPIOPinConfigure(GPIO_PD7_U2TX);
    ROM_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    
	//串口初始化
    ROM_UARTClockSourceSet(UART2_BASE, UART_CLOCK_SYSTEM);
    ROM_UARTConfigSetExpClk(UART2_BASE, ROM_SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    //UARTFIFOLevelSet(UART2_BASE, UART_FIFO_TX1_8, UART_FIFO_RX2_8);
    ROM_UARTTxIntModeSet(UART2_BASE, UART_TXINT_MODE_EOT);

	//串口中断初始化
    ROM_UARTIntEnable(UART2_BASE, UART_INT_RX);
    ROM_IntPrioritySet(INT_UART2, 4 << 5);
    IntRegister(INT_UART2, UART2_IRQHandler);
    ROM_IntEnable(INT_UART2);
	
	//启动串口
    ROM_UARTEnable(UART2_BASE);
    ROM_UARTFIFODisable(UART2_BASE);
    
    //随便发送一个字节，以便产生接收中断
    ROM_UARTCharPutNonBlocking(UART2_BASE, 0);
}

void UART2_IRQHandler(void)
{
    uint8_t Res;
    //发送中断
    if (UARTIntStatus(UART2_BASE, true) == UART_INT_TX) {
		BaseType_t xTaskWokenByReceive = pdFALSE;
		//发送队列中有数据需要发送
		if (xQueueReceiveFromISR(tx_queue, (void *) &Res, &xTaskWokenByReceive) == pdPASS) {
			ROM_UARTCharPutNonBlocking(UART2_BASE, Res);
            //清除中断
            UARTIntClear(UART2_BASE, UART_INT_TX);
		} else {
            //关闭发送中断
            ROM_UARTIntDisable(UART2_BASE, UART_INT_TX);
        }
        if(xTaskWokenByReceive)
            portYIELD_FROM_ISR(xTaskWokenByReceive);
    //接收中断
    } else if (UARTIntStatus(UART2_BASE, true) == UART_INT_RX) {
        //读取接收字节
        Res = UARTCharGetNonBlocking(UART2_BASE);
        if ((USART_RX_STA & 0x8000) == 0) {
			//接收到了0x0d
			if (USART_RX_STA & 0x4000) {
				if (Res != 0x0a)
					//接收错误,重新开始
					USART_RX_STA = 0;
				else
					//接收完成了
					USART_RX_STA |= 0x8000;
			//还没收到0X0D
			} else {
				if (Res == 0x0d)
					USART_RX_STA |= 0x4000;
				else {
					USART_RX_BUF[USART_RX_STA & 0X3FFF] = Res;
					USART_RX_STA++;
					if (USART_RX_STA > (USART_REC_LEN - 1))
						//接收数据错误,重新开始接收
						USART_RX_STA = 0;
				}
			}
		}
    }
}

/**********************************************************************************************************
*函 数 名: fputc
*功能说明: 重定义至printf函数
*形    参: 要发送的字节 设备文件
*返 回 值: 发送完成的字节
**********************************************************************************************************/
int fputc(int ch, FILE *f)
{
    if (vPortGetIPSR()) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        //向队列里面写入要发送的数据
        while (xQueueSendFromISR(tx_queue, &ch, &xHigherPriorityTaskWoken) != pdPASS);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
        //向队列里面写入要发送的数据
        xQueueSend(tx_queue, &ch, 10);
    }
    //使能发送中断
    ROM_UARTIntEnable(UART2_BASE, UART_INT_TX);
    return ch;
}
