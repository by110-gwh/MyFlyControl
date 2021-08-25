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

//发送队列
static QueueHandle_t tx_queue;

void UART7_IRQHandler(void);

/**********************************************************************************************************
*函 数 名: uart7_init
*功能说明: 串口7初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void uart7_init()
{
	//创建发送队列
	tx_queue = xQueueCreate(64, sizeof(uint8_t));

	//相关时钟使能
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	//IO初始化
    ROM_GPIOPinConfigure(GPIO_PE0_U7RX);
    ROM_GPIOPinConfigure(GPIO_PE1_U7TX);
    ROM_GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
	//串口初始化
    ROM_UARTClockSourceSet(UART7_BASE, UART_CLOCK_SYSTEM);
    ROM_UARTConfigSetExpClk(UART7_BASE, ROM_SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    //UARTFIFOLevelSet(UART7_BASE, UART_FIFO_TX1_8, UART_FIFO_RX2_8);
    ROM_UARTTxIntModeSet(UART7_BASE, UART_TXINT_MODE_EOT);

	//串口中断初始化
    ROM_UARTIntEnable(UART7_BASE, UART_INT_RX);
    ROM_IntPrioritySet(INT_UART7, 4 << 5);
    IntRegister(INT_UART7, UART7_IRQHandler);
    ROM_IntEnable(INT_UART7);
	
	//启动串口
    ROM_UARTEnable(UART7_BASE);
    ROM_UARTFIFODisable(UART7_BASE);
    
    //随便发送一个字节，以便产生接收中断
    ROM_UARTCharPutNonBlocking(UART7_BASE, 0);
}

static void uart7_rec_callback(uint8_t data)
{
    openmv_rec_callback(data);
}

/**********************************************************************************************************
*函 数 名: UART7_IRQHandler
*功能说明: 串口7中断函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UART7_IRQHandler(void)
{
    uint8_t res;
    //发送中断
    if (UARTIntStatus(UART7_BASE, true) & UART_INT_TX) {
		BaseType_t xTaskWokenByReceive = pdFALSE;
		//发送队列中有数据需要发送
		if (xQueueReceiveFromISR(tx_queue, (void *) &res, &xTaskWokenByReceive) == pdPASS) {
			ROM_UARTCharPutNonBlocking(UART7_BASE, res);
            //清除中断
            UARTIntClear(UART7_BASE, UART_INT_TX);
		} else {
            //关闭发送中断
            ROM_UARTIntDisable(UART7_BASE, UART_INT_TX);
        }
        if(xTaskWokenByReceive)
            portYIELD_FROM_ISR(xTaskWokenByReceive);
    //接收中断
    } else if (UARTIntStatus(UART7_BASE, true) & UART_INT_RX) {
        //读取接收字节
        res = UARTCharGetNonBlocking(UART7_BASE);
        uart7_rec_callback(res);
    }
}

/**********************************************************************************************************
*函 数 名: uart7_send_data
*功能说明: 串口7发送数据
*形    参: 要发送的数据
*返 回 值: 无
**********************************************************************************************************/
void uart7_send_data(uint8_t data)
{
    //发送数据
    xQueueSend(tx_queue, &data, 10);
    //使能发送中断
    ROM_UARTIntEnable(UART2_BASE, UART_INT_TX);
}
