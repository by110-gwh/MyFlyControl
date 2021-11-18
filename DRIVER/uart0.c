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

//发送队列
static QueueHandle_t tx_queue;

void UART0_IRQHandler(void);

/**********************************************************************************************************
*函 数 名: uart0_init
*功能说明: 串口0初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void uart0_init()
{
	//创建发送队列
	tx_queue = xQueueCreate(64, sizeof(uint8_t));

	//相关时钟使能
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//IO初始化
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
	//串口初始化
    ROM_UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    //UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX2_8);
    ROM_UARTTxIntModeSet(UART0_BASE, UART_TXINT_MODE_EOT);

	//串口中断初始化
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX);
    ROM_IntPrioritySet(INT_UART0, 4 << 5);
    IntRegister(INT_UART0, UART0_IRQHandler);
    ROM_IntEnable(INT_UART0);
	
	//启动串口
    ROM_UARTEnable(UART0_BASE);
    ROM_UARTFIFODisable(UART0_BASE);
    
    //随便发送一个字节，以便产生接收中断
    ROM_UARTCharPutNonBlocking(UART0_BASE, 0);
}

static void uart0_rec_callback(uint8_t data)
{
    openmv_rec_callback(data);
}

/**********************************************************************************************************
*函 数 名: UART0_IRQHandler
*功能说明: 串口0中断函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UART0_IRQHandler(void)
{
    uint8_t res;
    //发送中断
    if (UARTIntStatus(UART0_BASE, true) & UART_INT_TX) {
		BaseType_t xTaskWokenByReceive = pdFALSE;
		//发送队列中有数据需要发送
		if (xQueueReceiveFromISR(tx_queue, (void *) &res, &xTaskWokenByReceive) == pdPASS) {
			ROM_UARTCharPutNonBlocking(UART0_BASE, res);
            //清除中断
            UARTIntClear(UART0_BASE, UART_INT_TX);
		} else {
            //关闭发送中断
            ROM_UARTIntDisable(UART0_BASE, UART_INT_TX);
        }
        if(xTaskWokenByReceive)
            portYIELD_FROM_ISR(xTaskWokenByReceive);
    //接收中断
    } else if (UARTIntStatus(UART0_BASE, true) & UART_INT_RX) {
        //读取接收字节
        res = UARTCharGetNonBlocking(UART0_BASE);
        uart0_rec_callback(res);
    }
}

/**********************************************************************************************************
*函 数 名: uart0_send_data
*功能说明: 串口0发送数据
*形    参: 要发送的数据
*返 回 值: 无
**********************************************************************************************************/
void uart0_send_data(uint8_t data)
{
    //发送数据
    xQueueSend(tx_queue, &data, 10);
    //使能发送中断
    ROM_UARTIntEnable(UART0_BASE, UART_INT_TX);
}
