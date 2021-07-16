#include "spi0.h"

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_ssi.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"

#include "FreeRTOS.h"
#include "queue.h"

#define SPI_EVENT_SUCCESSFUL 1 << 0
#define SPI_EVENT_ERROR 1 << 1

//SPI消息标志队列
static QueueHandle_t spi_queue;
//SPI发送缓冲区
static uint8_t *g_send_buf;
//SPI剩余发送长度
static uint8_t g_len;
//SPI接收缓冲区
static uint8_t *g_rec_buf;
//SPI剩余接收长度
static uint8_t g_rlen;

void SPI0_IRQHandle(void);

/**********************************************************************************************************
*函 数 名: spi0_init
*功能说明: SPI0初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void spi0_init(void)
{
    //使能外设时钟
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //GPIO重映射
    ROM_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    ROM_GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    ROM_GPIOPinConfigure(GPIO_PA4_SSI0RX);
    ROM_GPIOPinConfigure(GPIO_PA5_SSI0TX);
    
    //配置GPIO
    ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_2);
    
    //配置SPI
    ROM_SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 2000000, 8);
    ROM_SSIEnable(SSI0_BASE);
    
    //使能相应中断
    ROM_IntPrioritySet(INT_SSI0, 2 << 5);
    IntRegister(INT_SSI0, SPI0_IRQHandle);
    ROM_IntEnable(INT_SSI0);
    
    //初始化消息队列
	spi_queue = xQueueCreate(1, sizeof(uint8_t));
}

/**********************************************************************************************************
*函 数 名: SPI0_IRQHandle
*功能说明: SPI0中断服务函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SPI0_IRQHandle(void)
{
    //发送缓冲区空
    if (SSIIntStatus(SSI0_BASE, true) & SSI_TXFF) {
        //发送字节
        if (g_send_buf)
            SSIDataPutNonBlocking(SSI0_BASE, *g_send_buf++);
        else
            SSIDataPutNonBlocking(SSI0_BASE, 0);
        g_len--;
        //发送完毕关闭发送空中断
        if (g_len == 0) {
            SSIIntDisable(SSI0_BASE, SSI_TXFF);
        }
    }
    //接收半满或接收超时
    if (SSIIntStatus(SSI0_BASE, true) & SSI_RXFF ||
        SSIIntStatus(SSI0_BASE, true) & SSI_RXTO) {
        uint32_t rec_data;
        //接收数据
        while (SSIDataGetNonBlocking(SSI0_BASE, &rec_data)) {
            if (g_rec_buf) {
                *g_rec_buf++ = rec_data & 0xFF;
            }
            g_rlen--;
        }
        //接收完成
        if (g_rlen == 0) {
            uint8_t data;
            BaseType_t xResult;
            //发通知错误
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            data = SPI_EVENT_SUCCESSFUL;
            xResult = xQueueSendFromISR(spi_queue, &data, &xHigherPriorityTaskWoken);
            if(xResult == pdPASS)
              portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    //接收溢出
    if (SSIIntStatus(SSI0_BASE, true) & SSI_RXOR) {
        uint8_t data;
        BaseType_t xResult;
        //发通知错误
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        data = SPI_EVENT_ERROR;
        xResult = xQueueSendFromISR(spi_queue, &data, &xHigherPriorityTaskWoken);
        if(xResult == pdPASS)
          portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    
}

/**********************************************************************************************************
*函 数 名: spi0_transmit
*功能说明: SPI0传输
*形    参: 发送内容 接收内容 传输长度
*返 回 值: 0：成功 -1：失败 -2：SPI非空闲
**********************************************************************************************************/
int spi0_transmit(uint8_t *send_data, uint8_t *rec_data, uint8_t len)
{
    uint8_t res;
    
    ROM_SSIIntEnable(SSI0_BASE, SSI_RXTO);
    ROM_SSIIntEnable(SSI0_BASE, SSI_RXFF);
    ROM_SSIIntEnable(SSI0_BASE, SSI_RXOR);
    
    //上一次传输还未完成
    if (g_rlen)
        return -2;
    
    //配置相关标志，开启发送
    g_send_buf = send_data;
    g_rec_buf = rec_data;
    g_len = len;
    g_rlen = len;
    SSIIntEnable(SSI0_BASE, SSI_TXFF);
    
    //等待SPI传输完成
    xQueueReceive(spi_queue, (void *) &res, portMAX_DELAY);
	if (res & SPI_EVENT_ERROR)
		return -1;
    return 0;
}

/**********************************************************************************************************
*函 数 名: spi0_read_write
*功能说明: SPI0读写一个字节
*形    参: 发送内容
*返 回 值: 接收内容
**********************************************************************************************************/
uint8_t spi0_read_write(uint8_t data)
{
    uint32_t rec_data;
    SSIIntDisable(SSI0_BASE, SSI_RXTO);
    SSIIntDisable(SSI0_BASE, SSI_RXFF);
    SSIIntDisable(SSI0_BASE, SSI_RXOR);
    SSIDataPut(SSI0_BASE, data);
    while(SSIBusy(SSI0_BASE));
    SSIDataGet(SSI0_BASE, &rec_data);
    while(SSIBusy(SSI0_BASE));
    return(rec_data & 0xFF);
}
