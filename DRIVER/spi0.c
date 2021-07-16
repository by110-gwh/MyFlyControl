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

//SPI��Ϣ��־����
static QueueHandle_t spi_queue;
//SPI���ͻ�����
static uint8_t *g_send_buf;
//SPIʣ�෢�ͳ���
static uint8_t g_len;
//SPI���ջ�����
static uint8_t *g_rec_buf;
//SPIʣ����ճ���
static uint8_t g_rlen;

void SPI0_IRQHandle(void);

/**********************************************************************************************************
*�� �� ��: spi0_init
*����˵��: SPI0��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void spi0_init(void)
{
    //ʹ������ʱ��
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //GPIO��ӳ��
    ROM_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    ROM_GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    ROM_GPIOPinConfigure(GPIO_PA4_SSI0RX);
    ROM_GPIOPinConfigure(GPIO_PA5_SSI0TX);
    
    //����GPIO
    ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_2);
    
    //����SPI
    ROM_SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 2000000, 8);
    ROM_SSIEnable(SSI0_BASE);
    
    //ʹ����Ӧ�ж�
    ROM_IntPrioritySet(INT_SSI0, 2 << 5);
    IntRegister(INT_SSI0, SPI0_IRQHandle);
    ROM_IntEnable(INT_SSI0);
    
    //��ʼ����Ϣ����
	spi_queue = xQueueCreate(1, sizeof(uint8_t));
}

/**********************************************************************************************************
*�� �� ��: SPI0_IRQHandle
*����˵��: SPI0�жϷ�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void SPI0_IRQHandle(void)
{
    //���ͻ�������
    if (SSIIntStatus(SSI0_BASE, true) & SSI_TXFF) {
        //�����ֽ�
        if (g_send_buf)
            SSIDataPutNonBlocking(SSI0_BASE, *g_send_buf++);
        else
            SSIDataPutNonBlocking(SSI0_BASE, 0);
        g_len--;
        //������Ϲرշ��Ϳ��ж�
        if (g_len == 0) {
            SSIIntDisable(SSI0_BASE, SSI_TXFF);
        }
    }
    //���հ�������ճ�ʱ
    if (SSIIntStatus(SSI0_BASE, true) & SSI_RXFF ||
        SSIIntStatus(SSI0_BASE, true) & SSI_RXTO) {
        uint32_t rec_data;
        //��������
        while (SSIDataGetNonBlocking(SSI0_BASE, &rec_data)) {
            if (g_rec_buf) {
                *g_rec_buf++ = rec_data & 0xFF;
            }
            g_rlen--;
        }
        //�������
        if (g_rlen == 0) {
            uint8_t data;
            BaseType_t xResult;
            //��֪ͨ����
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            data = SPI_EVENT_SUCCESSFUL;
            xResult = xQueueSendFromISR(spi_queue, &data, &xHigherPriorityTaskWoken);
            if(xResult == pdPASS)
              portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    //�������
    if (SSIIntStatus(SSI0_BASE, true) & SSI_RXOR) {
        uint8_t data;
        BaseType_t xResult;
        //��֪ͨ����
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        data = SPI_EVENT_ERROR;
        xResult = xQueueSendFromISR(spi_queue, &data, &xHigherPriorityTaskWoken);
        if(xResult == pdPASS)
          portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    
}

/**********************************************************************************************************
*�� �� ��: spi0_transmit
*����˵��: SPI0����
*��    ��: �������� �������� ���䳤��
*�� �� ֵ: 0���ɹ� -1��ʧ�� -2��SPI�ǿ���
**********************************************************************************************************/
int spi0_transmit(uint8_t *send_data, uint8_t *rec_data, uint8_t len)
{
    uint8_t res;
    
    ROM_SSIIntEnable(SSI0_BASE, SSI_RXTO);
    ROM_SSIIntEnable(SSI0_BASE, SSI_RXFF);
    ROM_SSIIntEnable(SSI0_BASE, SSI_RXOR);
    
    //��һ�δ��仹δ���
    if (g_rlen)
        return -2;
    
    //������ر�־����������
    g_send_buf = send_data;
    g_rec_buf = rec_data;
    g_len = len;
    g_rlen = len;
    SSIIntEnable(SSI0_BASE, SSI_TXFF);
    
    //�ȴ�SPI�������
    xQueueReceive(spi_queue, (void *) &res, portMAX_DELAY);
	if (res & SPI_EVENT_ERROR)
		return -1;
    return 0;
}

/**********************************************************************************************************
*�� �� ��: spi0_read_write
*����˵��: SPI0��дһ���ֽ�
*��    ��: ��������
*�� �� ֵ: ��������
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
