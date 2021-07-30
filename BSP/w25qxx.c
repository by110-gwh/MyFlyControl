#include "w25qxx.h"
#include "spi1.h"

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

#include "FreeRTOS.h"
#include "task.h"

//W25Xϵ��/Qϵ��оƬ�б�
#define W25Q80 	0XEF13 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16
#define W25Q128	0XEF17

//ָ���
#define W25X_WriteEnable		0x06 
#define W25X_WriteDisable		0x04 
#define W25X_ReadStatusReg		0x05 
#define W25X_WriteStatusReg		0x01 
#define W25X_ReadData			0x03 
#define W25X_FastReadData		0x0B 
#define W25X_FastReadDual		0x3B 
#define W25X_PageProgram		0x02 
#define W25X_BlockErase			0xD8 
#define W25X_SectorErase		0x20 
#define W25X_ChipErase			0xC7 
#define W25X_PowerDown			0xB9 
#define W25X_ReleasePowerDown	0xAB 
#define W25X_DeviceID			0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F 

#define W25QXX_CS_H GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
#define W25QXX_CS_L GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);

/**********************************************************************************************************
*�� �� ��: w25qxx_init
*����˵��: ��ʼ��w25qϵ�д洢оƬ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void w25qxx_init(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
    
    //SPI FLASH��ѡ��
    W25QXX_CS_H
    
    //��ȡFLASH ID
	w25qxx_read_id();
}  

/**********************************************************************************************************
*�� �� ��: w25qxx_read_sr
*����˵��: ��ȡW25QXX��״̬�Ĵ���
*��    ��: ��
*�� �� ֵ: ״̬�Ĵ���
**********************************************************************************************************/
uint8_t w25qxx_read_sr(void)
{
    uint8_t byte=0;
    //ʹ������
    W25QXX_CS_L;
    //���Ͷ�ȡ״̬�Ĵ�������    
    spi1_read_write(W25X_ReadStatusReg);
    //��ȡһ���ֽ�
    byte=spi1_read_write(0Xff);
    //ȡ��Ƭѡ
    W25QXX_CS_H;
    return byte;
} 

/**********************************************************************************************************
*�� �� ��: w25qxx_write_sr
*����˵��: дW25QXX״̬�Ĵ���
*��    ��: Ҫд���״̬�Ĵ���
*�� �� ֵ: ��
**********************************************************************************************************/
void w25qxx_write_sr(uint8_t sr)
{
    //ʹ������
	W25QXX_CS_L;
    //����дȡ״̬�Ĵ�������
	spi1_read_write(W25X_WriteStatusReg);
    //д��һ���ֽ�
	spi1_read_write(sr);
    //ȡ��Ƭѡ
	W25QXX_CS_H;
}

/**********************************************************************************************************
*�� �� ��: w25qxx_write_enable
*����˵��: W25QXXдʹ��	
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void w25qxx_write_enable(void)
{
    //ʹ������
    W25QXX_CS_L;
    //����дʹ��
    spi1_read_write(W25X_WriteEnable);
    //ȡ��Ƭѡ
    W25QXX_CS_H;
}

/**********************************************************************************************************
*�� �� ��: w25qxx_write_disable
*����˵��: W25QXXд��ֹ	
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void w25qxx_write_disable(void)
{
    //ʹ������
    W25QXX_CS_L;
    //����д��ָֹ��
    spi1_read_write(W25X_WriteDisable);
    //ȡ��Ƭѡ
    W25QXX_CS_H;   	      
}

/**********************************************************************************************************
*�� �� ��: w25qxx_read_id
*����˵��: ��ȡоƬID
*��    ��: ��
*�� �� ֵ: оƬID
**********************************************************************************************************/
uint16_t w25qxx_read_id(void)
{
	uint16_t Temp;
	W25QXX_CS_L;
    //���Ͷ�ȡID����
	spi1_read_write(0x90);
	spi1_read_write(0x00);
	spi1_read_write(0x00);
	spi1_read_write(0x00);
	Temp = spi1_read_write(0xFF) << 8;
	Temp |= spi1_read_write(0xFF);
	W25QXX_CS_H;
	return Temp;
}

/**********************************************************************************************************
*�� �� ��: w25qxx_read
*����˵��: ��ָ����ַ��ʼ��ȡָ�����ȵ�����
*��    ��: ���ݴ洢�� ��ʼ��ȡ�ĵ�ַ Ҫ��ȡ���ֽ���
*�� �� ֵ: ��
**********************************************************************************************************/
void w25qxx_read(uint8_t *buf, uint32_t addr, uint16_t cnt)   
{
 	uint16_t i;
    //ʹ������
	W25QXX_CS_L;                               
    //���Ͷ�ȡ����
    spi1_read_write(W25X_ReadData);
    //����24bit��ַ
    spi1_read_write((uint8_t)(addr >> 16));
    spi1_read_write((uint8_t)(addr >> 8));
    spi1_read_write((uint8_t)addr);
    for(i = 0; i < cnt; i++) {
        //ѭ������
        buf[i] = spi1_read_write(0XFF);
    }
	W25QXX_CS_H;
}  

/**********************************************************************************************************
*�� �� ��: w25qxx_write_page
*����˵��: ��ָ����ַ��ʼд�����256�ֽڵ�����
*��    ��: ���ݴ洢�� ��ʼд��ĵ�ַ Ҫд����ֽ���
*�� �� ֵ: ��
**********************************************************************************************************/
void w25qxx_write_page(uint8_t* buf,uint32_t addr,uint16_t cnt)
{
    uint16_t i;  
    w25qxx_write_enable();
    W25QXX_CS_L;
    spi1_read_write(W25X_PageProgram);
    spi1_read_write((uint8_t)(addr >> 16));
    spi1_read_write((uint8_t)(addr >> 8));
    spi1_read_write((uint8_t)addr);
    //ѭ��д��
    for(i = 0; i < cnt; i++)
        spi1_read_write(buf[i]);  
    W25QXX_CS_H;
    w25qxx_wait_busy();
}

/**********************************************************************************************************
*�� �� ��: w25qxx_write
*����˵��: �޼���дSPI FLASH
*��    ��: ���ݴ洢�� ��ʼд��ĵ�ַ Ҫд����ֽ���
*�� �� ֵ: ��
**********************************************************************************************************/
void w25qxx_write(uint8_t* buf,uint32_t addr,uint16_t cnt)
{
    uint16_t pageremain;
    //��ҳʣ����ֽ���
    pageremain = 256 - addr % 256;
    //������256���ֽ�
    if(cnt <= pageremain)
    pageremain=cnt;
    while(1)
    {
        w25qxx_write_page(buf, addr, pageremain);
        //д�������
        if(cnt == pageremain)
            break;
        else {
            buf += pageremain;
            addr += pageremain;
            //��ȥ�Ѿ�д���˵��ֽ���
            cnt -= pageremain;
            //һ�ο���д��256���ֽ�
            if(cnt > 256)
                pageremain = 256;
            else
                pageremain = cnt;
        }
    }
}

/**********************************************************************************************************
*�� �� ��: w25qxx_erase_chip
*����˵��: ��������оƬ
*��    ��: �� 
*�� �� ֵ: ��
**********************************************************************************************************/
void w25qxx_erase_chip(void)
{
    w25qxx_write_enable();
    w25qxx_wait_busy();
    W25QXX_CS_L;
    spi1_read_write(W25X_ChipErase); 
    W25QXX_CS_H;
    w25qxx_wait_busy();
}

/**********************************************************************************************************
*�� �� ��: w25qxx_erase_sector
*����˵��: ����һ������
*��    ��: ��ַ 
*�� �� ֵ: ��
**********************************************************************************************************/
void w25qxx_erase_sector(uint32_t addr)
{
    w25qxx_write_enable();
    w25qxx_wait_busy();
    //ʹ������
  	W25QXX_CS_L;
    //������������ָ��
    spi1_read_write(W25X_SectorErase);
    //����24bit��ַ
    spi1_read_write((uint8_t)(addr >> 16));
    spi1_read_write((uint8_t)(addr >> 8));
    spi1_read_write((uint8_t)addr);
    //ȡ��Ƭѡ
	W25QXX_CS_H;
    w25qxx_wait_busy();   				   //�ȴ��������
}

/**********************************************************************************************************
*�� �� ��: w25qxx_wait_busy
*����˵��: �ȴ�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void w25qxx_wait_busy(void)
{
    //�ȴ�BUSYλ���
	while((w25qxx_read_sr() & 0x01) == 0x01);
}

/**********************************************************************************************************
*�� �� ��: w25qxx_power_down
*����˵��: �������ģʽ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void w25qxx_power_down(void)
{
    //ʹ������
    W25QXX_CS_L;
    spi1_read_write(W25X_PowerDown);
    //ȡ��Ƭѡ
    W25QXX_CS_H;
    //�ȴ�TPD
    vTaskDelay(3);
}

/**********************************************************************************************************
*�� �� ��: w25qxx_wakeup
*����˵��: ����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void w25qxx_wakeup(void)
{
    //ʹ������
    W25QXX_CS_L;
    spi1_read_write(W25X_ReleasePowerDown);
    //ȡ��Ƭѡ
    W25QXX_CS_H;
    //�ȴ�TRES1
    vTaskDelay(3);
}
