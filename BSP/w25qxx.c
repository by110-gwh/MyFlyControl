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

//W25X系列/Q系列芯片列表
#define W25Q80 	0XEF13 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16
#define W25Q128	0XEF17

//指令表
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
*函 数 名: w25qxx_init
*功能说明: 初始化w25q系列存储芯片
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void w25qxx_init(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
    
    //SPI FLASH不选中
    W25QXX_CS_H
    
    //读取FLASH ID
	w25qxx_read_id();
}  

/**********************************************************************************************************
*函 数 名: w25qxx_read_sr
*功能说明: 读取W25QXX的状态寄存器
*形    参: 无
*返 回 值: 状态寄存器
**********************************************************************************************************/
uint8_t w25qxx_read_sr(void)
{
    uint8_t byte=0;
    //使能器件
    W25QXX_CS_L;
    //发送读取状态寄存器命令    
    spi1_read_write(W25X_ReadStatusReg);
    //读取一个字节
    byte=spi1_read_write(0Xff);
    //取消片选
    W25QXX_CS_H;
    return byte;
} 

/**********************************************************************************************************
*函 数 名: w25qxx_write_sr
*功能说明: 写W25QXX状态寄存器
*形    参: 要写入的状态寄存器
*返 回 值: 无
**********************************************************************************************************/
void w25qxx_write_sr(uint8_t sr)
{
    //使能器件
	W25QXX_CS_L;
    //发送写取状态寄存器命令
	spi1_read_write(W25X_WriteStatusReg);
    //写入一个字节
	spi1_read_write(sr);
    //取消片选
	W25QXX_CS_H;
}

/**********************************************************************************************************
*函 数 名: w25qxx_write_enable
*功能说明: W25QXX写使能	
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void w25qxx_write_enable(void)
{
    //使能器件
    W25QXX_CS_L;
    //发送写使能
    spi1_read_write(W25X_WriteEnable);
    //取消片选
    W25QXX_CS_H;
}

/**********************************************************************************************************
*函 数 名: w25qxx_write_disable
*功能说明: W25QXX写禁止	
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void w25qxx_write_disable(void)
{
    //使能器件
    W25QXX_CS_L;
    //发送写禁止指令
    spi1_read_write(W25X_WriteDisable);
    //取消片选
    W25QXX_CS_H;   	      
}

/**********************************************************************************************************
*函 数 名: w25qxx_read_id
*功能说明: 读取芯片ID
*形    参: 无
*返 回 值: 芯片ID
**********************************************************************************************************/
uint16_t w25qxx_read_id(void)
{
	uint16_t Temp;
	W25QXX_CS_L;
    //发送读取ID命令
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
*函 数 名: w25qxx_read
*功能说明: 在指定地址开始读取指定长度的数据
*形    参: 数据存储区 开始读取的地址 要读取的字节数
*返 回 值: 无
**********************************************************************************************************/
void w25qxx_read(uint8_t *buf, uint32_t addr, uint16_t cnt)   
{
 	uint16_t i;
    //使能器件
	W25QXX_CS_L;                               
    //发送读取命令
    spi1_read_write(W25X_ReadData);
    //发送24bit地址
    spi1_read_write((uint8_t)(addr >> 16));
    spi1_read_write((uint8_t)(addr >> 8));
    spi1_read_write((uint8_t)addr);
    for(i = 0; i < cnt; i++) {
        //循环读数
        buf[i] = spi1_read_write(0XFF);
    }
	W25QXX_CS_H;
}  

/**********************************************************************************************************
*函 数 名: w25qxx_write_page
*功能说明: 在指定地址开始写入最大256字节的数据
*形    参: 数据存储区 开始写入的地址 要写入的字节数
*返 回 值: 无
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
    //循环写数
    for(i = 0; i < cnt; i++)
        spi1_read_write(buf[i]);  
    W25QXX_CS_H;
    w25qxx_wait_busy();
}

/**********************************************************************************************************
*函 数 名: w25qxx_write
*功能说明: 无检验写SPI FLASH
*形    参: 数据存储区 开始写入的地址 要写入的字节数
*返 回 值: 无
**********************************************************************************************************/
void w25qxx_write(uint8_t* buf,uint32_t addr,uint16_t cnt)
{
    uint16_t pageremain;
    //单页剩余的字节数
    pageremain = 256 - addr % 256;
    //不大于256个字节
    if(cnt <= pageremain)
    pageremain=cnt;
    while(1)
    {
        w25qxx_write_page(buf, addr, pageremain);
        //写入结束了
        if(cnt == pageremain)
            break;
        else {
            buf += pageremain;
            addr += pageremain;
            //减去已经写入了的字节数
            cnt -= pageremain;
            //一次可以写入256个字节
            if(cnt > 256)
                pageremain = 256;
            else
                pageremain = cnt;
        }
    }
}

/**********************************************************************************************************
*函 数 名: w25qxx_erase_chip
*功能说明: 擦除整个芯片
*形    参: 无 
*返 回 值: 无
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
*函 数 名: w25qxx_erase_sector
*功能说明: 擦除一个扇区
*形    参: 地址 
*返 回 值: 无
**********************************************************************************************************/
void w25qxx_erase_sector(uint32_t addr)
{
    w25qxx_write_enable();
    w25qxx_wait_busy();
    //使能器件
  	W25QXX_CS_L;
    //发送扇区擦除指令
    spi1_read_write(W25X_SectorErase);
    //发送24bit地址
    spi1_read_write((uint8_t)(addr >> 16));
    spi1_read_write((uint8_t)(addr >> 8));
    spi1_read_write((uint8_t)addr);
    //取消片选
	W25QXX_CS_H;
    w25qxx_wait_busy();   				   //等待擦除完成
}

/**********************************************************************************************************
*函 数 名: w25qxx_wait_busy
*功能说明: 等待空闲
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void w25qxx_wait_busy(void)
{
    //等待BUSY位清空
	while((w25qxx_read_sr() & 0x01) == 0x01);
}

/**********************************************************************************************************
*函 数 名: w25qxx_power_down
*功能说明: 进入掉电模式
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void w25qxx_power_down(void)
{
    //使能器件
    W25QXX_CS_L;
    spi1_read_write(W25X_PowerDown);
    //取消片选
    W25QXX_CS_H;
    //等待TPD
    vTaskDelay(3);
}

/**********************************************************************************************************
*函 数 名: w25qxx_wakeup
*功能说明: 唤醒
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void w25qxx_wakeup(void)
{
    //使能器件
    W25QXX_CS_L;
    spi1_read_write(W25X_ReleasePowerDown);
    //取消片选
    W25QXX_CS_H;
    //等待TRES1
    vTaskDelay(3);
}
