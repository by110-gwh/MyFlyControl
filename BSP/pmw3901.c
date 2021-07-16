#include "pmw3901.h"
#include "spi0.h"

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

#define PMW3901_CS_H GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
#define PMW3901_CS_L GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);

#if defined(__CC_ARM) 
	#pragma anon_unions	/*用于支持结构体联合体*/
#endif

typedef __packed struct motionBurst_s 
{
	__packed union 
	{
		uint8_t motion;
		__packed struct 
		{
			uint8_t frameFrom0    : 1;
			uint8_t runMode       : 2;
			uint8_t reserved1     : 1;
			uint8_t rawFrom0      : 1;
			uint8_t reserved2     : 2;
			uint8_t motionOccured : 1;
		};
	};

	uint8_t observation;
	int16_t deltaX;
	int16_t deltaY;

	uint8_t squal;

	uint8_t rawDataSum;
	uint8_t maxRawData;
	uint8_t minRawData;

	uint16_t shutter;
} pmw3901_motion_t;


/**********************************************************************************************************
*函 数 名: pmw3901_reg_write
*功能说明: PMW3901写寄存器
*形    参: 寄存器 写入寄存器的值
*返 回 值: 无
**********************************************************************************************************/
static void pmw3901_reg_write(uint8_t reg, uint8_t value)
{
	PMW3901_CS_L;
	// 最高位为1 写寄存器
    spi0_read_write(reg | 0x80u);
    spi0_read_write(value);
    ROM_SysCtlDelay(1200);
	PMW3901_CS_H;
}

/**********************************************************************************************************
*函 数 名: pmw3901_reg_read
*功能说明: PMW3901读寄存器
*形    参: 寄存器
*返 回 值: 读出的寄存器的值
**********************************************************************************************************/
static uint8_t pmw3901_reg_read(uint8_t reg)
{
    uint8_t rec_buf;
    
	PMW3901_CS_L;
	// 最高位为0 读寄存器
    spi0_read_write(reg & ~0x80u);
    ROM_SysCtlDelay(900);
    rec_buf = spi0_read_write(0);
    ROM_SysCtlDelay(600);
	PMW3901_CS_H;
    return rec_buf;
}

/**********************************************************************************************************
*函 数 名: pmw3901_reg_init
*功能说明: PMW3901寄存器初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void pmw3901_reg_init(void)
{	
	pmw3901_reg_write(0x7F, 0x00);
	pmw3901_reg_write(0x61, 0xAD);
	pmw3901_reg_write(0x7F, 0x03);
	pmw3901_reg_write(0x40, 0x00);
	pmw3901_reg_write(0x7F, 0x05);
	pmw3901_reg_write(0x41, 0xB3);
	pmw3901_reg_write(0x43, 0xF1);
	pmw3901_reg_write(0x45, 0x14);
	pmw3901_reg_write(0x5B, 0x32);
	pmw3901_reg_write(0x5F, 0x34);
	pmw3901_reg_write(0x7B, 0x08);
	pmw3901_reg_write(0x7F, 0x06);
	pmw3901_reg_write(0x44, 0x1B);
	pmw3901_reg_write(0x40, 0xBF);
	pmw3901_reg_write(0x4E, 0x3F);
	pmw3901_reg_write(0x7F, 0x08);
	pmw3901_reg_write(0x65, 0x20);
	pmw3901_reg_write(0x6A, 0x18);
	pmw3901_reg_write(0x7F, 0x09);
	pmw3901_reg_write(0x4F, 0xAF);
	pmw3901_reg_write(0x5F, 0x40);
	pmw3901_reg_write(0x48, 0x80);
	pmw3901_reg_write(0x49, 0x80);
	pmw3901_reg_write(0x57, 0x77);
	pmw3901_reg_write(0x60, 0x78);
	pmw3901_reg_write(0x61, 0x78);
	pmw3901_reg_write(0x62, 0x08);
	pmw3901_reg_write(0x63, 0x50);
	pmw3901_reg_write(0x7F, 0x0A);
	pmw3901_reg_write(0x45, 0x60);
	pmw3901_reg_write(0x7F, 0x00);
	pmw3901_reg_write(0x4D, 0x11);
	pmw3901_reg_write(0x55, 0x80);
	pmw3901_reg_write(0x74, 0x1F);
	pmw3901_reg_write(0x75, 0x1F);
	pmw3901_reg_write(0x4A, 0x78);
	pmw3901_reg_write(0x4B, 0x78);
	pmw3901_reg_write(0x44, 0x08);
	pmw3901_reg_write(0x45, 0x50);
	pmw3901_reg_write(0x64, 0xFF);
	pmw3901_reg_write(0x65, 0x1F);
	pmw3901_reg_write(0x7F, 0x14);
	pmw3901_reg_write(0x65, 0x67);
	pmw3901_reg_write(0x66, 0x08);
	pmw3901_reg_write(0x63, 0x70);
	pmw3901_reg_write(0x7F, 0x15);
	pmw3901_reg_write(0x48, 0x48);
	pmw3901_reg_write(0x7F, 0x07);
	pmw3901_reg_write(0x41, 0x0D);
	pmw3901_reg_write(0x43, 0x14);
	pmw3901_reg_write(0x4B, 0x0E);
	pmw3901_reg_write(0x45, 0x0F);
	pmw3901_reg_write(0x44, 0x42);
	pmw3901_reg_write(0x4C, 0x80);
	pmw3901_reg_write(0x7F, 0x10);
	pmw3901_reg_write(0x5B, 0x02);
	pmw3901_reg_write(0x7F, 0x07);
	pmw3901_reg_write(0x40, 0x41);
	pmw3901_reg_write(0x70, 0x00);

	vTaskDelay(10); // delay 10ms

	pmw3901_reg_write(0x32, 0x44);
	pmw3901_reg_write(0x7F, 0x07);
	pmw3901_reg_write(0x40, 0x40);
	pmw3901_reg_write(0x7F, 0x06);
	pmw3901_reg_write(0x62, 0xF0);
	pmw3901_reg_write(0x63, 0x00);
	pmw3901_reg_write(0x7F, 0x0D);
	pmw3901_reg_write(0x48, 0xC0);
	pmw3901_reg_write(0x6F, 0xD5);
	pmw3901_reg_write(0x7F, 0x00);
	pmw3901_reg_write(0x5B, 0xA0);
	pmw3901_reg_write(0x4E, 0xA8);
	pmw3901_reg_write(0x5A, 0x50);
	pmw3901_reg_write(0x40, 0x80);
	
//	/*初始化LED_N*/
//	pmw3901_reg_write(0x7F, 0x0E);
//	pmw3901_reg_write(0x72, 0x0F);
//	pmw3901_reg_write(0x7F, 0x00);
}

void pmw3901_init(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
    
    PMW3901_CS_H;
    vTaskDelay(40);
    
	uint8_t chipId = pmw3901_reg_read(0);
	uint8_t invChipId = pmw3901_reg_read(0x5f);
//	printf("Motion chip is: 0x%x\n", chipId);
//	printf("si pihc noitoM: 0x%x\n", invChipId);
    
    // 上电复位
	pmw3901_reg_write(0x3a, 0x5a);
	vTaskDelay(5);

	pmw3901_reg_init();
	vTaskDelay(5);
}

void pmw3901_read_motion(int16_t *dx, int16_t *dy, uint16_t *qual)
{
    pmw3901_motion_t motion;
    
	PMW3901_CS_L;
	// 最高位为0 读寄存器
    spi0_read_write(0x16);
    ROM_SysCtlDelay(900);
    spi0_transmit(0, (uint8_t *)&motion, sizeof(motion));
	PMW3901_CS_H;
    if(motion.minRawData == 0 && motion.maxRawData == 0) {
        *qual = 0;
        *dx = 0;
        *dy = 0;
    } else {
        *qual = motion.squal;
        *dx = -motion.deltaX;
        *dy = motion.deltaY;
    }
}
