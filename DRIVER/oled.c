#include "string.h"
#include "oledfont.h"
#include "oled.h"
#include "stdlib.h"
#include "stdio.h"

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

#define oled_DCout_H  HWREG(GPIO_PORTC_BASE + (GPIO_O_DATA + (GPIO_PIN_7 << 2))) = GPIO_PIN_7 //DC
#define oled_DCout_L  HWREG(GPIO_PORTC_BASE + (GPIO_O_DATA + (GPIO_PIN_7 << 2))) = 0          //DC
#define oled_RSTout_H HWREG(GPIO_PORTC_BASE + (GPIO_O_DATA + (GPIO_PIN_6 << 2))) = GPIO_PIN_6 //RES
#define oled_RSTout_L HWREG(GPIO_PORTC_BASE + (GPIO_O_DATA + (GPIO_PIN_6 << 2))) = 0          //RES
#define oled_MOSI_H   HWREG(GPIO_PORTC_BASE + (GPIO_O_DATA + (GPIO_PIN_5 << 2))) = GPIO_PIN_5 //D1
#define oled_MOSI_L   HWREG(GPIO_PORTC_BASE + (GPIO_O_DATA + (GPIO_PIN_5 << 2))) = 0          //D1
#define oled_SCK_H    HWREG(GPIO_PORTC_BASE + (GPIO_O_DATA + (GPIO_PIN_4 << 2))) = GPIO_PIN_4 //D0
#define oled_SCK_L    HWREG(GPIO_PORTC_BASE + (GPIO_O_DATA + (GPIO_PIN_4 << 2))) = 0          //D0

/**********************************************************************************************************
*�� �� ��: OLED_Init
*����˵��: OLED��ʾ����ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void oled_init(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

	//��λOLED
	oled_RSTout_L;
	vTaskDelay(100);
	oled_RSTout_H;

	oled_w_cmd(0xAE);
	oled_w_cmd(0xD5);
	oled_w_cmd(0x80);
	oled_w_cmd(0xA8);
	oled_w_cmd(0x3F);
	oled_w_cmd(0xD3);
	oled_w_cmd(0x00);
	oled_w_cmd(0x40);
	oled_w_cmd(0x8D);
	//��ʹ���ⲿ��Դ�ṩVCC�������ע�ͣ����������д���
	//oled_w_cmd(0x10);
	oled_w_cmd(0x14);
	oled_w_cmd(0x20);
	oled_w_cmd(0x00);
	oled_w_cmd(0xA1);
	oled_w_cmd(0xA0);
	oled_w_cmd(0xDA);
	oled_w_cmd(0x12);
	oled_w_cmd(0x81);
	oled_w_cmd(0xCF);
	oled_w_cmd(0xd9);
	oled_w_cmd(0xF1);
	oled_w_cmd(0xDB);
	oled_w_cmd(0x40);
	oled_w_cmd(0xA4);
	oled_w_cmd(0xA6);
	//0xa0���ҷ��� 0xa1����
	oled_w_cmd(0xa1);
	//0xc0���·��� 0xc8����
	oled_w_cmd(0xc8);
	oled_cls();
	//��ʼ��ʾ
	oled_w_cmd(0xAF);
}

/**********************************************************************************************************
*�� �� ��: oled_w_dat
*����˵��: дһ������
*��    ��: Ҫд������
*�� �� ֵ: ��
**********************************************************************************************************/
void oled_w_dat(uint8_t dat)
{
	uint8_t i = 8;
	oled_DCout_H;
	for (i = 0; i < 8; i++) {
		oled_SCK_L;
		if (dat & 0x80)
			oled_MOSI_H;
		else
			oled_MOSI_L;
		oled_SCK_H;
		dat <<= 1;
	}
}

/**********************************************************************************************************
*�� �� ��: oled_w_cmd
*����˵��: дһ������
*��    ��: OLED����
*�� �� ֵ: ��
**********************************************************************************************************/
void oled_w_cmd(uint8_t cmd)
{
	uint8_t i = 8;
	oled_DCout_L;
	for (i = 0; i < 8; i++) {
		oled_SCK_L;
		if (cmd & 0x80)
			oled_MOSI_H;
		else
			oled_MOSI_L;
		oled_SCK_H;
		cmd <<= 1;
	}
	oled_DCout_H;
}

/**********************************************************************************************************
*�� �� ��: oled_set_pos
*����˵��: ��������
*��    ��: x���� y����
*�� �� ֵ: ��
**********************************************************************************************************/
void oled_set_pos(uint8_t x, uint8_t y)
{
	oled_w_cmd(0xb0 + y);
	oled_w_cmd(((x & 0xf0) >> 4) | 0x10);
	oled_w_cmd((x & 0x0f) | 0x01);
}

/**********************************************************************************************************
*�� �� ��: oled_cls
*����˵��: ȫ�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void oled_cls()
{
	uint8_t y, x;
	for (y = 0; y < 8; y++) {
		oled_w_cmd(0xb0 + y);
		oled_w_cmd(0x01);
		oled_w_cmd(0x10);
		for (x = 0; x < 128; x++) {
			oled_w_dat(0);
		}
	}
}

/**********************************************************************************************************
*�� �� ��: oled_6x8_str
*����˵��: ��ʾ6X8һ���׼��ASCII�ַ���
*��    ��: x���� y���� Ҫ��ʾ���ַ���
*�� �� ֵ: ��
**********************************************************************************************************/
void oled_6x8_str(uint8_t x, uint8_t y, char ch[])
{
	uint8_t c = 0, i = 0, j = 0;
	while (ch[j] != '\0') {
		c = ch[j] - 32;
		if (x > 126) {
			x = 0;
			y++;
		}
		oled_set_pos(x, y);
		for (i = 0; i < 6; i++) {
			oled_w_dat(F6x8[c][i]);
		}
		x += 6;
		j++;
	}
}

/**********************************************************************************************************
*�� �� ��: oled_6x8_char
*����˵��: ��ʾһ��6X8���ַ�
*��    ��: x���� y���� Ҫ��ʾ���ַ�
*�� �� ֵ: ��
**********************************************************************************************************/
void oled_6x8_char(uint8_t x, uint8_t y, char ucData)
{
	uint8_t i, ucDataTmp;
	ucDataTmp = ucData - 32;
	if (x > 126) {
		x = 0;
		y++;
	}
	oled_set_pos(x, y);
	for (i = 0; i < 6; i++) {
		oled_w_dat(F6x8[ucDataTmp][i]);
	}
}

/**********************************************************************************************************
*�� �� ��: oled_6x8_number
*����˵��: ��ʾ6X8�ĸ�����
*��    ��: x���� y���� λ�� Ҫ��ʾ����
*�� �� ֵ: ��
**********************************************************************************************************/
void oled_6x8_number(uint8_t x, uint8_t y, uint8_t w, float number)
{
    float max;
    char buffer[14];
    char format[7];
    int i;
    
    if (w > 9 || w == 0)
        return;
    
    max = 1;
    for (i = 0; i < w; i++)
        max *= 10;
    
    if (number >= max) {
        oled_6x8_str(x, y, "E");
    } else {
        sprintf(format, "%%%d.3f", w);
        sprintf(buffer, format, number);
        buffer[w] = '\0';
        oled_6x8_str(x, y, buffer);
    }
}

/**********************************************************************************************************
*�� �� ��: oled_8x16_str
*����˵��: ��ʾ8X16һ���׼��ASCII�ַ���
*��    ��: x���� y���� Ҫ��ʾ���ַ���
*�� �� ֵ: ��
**********************************************************************************************************/
void oled_8x16_str(uint8_t x, uint8_t y, char ch[])
{
	uint8_t c = 0, i = 0, j = 0;
	while (ch[j] != '\0') {
		c = ch[j] - 32;
		if (x > 120) {
			x = 0;
			y++;
		}
		oled_set_pos(x, y);
		for (i = 0; i < 8; i++) {
			oled_w_dat(F8X16[c * 16 + i]);
		}
		oled_set_pos(x, y + 1);
		for (i = 0; i < 8; i++) {
			oled_w_dat(F8X16[c * 16 + i + 8]);
		}
		x += 8;
		j++;
	}
}

/**********************************************************************************************************
*�� �� ��: oled_8x16_char
*����˵��: ��ʾһ��8X16���ַ�
*��    ��: x���� y���� Ҫ��ʾ���ַ�
*�� �� ֵ: ��
**********************************************************************************************************/
void oled_8x16_char(uint8_t x, uint8_t y, char ch)
{
	uint8_t c = 0, i = 0, j = 0;
	c = ch - 32;
	if (x > 120) {
		x = 0;
		y++;
	}
	oled_set_pos(x, y);
	for (i = 0; i < 8; i++) {
		oled_w_dat(F8X16[c * 16 + i]);
	}
	oled_set_pos(x, y + 1);
	for (i = 0; i < 8; i++) {
		oled_w_dat(F8X16[c * 16 + i + 8]);
	}
	x += 8;
	j++;
}

/**********************************************************************************************************
*�� �� ��: oled_8x16_number
*����˵��: ��ʾ8X16�ĸ�����
*��    ��: x���� y���� λ�� Ҫ��ʾ����
*�� �� ֵ: ��
**********************************************************************************************************/
void oled_8x16_number(uint8_t x, uint8_t y, uint8_t w, float number)
{
    float max;
    char buffer[14];
    char format[7];
    int i;
    
    if (w > 9 || w == 0)
        return;
    
    max = 1;
    for (i = 0; i < w; i++)
        max *= 10;
    
    if (number >= max) {
        oled_8x16_str(x, y, "E");
    } else {
        sprintf(format, "%%%d.3f", w);
        sprintf(buffer, format, number);
        buffer[w] = '\0';
        oled_8x16_str(x, y, buffer);
    }
}

/**********************************************************************************************************
*�� �� ��: oled_clear_line
*����˵��: ���x��y��������ݣ��߶�8
*��    ��: x���� y����
*�� �� ֵ: ��
**********************************************************************************************************/
void oled_clear_line(uint8_t x, uint8_t y)
{
	oled_w_cmd(0xb0 + y);
	oled_w_cmd(0x01);
	oled_w_cmd(0x10);
	oled_set_pos(x, y);
	for (; x < 128; x++) {
		oled_w_dat(0);
	}
}
