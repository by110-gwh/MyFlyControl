#include "string.h"
#include "oledfont.h"
#include "oled.h"
#include "stdlib.h"
#include "stm32f1xx_hal.h"
#include "bitband.h"

#include "FreeRTOS.h"
#include "task.h"

#define oled_DCout PCout(0)  //DC
#define oled_RSTout PCout(1) //RES
#define oled_MOSI PCout(2)   //D1
#define oled_SCK PCout(3)    //D0

/**********************************************************************************************************
*函 数 名: OLED_Init
*功能说明: OLED显示屏初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void oled_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	//使能GPIOC时钟
	__HAL_RCC_GPIOC_CLK_ENABLE();
	//初始化IO引脚
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	//复位OLED
	oled_RSTout = 0;
	vTaskDelay(100);
	oled_RSTout = 1;

	oled_w_cmd(0xAE);
	oled_w_cmd(0xD5);
	oled_w_cmd(0x80);
	oled_w_cmd(0xA8);
	oled_w_cmd(0x3F);
	oled_w_cmd(0xD3);
	oled_w_cmd(0x00);
	oled_w_cmd(0x40);
	oled_w_cmd(0x8D);
	//若使用外部电源提供VCC则打开下面注释，屏蔽下下行代码
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
	//0xa0左右反置 0xa1正常
	oled_w_cmd(0xa1);
	//0xc0上下反置 0xc8正常
	oled_w_cmd(0xc8);
	oled_cls();
	//开始显示
	oled_w_cmd(0xAF);
}

/**********************************************************************************************************
*函 数 名: oled_w_dat
*功能说明: 写一个数据
*形    参: 要写的数据
*返 回 值: 无
**********************************************************************************************************/
void oled_w_dat(uint8_t dat)
{
	uint8_t i = 8;
	oled_DCout = 1;
	for (i = 0; i < 8; i++) {
		oled_SCK = 0;
		if (dat & 0x80)
			oled_MOSI = 1;
		else
			oled_MOSI = 0;
		oled_SCK = 1;
		dat <<= 1;
	}
}

/**********************************************************************************************************
*函 数 名: oled_w_cmd
*功能说明: 写一个命令
*形    参: OLED命令
*返 回 值: 无
**********************************************************************************************************/
void oled_w_cmd(uint8_t cmd)
{
	uint8_t i = 8;
	oled_DCout = 0;
	for (i = 0; i < 8; i++) {
		oled_SCK = 0;
		if (cmd & 0x80)
			oled_MOSI = 1;
		else
			oled_MOSI = 0;
		oled_SCK = 1;
		cmd <<= 1;
	}
	oled_DCout = 1;
}

/**********************************************************************************************************
*函 数 名: oled_set_pos
*功能说明: 设置坐标
*形    参: x坐标 y坐标
*返 回 值: 无
**********************************************************************************************************/
void oled_set_pos(uint8_t x, uint8_t y)
{
	oled_w_cmd(0xb0 + y);
	oled_w_cmd(((x & 0xf0) >> 4) | 0x10);
	oled_w_cmd((x & 0x0f) | 0x01);
}

/**********************************************************************************************************
*函 数 名: oled_cls
*功能说明: 全屏清除
*形    参: 无
*返 回 值: 无
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
*函 数 名: oled_6x8_str
*功能说明: 显示6X8一组标准的ASCII字符串
*形    参: x坐标 y坐标 要显示的字符串
*返 回 值: 无
**********************************************************************************************************/
void oled_6x8_str(uint8_t x, uint8_t y, uint8_t ch[])
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
*函 数 名: oled_6x8_char
*功能说明: 显示一个6X8的字符
*形    参: x坐标 y坐标 要显示的字符
*返 回 值: 无
**********************************************************************************************************/
void oled_6x8_char(uint8_t x, uint8_t y, uint8_t ucData)
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
*函 数 名: oled_6x8_number
*功能说明: 显示6X8的浮点数
*形    参: x坐标 y坐标 要显示的数
*返 回 值: 无
**********************************************************************************************************/
void oled_6x8_number(uint8_t x, uint8_t y, float number)
{
	uint8_t i = 0;
	uint8_t temp[16];
	uint8_t *point = temp;
	float decimal;
	int data;
	if (number < 0) {
		temp[0] = '-';
		oled_6x8_char(x, y, temp[0]);
		x += 6;
		number = -number;
	}
	data = (int)number;
	//小数部分
	decimal = number - data;

	///是否能被10^9整除
	if (data >= 1000000000) {
		temp[i] = 48 + data / 1000000000;
		data = data % 1000000000;
		i++;
	}
	//是否能被10^8整除
	if (data >= 100000000) {
		temp[i] = 48 + data / 100000000;
		data = data % 100000000;
		i++;
	} else if (data < 100000000 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//是否能被10^7整除
	if (data >= 10000000) {
		temp[i] = 48 + data / 10000000;
		data = data % 10000000;
		i++;
	} else if (data < 10000000 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//是否能被10^6整除
	if (data >= 1000000) 
	{
		temp[i] = 48 + data / 1000000;
		data = data % 1000000;
		i++;
	} else if (data < 1000000 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//是否能被100000整除
	if (data >= 100000) {
		temp[i] = 48 + data / 100000;
		data = data % 100000;
		i++;
	} else if (data < 100000 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//是否能被10000整除
	if (data >= 10000)  {
		temp[i] = 48 + data / 10000;
		data = data % 10000;
		i++;
	} else if (data < 10000 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//是否能被1000整除
	if (data >= 1000) {
		temp[i] = 48 + data / 1000;
		data = data % 1000;
		i++;
	} else if (data < 1000 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//是否能被100整除
	if (data >= 100) {
		temp[i] = 48 + data / 100;
		data = data % 100;
		i++;
	} else if (data < 100 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//是否能被10整除
	if (data >= 10) {
		temp[i] = 48 + data / 10;
		data = data % 10;
		i++;
	} else if (data < 10 && i != 0) {
		temp[i] = 48;
		i++;
	}
	temp[i] = 48 + data;
	//判断是否有小数部分
	if (decimal >= 0.0001) {
		i++;
		//显示小数点
		temp[i] = '.'; 
		i++;
		data = (int)(decimal * 1000);
		temp[i] = 48 + data / 100;
		data = data % 100;
		i++;
		if (data > 0) {
			temp[i] = 48 + data / 10;
			data = data % 10;
		}
		if (data >= 0) {
			i++;
			temp[i] = data + 48;
		}
	}
	i++;
	temp[i] = '\0';
	oled_6x8_str(x, y, point);
}

/**********************************************************************************************************
*函 数 名: oled_8x16_str
*功能说明: 显示8X16一组标准的ASCII字符串
*形    参: x坐标 y坐标 要显示的字符串
*返 回 值: 无
**********************************************************************************************************/
void oled_8x16_str(uint8_t x, uint8_t y, uint8_t ch[])
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
*函 数 名: oled_8x16_char
*功能说明: 显示一个8X16的字符
*形    参: x坐标 y坐标 要显示的字符
*返 回 值: 无
**********************************************************************************************************/
void oled_8x16_char(uint8_t x, uint8_t y, uint8_t ch)
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
*函 数 名: oled_8x16_number
*功能说明: 显示8X16的浮点数
*形    参: x坐标 y坐标 要显示的数
*返 回 值: 无
**********************************************************************************************************/
void oled_8x16_number(uint8_t x, uint8_t y, float number)
{
	uint8_t i = 0;
	uint8_t temp[16];
	uint8_t *point = temp;
	float decimal;
	int data;

	if (number < 0) {
		temp[0] = '-';
		oled_8x16_char(x, y, temp[0]);
		x += 1;
		number = -number;
	}
	data = (int)number;
	//小数部分
	decimal = number - data;
	//是否可被1000整除
	if (data >= 1000) {
		temp[i] = 48 + data / 1000;
		data = data % 1000;
		i++;
	}
	//可否被100整除
	if (data >= 100) {
		temp[i] = 48 + data / 100;
		data = data % 100;
		i++;
	} else if (data < 100 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//可否被10整除
	if (data >= 10) {
		temp[i] = 48 + data / 10;
		data = data % 10;
		i++;
	} else if (data < 10 && i != 0) {
		temp[i] = 48;
		i++;
	}
	temp[i] = 48 + data;
	//判断是够为小数
	if (decimal >= 0.0001) {
		i++;
		//显示小数点
		temp[i] = '.';
		i++;
		data = (int)(decimal * 1000);
		temp[i] = 48 + data / 100;
		data = data % 100;
		i++;
		if (data > 0) {
			temp[i] = 48 + data / 10;
			data = data % 10;
		}
		if (data >= 0) {
			i++;
			temp[i] = data + 48;
		}
	}
	i++;
	temp[i] = '\0';
	oled_8x16_str(x, y, point);
}

/**********************************************************************************************************
*函 数 名: oled_clear_line
*功能说明: 清除x行y后面的内容，高度8
*形    参: x坐标 y坐标
*返 回 值: 无
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
