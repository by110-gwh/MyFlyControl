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
*�� �� ��: OLED_Init
*����˵��: OLED��ʾ����ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void oled_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	//ʹ��GPIOCʱ��
	__HAL_RCC_GPIOC_CLK_ENABLE();
	//��ʼ��IO����
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	//��λOLED
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
*�� �� ��: oled_w_cmd
*����˵��: дһ������
*��    ��: OLED����
*�� �� ֵ: ��
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
*�� �� ��: oled_6x8_char
*����˵��: ��ʾһ��6X8���ַ�
*��    ��: x���� y���� Ҫ��ʾ���ַ�
*�� �� ֵ: ��
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
*�� �� ��: oled_6x8_number
*����˵��: ��ʾ6X8�ĸ�����
*��    ��: x���� y���� Ҫ��ʾ����
*�� �� ֵ: ��
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
	//С������
	decimal = number - data;

	///�Ƿ��ܱ�10^9����
	if (data >= 1000000000) {
		temp[i] = 48 + data / 1000000000;
		data = data % 1000000000;
		i++;
	}
	//�Ƿ��ܱ�10^8����
	if (data >= 100000000) {
		temp[i] = 48 + data / 100000000;
		data = data % 100000000;
		i++;
	} else if (data < 100000000 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//�Ƿ��ܱ�10^7����
	if (data >= 10000000) {
		temp[i] = 48 + data / 10000000;
		data = data % 10000000;
		i++;
	} else if (data < 10000000 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//�Ƿ��ܱ�10^6����
	if (data >= 1000000) 
	{
		temp[i] = 48 + data / 1000000;
		data = data % 1000000;
		i++;
	} else if (data < 1000000 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//�Ƿ��ܱ�100000����
	if (data >= 100000) {
		temp[i] = 48 + data / 100000;
		data = data % 100000;
		i++;
	} else if (data < 100000 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//�Ƿ��ܱ�10000����
	if (data >= 10000)  {
		temp[i] = 48 + data / 10000;
		data = data % 10000;
		i++;
	} else if (data < 10000 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//�Ƿ��ܱ�1000����
	if (data >= 1000) {
		temp[i] = 48 + data / 1000;
		data = data % 1000;
		i++;
	} else if (data < 1000 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//�Ƿ��ܱ�100����
	if (data >= 100) {
		temp[i] = 48 + data / 100;
		data = data % 100;
		i++;
	} else if (data < 100 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//�Ƿ��ܱ�10����
	if (data >= 10) {
		temp[i] = 48 + data / 10;
		data = data % 10;
		i++;
	} else if (data < 10 && i != 0) {
		temp[i] = 48;
		i++;
	}
	temp[i] = 48 + data;
	//�ж��Ƿ���С������
	if (decimal >= 0.0001) {
		i++;
		//��ʾС����
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
*�� �� ��: oled_8x16_str
*����˵��: ��ʾ8X16һ���׼��ASCII�ַ���
*��    ��: x���� y���� Ҫ��ʾ���ַ���
*�� �� ֵ: ��
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
*�� �� ��: oled_8x16_char
*����˵��: ��ʾһ��8X16���ַ�
*��    ��: x���� y���� Ҫ��ʾ���ַ�
*�� �� ֵ: ��
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
*�� �� ��: oled_8x16_number
*����˵��: ��ʾ8X16�ĸ�����
*��    ��: x���� y���� Ҫ��ʾ����
*�� �� ֵ: ��
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
	//С������
	decimal = number - data;
	//�Ƿ�ɱ�1000����
	if (data >= 1000) {
		temp[i] = 48 + data / 1000;
		data = data % 1000;
		i++;
	}
	//�ɷ�100����
	if (data >= 100) {
		temp[i] = 48 + data / 100;
		data = data % 100;
		i++;
	} else if (data < 100 && i != 0) {
		temp[i] = 0 + 48;
		i++;
	}
	//�ɷ�10����
	if (data >= 10) {
		temp[i] = 48 + data / 10;
		data = data % 10;
		i++;
	} else if (data < 10 && i != 0) {
		temp[i] = 48;
		i++;
	}
	temp[i] = 48 + data;
	//�ж��ǹ�ΪС��
	if (decimal >= 0.0001) {
		i++;
		//��ʾС����
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
