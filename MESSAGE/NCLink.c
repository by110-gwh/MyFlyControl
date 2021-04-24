#include "NCLink.h"
#include "stdint.h"
#include "ahrs_aux.h"
#include "mpu6050.h"
#include "stm32f1xx_hal.h"
#include "navigation.h"
#include "remote_control.h"

#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

#define NCLINK_STATUS 0x01
#define NCLINK_SENSER 0x02
#define NCLINK_RCDATA 0x03
#define NCLINK_GPS 0x04
#define NCLINK_OBS_NE 0x05
#define NCLINK_OBS_UOP 0x06
#define NCLINK_FUS_U 0x07
#define NCLINK_FUS_NE 0x08
#define NCLINK_USER 0x09
#define NCLINK_SEND_CAL_RAW1 0x11
#define NCLINK_SEND_CAL_RAW2 0x12
#define NCLINK_SEND_CAL_PARA1 0x13
#define NCLINK_SEND_CAL_PARA2 0x14
#define NCLINK_SEND_CAL_PARA3 0x15

//地面站发送数据
uint8_t nclink_databuf[100];
//串口句柄
UART_HandleTypeDef huart1;

/**********************************************************************************************************
*函 数 名: NCLink_Init
*功能说明: 地面站初始化，使能串口
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void NCLink_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 921600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart1);
	
	HAL_NVIC_SetPriority(USART1_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**********************************************************************************************************
*函 数 名: Serial_Data_Send
*功能说明: 串口发送函数
*形    参: 数据指针 数据数量
*返 回 值: 无
**********************************************************************************************************/
static void Serial_Data_Send(uint8_t *buf, uint32_t cnt)
{
	HAL_UART_Transmit_IT(&huart1, buf, cnt);
}

/**********************************************************************************************************
*函 数 名: NCLink_Send_RCData
*功能说明: 发送遥控器各通道数据给地面站
*形    参: 遥控器第1通道数据 遥控器第2通道数据 遥控器第3通道数据 遥控器第4通道数据
			遥控器第5通道数据 遥控器第6通道数据 遥控器第7通道数据 遥控器第8通道数据
*返 回 值: 无
**********************************************************************************************************/
void NCLink_Send_RCData(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4,
						uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8)
{
	uint8_t cnt = 0, i = 0, sum = 0;
	nclink_databuf[cnt++] = 0xFF;
	nclink_databuf[cnt++] = 0xFC;
	nclink_databuf[cnt++] = NCLINK_RCDATA;
	nclink_databuf[cnt++] = 0;
	nclink_databuf[cnt++] = BYTE1(ch1);
	nclink_databuf[cnt++] = BYTE0(ch1);
	nclink_databuf[cnt++] = BYTE1(ch2);
	nclink_databuf[cnt++] = BYTE0(ch2);
	nclink_databuf[cnt++] = BYTE1(ch3);
	nclink_databuf[cnt++] = BYTE0(ch3);
	nclink_databuf[cnt++] = BYTE1(ch4);
	nclink_databuf[cnt++] = BYTE0(ch4);
	nclink_databuf[cnt++] = BYTE1(ch5);
	nclink_databuf[cnt++] = BYTE0(ch5);
	nclink_databuf[cnt++] = BYTE1(ch6);
	nclink_databuf[cnt++] = BYTE0(ch6);
	nclink_databuf[cnt++] = BYTE1(ch7);
	nclink_databuf[cnt++] = BYTE0(ch7);
	nclink_databuf[cnt++] = BYTE1(ch8);
	nclink_databuf[cnt++] = BYTE0(ch8);
	nclink_databuf[3] = cnt - 4;
	for (i = 0; i < cnt; i++)
		sum ^= nclink_databuf[i];
	nclink_databuf[cnt++] = sum;
	nclink_databuf[cnt++] = 0xA1;
	nclink_databuf[cnt++] = 0xA2;
	Serial_Data_Send(nclink_databuf, cnt);
}

/**********************************************************************************************************
*函 数 名: NCLink_Send_Fusion_NE
*功能说明: 发送水平方向状态估计数据给地面站
*形    参: 正北方向位置估计 正东方向位置估计 正北方向速度估计 正东方向速度估计 正北方向加速度估计 正东方向加速度估计
*返 回 值: 无
**********************************************************************************************************/
void NCLink_Send_Fusion_NE(float lat_pos_fus, float lng_pos_fus,
						   float lat_vel_fus, float lng_vel_fus,
						   float lat_accel_fus, float lng_accel_fus)
{
	uint8_t sum = 0, cnt = 0, i = 0;
	int32_t temp;
	nclink_databuf[cnt++] = 0xFF;
	nclink_databuf[cnt++] = 0xFC;
	nclink_databuf[cnt++] = NCLINK_FUS_NE;
	nclink_databuf[cnt++] = 0;
	temp = 100 * lat_pos_fus;
	nclink_databuf[cnt++] = BYTE3(temp);
	nclink_databuf[cnt++] = BYTE2(temp);
	nclink_databuf[cnt++] = BYTE1(temp);
	nclink_databuf[cnt++] = BYTE0(temp);
	temp = 100 * lng_pos_fus;
	nclink_databuf[cnt++] = BYTE3(temp);
	nclink_databuf[cnt++] = BYTE2(temp);
	nclink_databuf[cnt++] = BYTE1(temp);
	nclink_databuf[cnt++] = BYTE0(temp);
	temp = 100 * lat_vel_fus;
	nclink_databuf[cnt++] = BYTE3(temp);
	nclink_databuf[cnt++] = BYTE2(temp);
	nclink_databuf[cnt++] = BYTE1(temp);
	nclink_databuf[cnt++] = BYTE0(temp);
	temp = 100 * lng_vel_fus;
	nclink_databuf[cnt++] = BYTE3(temp);
	nclink_databuf[cnt++] = BYTE2(temp);
	nclink_databuf[cnt++] = BYTE1(temp);
	nclink_databuf[cnt++] = BYTE0(temp);
	temp = 100 * lat_accel_fus;
	nclink_databuf[cnt++] = BYTE3(temp);
	nclink_databuf[cnt++] = BYTE2(temp);
	nclink_databuf[cnt++] = BYTE1(temp);
	nclink_databuf[cnt++] = BYTE0(temp);
	temp = 100 * lng_accel_fus;
	nclink_databuf[cnt++] = BYTE3(temp);
	nclink_databuf[cnt++] = BYTE2(temp);
	nclink_databuf[cnt++] = BYTE1(temp);
	nclink_databuf[cnt++] = BYTE0(temp);
	nclink_databuf[3] = cnt - 4;
	for (i = 0; i < cnt; i++)
		sum ^= nclink_databuf[i];
	nclink_databuf[cnt++] = sum;
	nclink_databuf[cnt++] = 0xA1;
	nclink_databuf[cnt++] = 0xA2;
	Serial_Data_Send(nclink_databuf, cnt);
}

/**********************************************************************************************************
*函 数 名: NCLink_Send_Status
*功能说明: 发送姿态、温度、飞控状态给地面站
*形    参: 横滚角 俯仰角 偏航角 偏航角速度 偏航角速度 偏航角速度 温度 偏航角速度 飞行模式 解锁状态
*返 回 值: 无
**********************************************************************************************************/
void NCLink_Send_Status(float roll, float pitch, float yaw,
						float roll_gyro, float pitch_gyro, float yaw_gyro,
						float imutemp, float vbat, uint8_t fly_model, uint8_t armed)
{
	uint8_t cnt = 0;
	int16_t temp;
	uint8_t sum = 0;
	uint8_t i;
	nclink_databuf[cnt++] = 0xFF;
	nclink_databuf[cnt++] = 0xFC;
	nclink_databuf[cnt++] = NCLINK_STATUS;
	nclink_databuf[cnt++] = 0;

	temp = (int)(roll * 100);
	nclink_databuf[cnt++] = BYTE1(temp);
	nclink_databuf[cnt++] = BYTE0(temp);
	temp = (int)(pitch * 100);
	nclink_databuf[cnt++] = BYTE1(temp);
	nclink_databuf[cnt++] = BYTE0(temp);
	temp = (int)(yaw * 100);
	nclink_databuf[cnt++] = BYTE1(temp);
	nclink_databuf[cnt++] = BYTE0(temp);
	temp = 100 * roll_gyro;
	nclink_databuf[cnt++] = BYTE3(temp);
	nclink_databuf[cnt++] = BYTE2(temp);
	nclink_databuf[cnt++] = BYTE1(temp);
	nclink_databuf[cnt++] = BYTE0(temp);
	temp = 100 * pitch_gyro;
	nclink_databuf[cnt++] = BYTE3(temp);
	nclink_databuf[cnt++] = BYTE2(temp);
	nclink_databuf[cnt++] = BYTE1(temp);
	nclink_databuf[cnt++] = BYTE0(temp);

	temp = 100 * yaw_gyro;
	nclink_databuf[cnt++] = BYTE3(temp);
	nclink_databuf[cnt++] = BYTE2(temp);
	nclink_databuf[cnt++] = BYTE1(temp);
	nclink_databuf[cnt++] = BYTE0(temp);

	temp = (int16_t)(100 * imutemp);
	nclink_databuf[cnt++] = BYTE1(temp);
	nclink_databuf[cnt++] = BYTE0(temp);

	temp = (int16_t)(100 * vbat);
	nclink_databuf[cnt++] = BYTE1(temp);
	nclink_databuf[cnt++] = BYTE0(temp);

	nclink_databuf[cnt++] = fly_model;
	nclink_databuf[cnt++] = armed;
	nclink_databuf[3] = cnt - 4;
	for (i = 0; i < cnt; i++)
		sum ^= nclink_databuf[i];
	nclink_databuf[cnt++] = sum;
	nclink_databuf[cnt++] = 0xA1;
	nclink_databuf[cnt++] = 0xA2;
	Serial_Data_Send(nclink_databuf, cnt);
}

/**********************************************************************************************************
*函 数 名: NCLink_SEND_StateMachine
*功能说明: 状态机——飞控发送数据给地面站
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void NCLink_SEND_StateMachine(void)
{
	//NCLink_Send_Status(Roll, Pitch, Yaw, 0, 0, 0, tempDataFilter, 0, 0, 0);
	//NCLink_Send_Fusion_NE(0, 0, 0, 0, navigation_acce.x, navigation_acce.y);
	NCLink_Send_RCData(rc_raw_data[0], rc_raw_data[1], rc_raw_data[2], rc_raw_data[3],
		rc_raw_data[4], rc_raw_data[5], rc_raw_data[6], rc_raw_data[7]);
}
