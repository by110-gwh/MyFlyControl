#include "spi.h"
#include "stm32f1xx_hal.h"

/*
PB13     ------> SPI2_SCK
PB14     ------> SPI2_MISO
PB15     ------> SPI2_MOSI
*/

/**********************************************************************************************************
*函 数 名: Spi_GPIO_Init
*功能说明: SPI从机设备CS引脚初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	SPI_HandleTypeDef SPI_InitStructure;
	
	//外设时钟使能
	__HAL_RCC_SPI2_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	//IO初始化
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	//SPI2初始化
	SPI_InitStructure.Instance = SPI2;
	SPI_InitStructure.Init.Mode = SPI_MODE_MASTER;
	SPI_InitStructure.Init.Direction = SPI_DIRECTION_2LINES;
	SPI_InitStructure.Init.DataSize = SPI_DATASIZE_8BIT;
	SPI_InitStructure.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPI_InitStructure.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPI_InitStructure.Init.NSS = SPI_NSS_SOFT;
	SPI_InitStructure.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	SPI_InitStructure.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPI_InitStructure.Init.TIMode = SPI_TIMODE_DISABLE;
	SPI_InitStructure.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	SPI_InitStructure.Init.CRCPolynomial = 10;
	HAL_SPI_Init(&SPI_InitStructure);
}

/**********************************************************************************************************
*函 数 名: Spi_SingleWirteAndRead
*功能说明: SPI单字节读取
*形    参: 设备号 写入的数据
*返 回 值: 读取到的数据
**********************************************************************************************************/
uint8_t Spi_SingleWirteAndRead(uint8_t deviceNum, uint8_t dat)
{
    if(deviceNum == 1)
    {
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(SPI1, dat);
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
        return SPI_I2S_ReceiveData(SPI1);
    }
    else if(deviceNum == 2)
    {
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(SPI2, dat);
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
        return SPI_I2S_ReceiveData(SPI2);
    }
    else
    {
        return 0;
    }
}

/**********************************************************************************************************
*函 数 名: SPI_MultiWriteAndRead
*功能说明: SPI多字节读取
*形    参: 设备号 写入数据缓冲区指针 读出数据缓冲区指针 数据长度
            同时只能写入或者读出，写入时读取缓冲区设置为NULL，读出时反之
*返 回 值: 无
**********************************************************************************************************/
void SPI_MultiWriteAndRead(uint8_t deviceNum, uint8_t *out, uint8_t *in, int len)
{
    uint8_t b;
    if(deviceNum == 1)
    {
        while (len--)
        {
            b = in ? *(in++) : 0xFF;
            while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(SPI1, b);
            while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
            b = SPI_I2S_ReceiveData(SPI1);
            if (out)
                *(out++) = b;
        }
    }
    else if(deviceNum == 2)
    {
        while (len--)
        {
            b = in ? *(in++) : 0xFF;
            while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(SPI2, b);
            while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
            b = SPI_I2S_ReceiveData(SPI2);
            if (out)
                *(out++) = b;
        }
    }
}
