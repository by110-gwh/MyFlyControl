#include "ist8310.h"
#include "i2c.h"
#include "Filter.h"

//I2C地址
#define IST8310_ADDRESS 0xE

//IST8310寄存器地址
#define IST8310_REG_WIA           0x00	//Who I am
#define IST8310_REG_INFO          0x01	//More Info
#define IST8310_REG_STAT1         0x02	//Status register
#define IST8310_REG_DATAXL        0x03	//Output Value x
#define IST8310_REG_DATAXH        0x04	//Output Value x
#define IST8310_REG_DATAYL        0x05	//Output Value y
#define IST8310_REG_DATAYH        0x06	//Output Value y
#define IST8310_REG_DATAZL        0x07	//Output Value z
#define IST8310_REG_DATAZH        0x08	//Output Value z
#define IST8310_REG_STAT2         0x09	//Status register
#define IST8310_REG_CNTRL1        0x0A	//Control setting register 1
#define IST8310_REG_CNTRL2        0x0B	//Control setting register 2
#define IST8310_REG_STB           0x0C	//Self-Test response
#define IST8310_REG_STB           0x0C	//Self-Test response
#define IST8310_REG_TEMPL         0x1C	//Temperature data
#define IST8310_REG_TEMPH         0x1D	//Temperature data
#define IST8310_REG_AVGCNTL       0x41	//Average Control Register
#define IST8310_REG_PTCNTL        0x42	//Pulse Duration Control Register

//器件ID
#define IST8310_CHIP_ID 0x10

/**********************************************************************************************************
*函 数 名: IST8310_Detect
*功能说明: 检测IST8310是否存在
*形    参: 无
*返 回 值: 1：存在，0：不存在
**********************************************************************************************************/
uint8_t IST8310_Detect(void)
{
    uint8_t who_am_i;

    i2c_single_read(IST8310_ADDRESS, IST8310_REG_WIA, &who_am_i);

    if(who_am_i == IST8310_CHIP_ID)
        return 1;
    else
        return 0;
}

/**********************************************************************************************************
*函 数 名: IST8310_Init
*功能说明: IST8310寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void IST8310_Init(void)
{
	//开启16x内部平均
	i2c_single_write(IST8310_ADDRESS, IST8310_REG_AVGCNTL, 0x24);
	//Set/Reset内部平均
	i2c_single_write(IST8310_ADDRESS, IST8310_REG_PTCNTL, 0xC0);
}

/**********************************************************************************************************
*函 数 名: IST8310_ReadMag
*功能说明: IST8310读取加速度传感器原始数据
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void IST8310_ReadMag(Vector3i_t* mag)
{
    uint8_t buffer[6];

	i2c_multi_read_it(IST8310_ADDRESS, IST8310_REG_DATAXL, buffer, 6);
	
    mag->x = ((((int16_t)buffer[1]) << 8) | buffer[0]);
    mag->y = ((((int16_t)buffer[3]) << 8) | buffer[2]);
    mag->z = ((((int16_t)buffer[5]) << 8) | buffer[4]);

    //统一传感器坐标系（并非定义安装方向）
    mag->x = mag->x;
    mag->y = -mag->y;
    mag->z = mag->z;
}

/**********************************************************************************************************
*函 数 名: get_mag_data
*功能说明: 获取磁力计数据并滤波
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void IST8310_Single_Measurement()
{
	i2c_single_write(IST8310_ADDRESS,IST8310_REG_CNTRL1,0x01);
}
