#include "ist8310.h"
#include "i2c.h"
#include "Filter.h"

//I2C��ַ
#define IST8310_ADDRESS 0xE

//IST8310�Ĵ�����ַ
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

//����ID
#define IST8310_CHIP_ID 0x10

/**********************************************************************************************************
*�� �� ��: IST8310_Detect
*����˵��: ���IST8310�Ƿ����
*��    ��: ��
*�� �� ֵ: 1�����ڣ�0��������
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
*�� �� ��: IST8310_Init
*����˵��: IST8310�Ĵ������ó�ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void IST8310_Init(void)
{
	//����16x�ڲ�ƽ��
	i2c_single_write(IST8310_ADDRESS, IST8310_REG_AVGCNTL, 0x24);
	//Set/Reset�ڲ�ƽ��
	i2c_single_write(IST8310_ADDRESS, IST8310_REG_PTCNTL, 0xC0);
}

/**********************************************************************************************************
*�� �� ��: IST8310_ReadMag
*����˵��: IST8310��ȡ���ٶȴ�����ԭʼ����
*��    ��: ��������ָ��
*�� �� ֵ: ��
**********************************************************************************************************/
void IST8310_ReadMag(Vector3i_t* mag)
{
    uint8_t buffer[6];

	i2c_multi_read_it(IST8310_ADDRESS, IST8310_REG_DATAXL, buffer, 6);
	
    mag->x = ((((int16_t)buffer[1]) << 8) | buffer[0]);
    mag->y = ((((int16_t)buffer[3]) << 8) | buffer[2]);
    mag->z = ((((int16_t)buffer[5]) << 8) | buffer[4]);

    //ͳһ����������ϵ�����Ƕ��尲װ����
    mag->x = mag->x;
    mag->y = -mag->y;
    mag->z = mag->z;
}

/**********************************************************************************************************
*�� �� ��: get_mag_data
*����˵��: ��ȡ���������ݲ��˲�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void IST8310_Single_Measurement()
{
	i2c_single_write(IST8310_ADDRESS,IST8310_REG_CNTRL1,0x01);
}
