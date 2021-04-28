#include "navigation.h"
#include "ahrs_aux.h"
#include "vector3.h"
#include "Filter.h"
#include "imu.h"
#include "math.h"

#include "FreeRTOS.h"
#include "task.h"

#define AcceGravity 9.80665f

//���Ե������ٶ�
Vector3f_t navigation_acce;
//���Ե������ٶ�ģ��
float navigation_acce_length;

//���ڹ��Ե����ļ��ٶ��˲�����
static Butter_BufferData Butter_Buffer_Navigation[3];
//���ڹ��Ե����ļ��ٶ��˲�����
static Butter_Parameter Butter_15HZ_Parameter_Navigation;

/**********************************************************************************************************
*�� �� ��: navigation_init
*����˵��: ���Ե���������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void navigation_init(void)
{
	Set_Cutoff_Frequency(Sampling_Freq, 15,&Butter_15HZ_Parameter_Navigation);
}

/**********************************************************************************************************
*�� �� ��: navigation_prepare
*����˵��: ���Ե���׼�����������ڹ��Ե����ļ��ٶ�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void navigation_prepare(void)
{
	Vector3f_t Body_Frame, Earth_Frame;
	
	Body_Frame.x = Butterworth_Filter(accDataFilter.x, &Butter_Buffer_Navigation[0], &Butter_15HZ_Parameter_Navigation);
	Body_Frame.y = Butterworth_Filter(accDataFilter.y, &Butter_Buffer_Navigation[1], &Butter_15HZ_Parameter_Navigation);
	Body_Frame.z = Butterworth_Filter(accDataFilter.z, &Butter_Buffer_Navigation[2], &Butter_15HZ_Parameter_Navigation);
	
	Vector_From_BodyFrame2EarthFrame(&Body_Frame, &Earth_Frame);
	
	navigation_acce = Earth_Frame;

	navigation_acce.z *= ACCEL_SCALE;
	//��ȥ�������ٶ�
	navigation_acce.z -= AcceGravity;
	//ת��Ϊ���ٶ�cm/s^2
	navigation_acce.z *= 100;

	navigation_acce.x *= ACCEL_SCALE;
	//ת��Ϊ���ٶ�cm/s^2
	navigation_acce.x *= 100;

	navigation_acce.y *= ACCEL_SCALE;
	//ת��Ϊ���ٶ�cm/s^2
	navigation_acce.y *= 100;

	navigation_acce_length = sqrt(navigation_acce.z * navigation_acce.z + navigation_acce.x * navigation_acce.x + navigation_acce.y * navigation_acce.y);
}
