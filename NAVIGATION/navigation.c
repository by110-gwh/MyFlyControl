#include "navigation.h"
#include "ahrs_aux.h"
#include "vector3.h"
#include "Filter.h"
#include "imu.h"
#include "math.h"
#include "time_cnt.h"
#include "sr04.h"

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

//�߶ȹ��Ե������ٶ�
float high_acce;
//�߶ȹ��Ե���λ��
float high_pos;
//�߶ȹ��Ե����ٶ�
float high_vel;

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
float constrain_float(float amt, float low, float high);
/**********************************************************************************************************
*�� �� ��: high_kalman_filter
*����˵��: �߶ȿ������˲�
*��    ��: �߶ȹ۲��� �۲⴫������ʱ�� �ߵ����ٶ� 
*�� �� ֵ: ��
**********************************************************************************************************/
void high_kalman_filter()
{
    static uint8_t cnt;
    //���ٶ����
    static float acce_bias;
    //���ڼ���ʱ���Ľṹ��
    static Testime Time_Delta;
    //ʱ���
    float dt;
    static float pos_history[20];
    //����Э����
    static float pos_conv[2][2] = {
        {0.18, 0.1},
        {0.1, 0.18}
    };
    //����Э����
    float prior_conv[2][2];
    //ϵͳ����Э����
    float Q[2] = {5.0e-4f, 6.0e-4f};
    float R = 100;
    //�������������
    float K[2];
    float temp;

    //���¼���ʱ���
    Get_Time_Period(&Time_Delta);
    dt = Time_Delta.Time_Delta / 1000000.0;

    //����״̬
    high_acce = navigation_acce.z;
    high_acce += acce_bias;
    high_pos += high_vel * dt + (high_acce * dt * dt) / 2.0;
    high_vel += high_acce * dt;
	
    //���۲�ֵ����ʱ�Ž����ں�
    if(high_raw_data) {
		//����Э����
		temp = pos_conv[0][1] + pos_conv[1][1] * dt;
		prior_conv[0][0] = pos_conv[0][0] + pos_conv[1][0] * dt + temp * dt + Q[0];
		prior_conv[0][1] = temp;
		prior_conv[1][0] = pos_conv[1][0] + pos_conv[1][1] * dt;;
		prior_conv[1][1] = pos_conv[1][1] + Q[1];
        
		//���㿨��������
		K[0] = prior_conv[0][0] / (prior_conv[0][0] + R);
		K[1] = prior_conv[1][0] / (prior_conv[0][0] + R);
        
		//�ں��������
		temp = high_raw_data / 10 - pos_history[20 - 1];
		high_pos += K[0] * temp;
		high_vel += K[1] * temp;
		acce_bias += 0.0005 * temp;
		acce_bias = constrain_float(acce_bias, -200, 200);
        
		//����״̬Э�������
		pos_conv[0][0] = (1 - K[0]) * prior_conv[0][0];
		pos_conv[0][1] = (1 - K[0]) * prior_conv[0][1];
		pos_conv[1][0] = prior_conv[1][0] - K[1] * prior_conv[0][0];
		pos_conv[1][1] = prior_conv[1][1] - K[1] * prior_conv[0][1];
	}
    
    //5ms����һ��
    for(cnt = 20 - 1; cnt > 0; cnt--) {
       pos_history[cnt] = pos_history[cnt - 1];
    }
    pos_history[0] = high_pos;
}
