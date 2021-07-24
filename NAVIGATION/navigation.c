#include "navigation.h"
#include "ahrs_aux.h"
#include "vector3.h"
#include "Filter.h"
#include "imu.h"
#include "math.h"
#include "time_cnt.h"
#include "vl53l1x.h"
#include "optical_flow_task.h"

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

float pos_x, pos_y, pos_z;
float speed_x, speed_y, speed_z;
float acce_x, acce_y, acce_z;
static float optical_flow_pos_x_integral;
static float optical_flow_pos_y_integral;
static float filter_weight_speed = 0.01;
static float filter_weight_pos = 0.01;

/**********************************************************************************************************
*�� �� ��: constrain_float
*����˵��: ���������Сֵ
*��    ��: Ҫ���Ƶ�ֵ ��Сֵ ���ֵ
*�� �� ֵ: ���ֵ
**********************************************************************************************************/
static float constrain_float(float amt, float low, float high)
{
	return ((amt) < (low) ? (low) : ((amt) > (high)? (high) : (amt)));
}

/**********************************************************************************************************
*�� �� ��: navigation_init
*����˵��: ���Ե���������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void navigation_init(void)
{
    //��ʼ���˲�������
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
	
    //�������ٶȵ�ͨ�˲�
	Body_Frame.x = Butterworth_Filter(accDataFilter.x, &Butter_Buffer_Navigation[0], &Butter_15HZ_Parameter_Navigation);
	Body_Frame.y = Butterworth_Filter(accDataFilter.y, &Butter_Buffer_Navigation[1], &Butter_15HZ_Parameter_Navigation);
	Body_Frame.z = Butterworth_Filter(accDataFilter.z, &Butter_Buffer_Navigation[2], &Butter_15HZ_Parameter_Navigation);
	
    //������ٶ�ת�����������ٶ�
	Vector_From_BodyFrame2EarthFrame(&Body_Frame, &Earth_Frame);
	navigation_acce = Earth_Frame;

    //תΪm/s2
    navigation_acce.z -= 0xA0;
	navigation_acce.z *= ACCEL_SCALE;
	//��ȥ�������ٶ�
	navigation_acce.z -= AcceGravity;
	//ת��Ϊ���ٶ�cm/s^2
	navigation_acce.z *= 100;

    //תΪm/s2
	navigation_acce.x *= ACCEL_SCALE;
	//ת��Ϊ���ٶ�cm/s^2
	navigation_acce.x *= 100;

    //תΪm/s2
	navigation_acce.y *= ACCEL_SCALE;
	//ת��Ϊ���ٶ�cm/s^2
	navigation_acce.y *= 100;

    //���㵼�����ٶ�ģ��
	navigation_acce_length = sqrt(navigation_acce.z * navigation_acce.z + navigation_acce.x * navigation_acce.x + navigation_acce.y * navigation_acce.y);
}

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
    float Q[2] = {5.0e-3f, 6.0e-3f};
    float R = 100;
    //�������������
    float K[2];
    float temp;

    //���¼���ʱ���
    Get_Time_Period(&Time_Delta);
    dt = Time_Delta.Time_Delta / 1000000.0;

    //����״̬
    acce_z = navigation_acce.z;
    acce_z += acce_bias;
    pos_z += speed_z * dt + (acce_z * dt * dt) / 2.0f;
    speed_z += acce_z * dt;
	
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
		temp = high_raw_data * Cos_Roll * Cos_Pitch / 10 - pos_history[20 - 1];
		pos_z += K[0] * temp;
		speed_z += K[1] * temp;
		acce_bias += 0.0015f * temp;
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
    pos_history[0] = pos_z;
}
#include <stdio.h>
static float pos_history_x[40];
static float pos_history_y[40];
static float speed_history_x[40];
static float speed_history_y[40];
void pos_filter(void)
{
    float dt = 0.005f;
    //���
    float optical_flow_speed_err_x = optical_flow_speed_x - speed_history_x[39];
    float optical_flow_speed_err_y = optical_flow_speed_y - speed_history_y[39];
    float optical_flow_pos_err_x = optical_flow_pos_x - pos_history_x[39];
    float optical_flow_pos_err_y = optical_flow_pos_y - pos_history_y[39];
    
    //�����˲�
    acce_x = -navigation_acce.x;
    acce_y = navigation_acce.y;
    speed_x += acce_x * dt + filter_weight_speed * optical_flow_speed_err_x;
    speed_y += acce_y * dt + filter_weight_speed * optical_flow_speed_err_y;
    pos_x += speed_x * dt + 0.5f * acce_x * dt * dt + filter_weight_pos * optical_flow_pos_err_x;
    pos_y += speed_y * dt + 0.5f * acce_y * dt * dt + filter_weight_pos * optical_flow_pos_err_y;
    
    printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n", navigation_acce.y / 10, speed_y, pos_y, optical_flow_pos_y, optical_flow_speed_y, optical_flow_speed_err_y, optical_flow_pos_err_y); 
    
    //������ʷֵ
    uint8_t i;
    for (i = 39; i > 0; i--) {
        speed_history_x[i] = speed_history_x[i - 1];
        speed_history_y[i] = speed_history_y[i - 1];
        pos_history_x[i] = pos_history_x[i - 1];
        pos_history_y[i] = pos_history_y[i - 1];
    }
    speed_history_x[0] = speed_x;
    speed_history_y[0] = speed_y;
    pos_history_x[0] = pos_x;
    pos_history_y[0] = pos_y;
}
