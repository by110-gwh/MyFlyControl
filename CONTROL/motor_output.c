#include "motor_output.h"
#include "pwm.h"
#include "remote_control.h"
#include "gyro_control.h"

#include "FreeRTOS.h"
#include "task.h"

//��������޶�
#define Thr_Min 1000
//��������
#define Thr_Idle 1100

uint16_t throttle_motor_output;

//�ĸ�������ֵ
static uint16_t Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4;
//����Ѿ���ͣת��־
static uint8_t urgent_stop_flag;
static uint8_t motor_lock;

/**********************************************************************************************************
*�� �� ��: value_limit
*����˵��: �޶�data�����ֵ����Сֵ
*��    ��: ��Сֵ ���ֵ ����
*�� �� ֵ: �޶��������
**********************************************************************************************************/
static uint16_t value_limit(uint16_t min,uint16_t max,uint16_t data)
{
	if(data >= max)
		data = max;
	else if (data<=min)
		data = min;
	return data;
}

/**********************************************************************************************************
*�� �� ��: motor_output_init
*����˵��: ��������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void motor_output_init(void)
{
	motor_lock = 1;
	//PWM��ʼ��
	pwm_init();
	//�ĸ����ͣת
	Motor_PWM_1 = Thr_Min;
	Motor_PWM_2 = Thr_Min;
	Motor_PWM_3 = Thr_Min;
	Motor_PWM_4 = Thr_Min;
	pwm_set(Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4);
}

/**********************************************************************************************************
*�� �� ��: motor_output_unlock
*����˵��: �������������ȹ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void motor_output_unlock(void)
{
	uint8_t idel_cnt;
	
	//ͣתģʽ
	if (rc_raw_data[5] > rc_calibration_data[6].middle) {
		urgent_stop_flag = 1;
		Motor_PWM_1 = Thr_Min;
		Motor_PWM_2 = Thr_Min;
		Motor_PWM_3 = Thr_Min;
		Motor_PWM_4 = Thr_Min;
		pwm_set(Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4);
	} else {
		//��λ��־λ
		urgent_stop_flag = 0;
		//������ת
		for (idel_cnt = 0; idel_cnt <= 100; idel_cnt++) {
			Motor_PWM_1 = Thr_Min + (Thr_Idle - Thr_Min) * (idel_cnt / 100.F);
			Motor_PWM_2 = Thr_Min + (Thr_Idle - Thr_Min) * (idel_cnt / 100.F);
			Motor_PWM_3 = Thr_Min + (Thr_Idle - Thr_Min) * (idel_cnt / 100.F);
			Motor_PWM_4 = Thr_Min + (Thr_Idle - Thr_Min) * (idel_cnt / 100.F);
			pwm_set(Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4);
			vTaskDelay(50);
		}
	}
	motor_lock = 0;
}

/**********************************************************************************************************
*�� �� ��: motor_output_output
*����˵��: ������PWM���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void motor_output_output(void)
{
	uint16_t roll_motor_output;
	uint16_t pitch_motor_output;
	uint16_t yaw_motor_output;

	//�������
	if (motor_lock) {
		
	//����ͣ��
	} else if (rc_raw_data[5] > rc_calibration_data[6].middle || urgent_stop_flag == 1) {
		urgent_stop_flag = 1;
		//�ĸ����ͣת
		Motor_PWM_1 = Thr_Min;
		Motor_PWM_2 = Thr_Min;
		Motor_PWM_3 = Thr_Min;
		Motor_PWM_4 = Thr_Min;
		pwm_set(Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4);
	} else {
		pitch_motor_output = pitch_gyro_pid.control_output;
		roll_motor_output = roll_gyro_pid.control_output;
		yaw_motor_output = yaw_gyro_pid.control_output;
		//�����ĸ�������ֵ
		Motor_PWM_1 = throttle_motor_output - roll_motor_output + pitch_motor_output - yaw_motor_output;
		Motor_PWM_2 = throttle_motor_output + roll_motor_output - pitch_motor_output - yaw_motor_output;
		Motor_PWM_3 = throttle_motor_output + roll_motor_output + pitch_motor_output + yaw_motor_output;
		Motor_PWM_4 = throttle_motor_output - roll_motor_output - pitch_motor_output + yaw_motor_output;
		//������޷�
		Motor_PWM_1 = value_limit(Thr_Idle, 2000, Motor_PWM_1);
        Motor_PWM_2 = value_limit(Thr_Idle, 2000, Motor_PWM_2);
        Motor_PWM_3 = value_limit(Thr_Idle, 2000, Motor_PWM_3);
        Motor_PWM_4 = value_limit(Thr_Idle, 2000, Motor_PWM_4);
		pwm_set(Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4);
	}
}

/**********************************************************************************************************
*�� �� ��: motor_output_lock
*����˵��: ������ͣ������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void motor_output_lock(void)
{
	motor_lock = 1;
	Motor_PWM_1 = Thr_Min;
	Motor_PWM_2 = Thr_Min;
	Motor_PWM_3 = Thr_Min;
	Motor_PWM_4 = Thr_Min;
	pwm_set(Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4);
}
