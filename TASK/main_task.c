#include "main_task.h"
#include "remote_control.h"
#include "key.h"
#include "safe_task.h"
#include "display_task.h"
#include "fly_task.h"
#include "esc_task.h"
#include "usmart_task.h"
#include "imu.h"
#include "i2c.h"
#include "i2c1.h"
#include "motor_output.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "controller.h"

#include "FreeRTOS.h"
#include "task.h"

//�����ջ��С
#define MAIN_TASK_STACK            512
//�������ȼ�
#define MAIN_TASK_PRIORITY         5

//����������
xTaskHandle main_task_handle;
//�����˳���־
volatile uint8_t main_task_exit;

/**********************************************************************************************************
*�� �� ��: main_task
*����˵��: ������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
portTASK_FUNCTION(main_task, parameters)
{
	key_init();
	rc_init();
	i2c_init();
	i2c1_init();
	motor_output_init();
	usmart_task_create();
	//���ң�����Ƿ�����
	page_number = 17;
	while (!rc_is_on())
		vTaskDelay(1);
	page_number = 0;
    while (!main_task_exit) {
        uint8_t key;
		key = key_scan();
		//key0��������ң�����г�У׼
		if ((key & KEY0) << 1 && key & 1) {
			page_number = 14;
			rc_calibration_task();
			page_number = 0;
		}
		
		key = rc_scan();
		//���ڰ˽��е��У׼
		if (key == 0x0E) {
			page_number = 16;
			esc_calibration();
		//����˽��м��ټ�У׼
		} else if (key == 0x01) {
			page_number = 18;
			accel_calibration();
			page_number = 0;
		//����˽��д�����У׼
		} else if (key == 0x07) {
			page_number = 19;
			mag_calibration();
			page_number = 0;
		//���ڰ˽��н���
		} else if (key == 0x08) {
			//������У׼
			page_number = 2;
			gyro_calibration();
			//�������񴴽�
			page_number = 3;
			fly_task_create();
            vTaskDelay(2000);
			//�ȵ�ң��������ң�й�λ
			while (rc_direct_is_reset() == 0);
			page_number = 4;
//            safe_task_create();
			//�������
			motor_output_unlock();
			//��PID����
            controller_init();
			//�ȴ��������ʱ����̬�ں���Ϻ󣬸���ƫ���ڴ�
			yaw_angle_pid.short_circuit_flag = 1;
			page_number = 1;
			while(1) {
				key = rc_scan();
				if (key == 0x08) {
					break;
				}
				vTaskDelay(10);
			}
			fly_task_exit = 1;
			vTaskDelay(5);
			motor_output_lock();
			page_number = 0;
		}
		vTaskDelay(6);
    }
	vTaskDelete(NULL);
}

/**********************************************************************************************************
*�� �� ��: main_task_create
*����˵��: ������������񴴽�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void main_task_create(void)
{
	main_task_exit = 0;
    xTaskCreate(main_task, "main_task", MAIN_TASK_STACK, NULL, MAIN_TASK_PRIORITY, &main_task_handle);
}
