#include "main_task.h"
#include "remote_control.h"
#include "key.h"
#include "safe_task.h"
#include "display_task.h"
#include "fly_task.h"
#include "esc_task.h"
#include "usmart_task.h"
#include "beep_task.h"
#include "imu.h"
#include "i2c.h"
#include "i2c1.h"
#include "spi0.h"
#include "spi1.h"
#include "uart7.h"
#include "w25qxx.h"
#include "motor_output.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "controller.h"
#include "openmv.h"
#include "sr04.h"

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
uint8_t fly_task_num;

/**********************************************************************************************************
*�� �� ��: fly_enter
*����˵��: �����������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void fly_enter(void)
{
    //������У׼
    page_number = 2;
    gyro_calibration();
    //�������񴴽�
    page_number = 3;
    beep_duty = 5;
    beep_cycle = 100;
    beep_time = 0xFF;
    fly_task_create();
    vTaskDelay(2000);
    page_number = 4;
    safe_task_create();
    //��PID����
    controller_init();
    //�������
    motor_output_unlock();
    //�ȴ��������ʱ����̬�ں���Ϻ󣬸���ƫ���ڴ�
    yaw_angle_pid_data.short_circuit_flag = 1;
    page_number = 1;
}

/**********************************************************************************************************
*�� �� ��: fly_exit
*����˵��: �˳���������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void fly_exit(void)
{
    safe_task_exit = 1;
    fly_task_exit = 1;
    vTaskDelay(5);
    motor_output_lock();
    beep_time = 0;
    page_number = 0;
}

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
    spi0_init();
    spi1_init();
    w25qxx_init();
	motor_output_init();
	usmart_task_create();
    beep_task_create();
    uart7_init();
    openmv_init();
    sr04_init();
    
	//���ң�����Ƿ�����
	page_number = 17;
	while (!rc_is_on())
		vTaskDelay(1);
    while (!main_task_exit) {
        uint8_t key;
        page_number = 0;
		key = key_scan();
		//key0��������ң�����г�У׼
		if ((key & (KEY0 << 1)) && key & 1) {
			page_number = 14;
			rc_calibration_task();
			page_number = 0;
		} else if ((key & (KEY1 << 1)) && key & 1) {
            uint32_t i = 5 * 100;
            while (i--) {
                page_number = 5;
                key = key_scan();
                //ͨ������ȷ��ִ���ĸ�����
                if ((key & (KEY0 << 1)) && key & 1) {
                    fly_task_num = 1;
                }
                if ((key & (KEY1 << 1)) && key & 1) {
                    fly_task_num = 2;
                }
                //������������
                if (((key & (KEY0 << 1))  && key & 1) || ((key & (KEY1 << 1)) && key & 1)) {
                    vTaskDelay(3000);
                    fly_enter();
                    while(1) {
                        key = rc_scan();
                        if (key == 0x08) {
                            break;
                        }
                        vTaskDelay(50);
                    }
                    fly_exit();
                    break;
                }
                vTaskDelay(10);
            }
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
            //�ȵ�ң��������ң�й�λ
            while (rc_direct_is_reset() == 0);
			fly_enter();
			while(1) {
				key = rc_scan();
				if (key == 0x08) {
					break;
				}
				vTaskDelay(50);
			}
            fly_exit();
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
