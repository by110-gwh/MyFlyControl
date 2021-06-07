#include "fly_task.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "ppm.h"
#include "imu.h"
#include "vl53l1x.h"
#include "remote_control.h"
#include "controller.h"
#include "motor_output.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stdio.h"

//�����ջ��С
#define FLY_TASK_STACK 512
//�������ȼ�
#define FLY_TASK_PRIORITY 13

//����������
xTaskHandle fly_task_handle;
//�����˳���־
volatile uint8_t fly_task_exit;

/**********************************************************************************************************
*�� �� ��: fly_task
*����˵��: ���ж�ʱ����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
portTASK_FUNCTION(fly_task, pvParameters)
{
    portTickType xLastWakeTime;
	
	imu_init();
	ahrs_init();
    controller_init();
    vl53l1x_init();
    navigation_init();

    xLastWakeTime = xTaskGetTickCount();
    while (!fly_task_exit) {
		//��ȡimu����
		get_imu_data();
        //��ȡ�߶�
        vl53l1x_task();
        //��̬����
		ahrs_update();
        //�������ٶȼ���
        navigation_prepare();
        //�߶�λ�ù���
        high_kalman_filter();
        //������
        controller_run();
        //���������
		motor_output_output();
        
        //extern float high_vel, high_acce, high_pos;
        //printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n", high_vel, high_raw_data / 10.0, high_pos, high_acce, navigation_acce.z); 
        printf("%d\r\n", throttle_motor_output);
        //˯��5ms
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
	vTaskDelete(NULL);
}


/**********************************************************************************************************
*�� �� ��: fly_task_create
*����˵��: ���������������񴴽�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void fly_task_create(void)
{
	fly_task_exit = 0;
    xTaskCreate(fly_task, "fly_task", FLY_TASK_STACK, NULL, FLY_TASK_PRIORITY, &fly_task_handle);
}
