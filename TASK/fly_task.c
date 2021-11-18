#include "fly_task.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "ppm.h"
#include "imu.h"
#include "vl53l1x.h"
#include "remote_control.h"
#include "controller.h"
#include "motor_output.h"
#include "optical_flow_task.h"

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
volatile uint8_t fly_task_exit = 1;
//����������±�־�����ڰ�ȫ���
volatile uint8_t fly_task_updata;

/**********************************************************************************************************
*�� �� ��: fly_task
*����˵��: ���ж�ʱ����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
portTASK_FUNCTION(fly_task, pvParameters)
{
    portTickType xLastWakeTime;
	
    optical_flow_task_create();
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
        high_filter();
        pos_filter();
        //������
        controller_run();
        //���������
		motor_output_output();
        
        fly_task_updata = 1;
        //˯��5ms
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
    optical_flow_task_exit = 1;
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
