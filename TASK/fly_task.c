#include "fly_task.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "ppm.h"
#include "imu.h"
#include "motor_output.h"
#include "attitude_self_stabilization.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "vl53l1x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

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

    //���������
    //vTaskSuspendAll();
	
	imu_init();
	ahrs_init();
    angle_control_init();
    gyro_control_init();
    vl53l1x_init();
    //���ѵ�����
    //xTaskResumeAll();


    xLastWakeTime = xTaskGetTickCount();
    while (!fly_task_exit)
    {
        
		//��ȡimu����
		get_imu_data();
		ahrs_update();
		attitude_self_stabilization_control();
        angle_control();
        gyro_control();
		motor_output_output();
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
