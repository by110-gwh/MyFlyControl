#include "safe_task.h"
#include "pwm.h"
#include "remote_control.h"
#include "fly_task.h"
#include "ahrs_aux.h"
#include "beep.h"

#include <stdbool.h>
#include <stdint.h>
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"

#include "FreeRTOS.h"
#include "task.h"

//�����ջ��С
#define SAFE_TASK_STACK 128
//�������ȼ�
#define SAFE_TASK_PRIORITY 10

//�����˳���־
volatile uint8_t safe_task_exit = 1;
//����������
xTaskHandle safe_task_handle;

/**********************************************************************************************************
*�� �� ��: safe_task_motor_stop
*����˵��: ����ͣ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void safe_task_motor_stop(void)
{
    __disable_fiq();
    beep_control(1);
    while (1) {
        pwm_set(1000, 1000, 1000, 1000);
    }
}

/**********************************************************************************************************
*�� �� ��: safe_task
*����˵��: ���а�ȫ����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
portTASK_FUNCTION(safe_task, pvParameters)
{
    IntRegister(FAULT_NMI, safe_task_motor_stop);
    IntRegister(FAULT_HARD, safe_task_motor_stop);
    IntRegister(FAULT_MPU, safe_task_motor_stop);
    IntRegister(FAULT_BUS, safe_task_motor_stop);
    IntRegister(FAULT_USAGE, safe_task_motor_stop);
    //�ȴ������ʼ��
    vTaskDelay((1000 / portTICK_RATE_MS));
    while (!safe_task_exit) {
        //ң��������
        if (remote_control_updata == 0)
            safe_task_motor_stop();
        //����������
        if (fly_task_exit == 0 && fly_task_updata == 0)
            safe_task_motor_stop();
        //������̬�쳣
        if (Pitch > 40 || Roll > 40)
            safe_task_motor_stop();
        
        fly_task_updata = 0;
        remote_control_updata = 0;
        //˯��200ms
        vTaskDelay((200 / portTICK_RATE_MS));
    }
    vTaskDelete(NULL);
}


/**********************************************************************************************************
*�� �� ��: safe_task_create
*����˵��: ���а�ȫ�������������񴴽�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void safe_task_create(void)
{
    safe_task_exit = 0;
    xTaskCreate(safe_task, "safe_task", SAFE_TASK_STACK, NULL, SAFE_TASK_PRIORITY, &safe_task_handle);
}
