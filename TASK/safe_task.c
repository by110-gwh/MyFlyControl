#include "safe_task.h"
#include "pwm.h"
#include "remote_control.h"
#include "fly_task.h"

#include "FreeRTOS.h"
#include "task.h"

//�����ջ��С
#define SAFE_TASK_STACK 128
//�������ȼ�
#define SAFE_TASK_PRIORITY 10


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
    //�ȴ������ʼ��
    vTaskDelay((1000 / portTICK_RATE_MS));
    while (1) {
        //ң��������
        if (remote_control_updata == 0)
            safe_task_motor_stop();
        //����������
        if (fly_task_exit == 0 && fly_task_updata == 0)
            safe_task_motor_stop();
        
        fly_task_updata = 0;
        remote_control_updata = 0;
        //˯��200ms
        vTaskDelay((200 / portTICK_RATE_MS));
    }
}


/**********************************************************************************************************
*�� �� ��: safe_task_create
*����˵��: ���а�ȫ�������������񴴽�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void safe_task_create(void)
{
    xTaskCreate(safe_task, "safe_task", SAFE_TASK_STACK, NULL, SAFE_TASK_PRIORITY, &safe_task_handle);
}
