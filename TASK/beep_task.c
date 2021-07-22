#include "beep_task.h"
#include "beep.h"

#include "FreeRTOS.h"
#include "task.h"

//�����ջ��С
#define BEEP_TASK_STACK 128
//�������ȼ�
#define BEEP_TASK_PRIORITY 6

//����������
xTaskHandle beep_task_handle;

//������ռ�ձ�
uint8_t beep_duty;
//����������
uint16_t beep_cycle;
//��������������
uint8_t beep_time;

/**********************************************************************************************************
*�� �� ��: beep_task
*����˵��: ����������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
portTASK_FUNCTION(beep_task, pvParameters)
{
    //��������ʼ��
    beep_init();
    while (1) {
        //��Ҫ����
        if (beep_time) {
            //����
            beep_control(1);
            vTaskDelay((beep_cycle * 10 * beep_duty / 100 / portTICK_RATE_MS));
            //ֹͣ����
            beep_control(0);
            vTaskDelay((beep_cycle * 10 * (100 - beep_duty) / 100 / portTICK_RATE_MS));
            //���·���
            if (beep_time != 0xFF && beep_time != 0) {
                beep_time--;
            }
        }
        //ֹͣ����
        beep_control(0);
        vTaskDelay(100);
    }
}


/**********************************************************************************************************
*�� �� ��: beep_task_create
*����˵��: ���������񴴽�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void beep_task_create(void)
{
    xTaskCreate(beep_task, "beep_task", BEEP_TASK_STACK, NULL, BEEP_TASK_PRIORITY, &beep_task_handle);
}
