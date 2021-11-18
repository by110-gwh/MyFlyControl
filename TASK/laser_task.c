#include "laser_task.h"
#include "laser.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

//�����ջ��С
#define LASER_TASK_STACK 128
//�������ȼ�
#define LASER_TASK_PRIORITY 8

//�����˳���־
volatile uint8_t laser_task_exit = 1;
//����������
xTaskHandle laser_task_handle;

//����ռ�ձ�
uint8_t laser_duty;
//��������
uint16_t laser_cycle;
//������˸����
uint8_t laser_time;

/**********************************************************************************************************
*�� �� ��: laser_task
*����˵��: �����������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
portTASK_FUNCTION(laser_task, pvParameters)
{
    laser_init();
    while (!laser_task_exit) {
        //��Ҫ����
        if (laser_time) {
            //��
            laser_on();
            vTaskDelay((laser_cycle * 10 * laser_duty / 100 / portTICK_RATE_MS));
            //Ϩ��
            laser_off();
            vTaskDelay((laser_cycle * 10 * (100 - laser_duty) / 100 / portTICK_RATE_MS));
            //���·���
            if (laser_time != 0xFF && laser_time != 0) {
                laser_time--;
            }
        }
        //ֹͣ����
        laser_off();
        vTaskDelay(100);
    }
    vTaskDelete(NULL);
}


/**********************************************************************************************************
*�� �� ��: laser_task_create
*����˵��: �������������񴴽�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void laser_task_create(void)
{
    laser_task_exit = 0;
    xTaskCreate(laser_task, "laser_task", LASER_TASK_STACK, NULL, LASER_TASK_PRIORITY, &laser_task_handle);
}
