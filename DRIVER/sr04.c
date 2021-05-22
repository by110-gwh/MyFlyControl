#include "sr04.h"
#include "stm32f1xx_hal.h"
#include "time_cnt.h"
#include "bitband.h"

#include "FreeRTOS.h"
#include "task.h"

//�����ջ��С
#define SR04_TASK_STACK            256
//�������ȼ�
#define SR04_TASK_PRIORITY         12
//��������
#define TRIG PBout(8)

//����������
xTaskHandle sr04_task_handle;
//�����˳���־
volatile uint8_t sr04_task_exit;
//�߶�ԭʼ����
uint16_t high_raw_data;

/**********************************************************************************************************
*�� �� ��: sr04_task
*����˵��: ��������������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
portTASK_FUNCTION(sr04_task, parameters)
{
    portTickType xLastWakeTime;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
	
    //��ʼ��GPIOʱ��
	__HAL_RCC_GPIOB_CLK_ENABLE();
    
    //��ʼ����������
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    //��ʼ���ж�����
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    xLastWakeTime = xTaskGetTickCount();
    while (!sr04_task_exit) {
        //������ʼ����
        TRIG = 1;
        vTaskDelay(1);
        TRIG = 0;
        
        //˯��100ms
        vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_RATE_MS));
    }
	vTaskDelete(NULL);
}

/**********************************************************************************************************
*�� �� ��: sr04_task_create
*����˵��: ������������񴴽�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void sr04_task_create(void)
{
	sr04_task_exit = 0;
    xTaskCreate(sr04_task, "sr04_task", SR04_TASK_STACK, NULL, SR04_TASK_PRIORITY, &sr04_task_handle);
}

/**********************************************************************************************************
*�� �� ��: sr04_exit_callback
*����˵��: �������ⲿ�жϻص�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void sr04_exit_callback()
{
    static Testime timer;
    static uint8_t wait_fall;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    //����������
    if (wait_fall == 0) {
        //��¼ʱ��
        Get_Time_Period(&timer);
        //����Ϊ�½��ش����ж�
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        wait_fall = 1;
    //�����½���
    } else {
        //����ʱ���
        Get_Time_Period(&timer);
        //�������
        high_raw_data = timer.Time_Delta * 0.340 / 2;
        if (high_raw_data > 2000)
            high_raw_data = 0;
        //����Ϊ�½��ش����ж�
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        wait_fall =-0;
    }
}

