#include "debug_uart.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>

#include "FreeRTOS.h"
#include "queue.h"

//���ھ��
UART_HandleTypeDef huart3;
//���Ͷ���
QueueHandle_t tx_queue;

//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
//����״̬���
uint16_t USART_RX_STA;
//���ջ���,���USART_REC_LEN���ֽ�.
uint8_t USART_RX_BUF[USART_REC_LEN];

//���������FreeRTOS��port.c�п����ҵ�
extern uint32_t vPortGetIPSR(void);

/**********************************************************************************************************
*�� �� ��: debug_uart_init
*����˵��: ���Դ��ڳ�ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void debug_uart_init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//�������Ͷ���
	tx_queue = xQueueCreate(64, sizeof(uint8_t));
	
	//���ʱ��ʹ��
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	//IO��ʼ��
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	//���ڳ�ʼ��
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart3);

	//�����е���ʼ��
	HAL_NVIC_SetPriority(USART3_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	
	//���������ж�
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

/**********************************************************************************************************
*�� �� ��: fputc
*����˵��: �ض�����printf����
*��    ��: Ҫ���͵��ֽ� �豸�ļ�
*�� �� ֵ: ������ɵ��ֽ�
**********************************************************************************************************/
int fputc(int ch, FILE *f)
{
	if (vPortGetIPSR()) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		//���������д��Ҫ���͵�����
        while (xQueueSendFromISR(tx_queue, &ch, &xHigherPriorityTaskWoken) != pdPASS);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
		//���������д��Ҫ���͵�����
        xQueueSend(tx_queue, &ch, 1);
    }
	//ʹ�ܷ��Ϳ��ж�
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_TXE);
	return ch;
}

void USART3_IRQHandler(void)
{
	uint8_t Res;
	//�����ж�
	if(__HAL_UART_GET_FLAG(&huart3, USART_SR_RXNE)) {
		//��ȡ��������
		Res = huart3.Instance->DR;
		//����δ���
		if ((USART_RX_STA & 0x8000) == 0) {
			//���յ���0x0d
			if (USART_RX_STA & 0x4000) {
				if (Res != 0x0a)
					//���մ���,���¿�ʼ
					USART_RX_STA = 0;
				else
					//���������
					USART_RX_STA |= 0x8000;
			//��û�յ�0X0D
			} else {
				if (Res == 0x0d)
					USART_RX_STA |= 0x4000;
				else {
					USART_RX_BUF[USART_RX_STA & 0X3FFF] = Res;
					USART_RX_STA++;
					if (USART_RX_STA > (USART_REC_LEN - 1))
						//�������ݴ���,���¿�ʼ����
						USART_RX_STA = 0;
				}
			}
		}
	} else if(__HAL_UART_GET_FLAG(&huart3, USART_SR_TXE)) {
		
		BaseType_t xTaskWokenByReceive = pdFALSE;
		//���Ͷ�������������Ҫ����
		if (xQueueReceiveFromISR(tx_queue, (void *) &Res, &xTaskWokenByReceive) == pdPASS)
			huart3.Instance->DR = Res;
		else
			//�����ݷ��;͹رշ����ж�
			__HAL_UART_DISABLE_IT(&huart3, UART_IT_TXE);
	}
}
