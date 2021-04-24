#include "main.h"
#include "time_cnt.h"
#include "paramer_save.h"
#include "main_task.h"
#include "display_task.h"
#include "esc_task.h"

#include "FreeRTOS.h"
#include "task.h"

//����������
xTaskHandle startTask;

void SystemClock_Config(void);
/**********************************************************************************************************
*�� �� ��: vStartTask
*����˵��: ϵͳ�������񣬵��ø����ʼ����������������Ϣ���к�Ҫ���е��û�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
portTASK_FUNCTION(vStartTask, pvParameters)
{
	Get_Time_Init();
	read_save_paramer();
	
	display_task_create();

	if (paramer_save_data.esc_calibration_flag == 1)
		esc_task_create();
	else
		main_task_create();
	
	vTaskDelete(NULL);
}

/**********************************************************************************************************
*�� �� ��: main
*����˵��: ϵͳ�������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
int main(void)
{
	//ʹ��SWD���Խӿ�
	__HAL_AFIO_REMAP_SWJ_NOJTAG();
	//��ʼ��HAL��
	HAL_Init();
	//ʱ��ϵͳ����72M
	SystemClock_Config();
	
	//������������
	xTaskCreate(vStartTask, "startTask", 128, NULL, configMAX_PRIORITIES - 1, &startTask);
	//OS����������
    vTaskStartScheduler();
	while (1);
}

/**********************************************************************************************************
*�� �� ��: HAL_TIM_PeriodElapsedCallback
*����˵��: ��ʱ����������жϻص�����
*��    ��: ��ʱ�����
*�� �� ֵ: ��
**********************************************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
}

/**********************************************************************************************************
*�� �� ��: SystemClock_Config
*����˵��: ʱ��ϵͳ����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	//����RCC_OscInitTypeDef�ṹ��ָ���Ĳ�����ʼ��RCC������
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	while (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK);
	//��ʼ��CPU��AHB��APBʱ��
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
							  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	while (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK);
}
