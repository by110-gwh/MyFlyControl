#include "time_cnt.h"
#include "paramer_save.h"
#include "main_task.h"
#include "display_task.h"
#include "esc_task.h"

#include <stdbool.h>
#include <stdint.h>
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

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
    paramer_save_init();
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
	//ʱ��ϵͳ����80M
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	
	//������������
	xTaskCreate(vStartTask, "startTask", 128, NULL, configMAX_PRIORITIES - 1, &startTask);
	//OS����������
    vTaskStartScheduler();
	while (1);
}

#ifdef DEBUG
/**********************************************************************************************************
*�� �� ��: __error__
*����˵��: ����������Ϣ��Ӧ
*��    ��: ���������ļ��� �к�
*�� �� ֵ: ��
**********************************************************************************************************/
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif
