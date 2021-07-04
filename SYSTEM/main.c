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

//启动任务句柄
xTaskHandle startTask;

void SystemClock_Config(void);
/**********************************************************************************************************
*函 数 名: vStartTask
*功能说明: 系统启动任务，调用各类初始化函数，并创建消息队列和要运行的用户任务
*形    参: 无
*返 回 值: 无
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
*函 数 名: main
*功能说明: 系统程序入口
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int main(void)
{
	//时钟系统配置80M
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	
	//创建启动任务
	xTaskCreate(vStartTask, "startTask", 128, NULL, configMAX_PRIORITIES - 1, &startTask);
	//OS调度器启动
    vTaskStartScheduler();
	while (1);
}

#ifdef DEBUG
/**********************************************************************************************************
*函 数 名: __error__
*功能说明: 驱动错误信息答应
*形    参: 错误发生的文件名 行号
*返 回 值: 无
**********************************************************************************************************/
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif
