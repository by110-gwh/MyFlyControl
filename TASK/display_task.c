#include "display_task.h"
#include "oled.h"
#include "ppm.h"
#include "remote_control.h"
#include "time_cnt.h"
#include "imu.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "vl53l1x.h"
#include "optical_flow_task.h"

#include "FreeRTOS.h"
#include "task.h"

//任务堆栈大小
#define DISPLAY_TASK_STACK            256
//任务优先级
#define DISPLAY_TASK_PRIORITY         6

//声明任务句柄
static xTaskHandle display_task_handle;
//任务退出标志
volatile uint8_t display_task_exit;
//当前页
volatile int16_t page_number;

/**********************************************************************************************************
*函 数 名: display_task
*功能说明: 显示任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(display_task,  parameters)
{
	oled_init();
	while (!display_task_exit) {
		if(page_number == 0) {
			oled_clear_line(0, 7);
			oled_6x8_str(0, 7, "main");
        }
		if(page_number == 1) {
			oled_clear_line(0, 0);
			oled_6x8_str(0, 0, "Yaw:");
			oled_6x8_number(40, 0, 7, Yaw);
			oled_6x8_number(90, 0, 5, gyroDataFilter.z * GYRO_CALIBRATION_COFF);
			oled_clear_line(0, 1);
			oled_6x8_str(0, 1, "Pitch:");
			oled_6x8_number(40, 1, 7, Pitch);
			oled_6x8_number(90, 1, 5, gyroDataFilter.x * GYRO_CALIBRATION_COFF);
			oled_clear_line(0, 2);
			oled_6x8_str(0, 2, "Roll:");
			oled_6x8_number(40, 2, 7, Roll);
			oled_6x8_number(90, 2, 5, gyroDataFilter.y * GYRO_CALIBRATION_COFF);
			oled_clear_line(0, 3);
			oled_6x8_str(0, 3, "Pos_z:");
			oled_6x8_number(40, 3, 7, pos_z);
			oled_6x8_number(90, 3, 5, high_raw_data);
			oled_clear_line(0, 4);
			oled_6x8_str(0, 4, "Pos_y:");
			oled_6x8_number(40, 4, 7, pos_y);
			oled_6x8_number(90, 4, 5, optical_flow_pos_y);
			oled_clear_line(0, 5);
			oled_6x8_str(0, 5, "Pos_x:");
			oled_6x8_number(40, 5, 7, pos_x);
			oled_6x8_number(90, 5, 5, optical_flow_pos_x);
			oled_clear_line(0, 6);
			oled_clear_line(0, 7);
		} else if(page_number == 2) {
			oled_clear_line(0, 0);
			oled_6x8_str(0, 0, "GYRO cailbration");
			oled_clear_line(0, 1);
			oled_clear_line(0, 2);
			oled_clear_line(0, 3);
			oled_clear_line(0, 4);
			oled_clear_line(0, 5);
			oled_clear_line(0, 6);
			oled_clear_line(0, 7);
		} else if(page_number == 3) {
			oled_clear_line(0, 0);
			oled_6x8_str(0, 0, "GYRO cailbration OK");
			oled_clear_line(0, 1);
			oled_6x8_str(0, 1, "remote control reset");
			oled_clear_line(0, 2);
			oled_clear_line(0, 3);
			oled_clear_line(0, 4);
			oled_clear_line(0, 5);
			oled_clear_line(0, 6);
			oled_clear_line(0, 7);
		} else if(page_number == 4) {
			oled_clear_line(0, 0);
			oled_6x8_str(0, 0, "GYRO cailbration OK");
			oled_clear_line(0, 1);
			oled_6x8_str(0, 1, "remote control reset OK");
			oled_clear_line(0, 2);
			oled_6x8_str(0, 2, "engine starting");
			oled_clear_line(0, 3);
			oled_clear_line(0, 4);
			oled_clear_line(0, 5);
			oled_clear_line(0, 6);
			oled_clear_line(0, 7);
		} else if(page_number == 5) {
			oled_clear_line(0, 0);
			oled_6x8_str(0, 0, "key1 to task1");
			oled_clear_line(0, 1);
			oled_6x8_str(0, 1, "key2 to task2");
			oled_clear_line(0, 2);
			oled_clear_line(0, 3);
			oled_clear_line(0, 4);
			oled_clear_line(0, 5);
			oled_clear_line(0, 6);
			oled_clear_line(0, 7);
		//遥控器矫正显示页
		} else if(page_number == 14) {
			oled_clear_line(0, 0);
			oled_6x8_str(0, 0, "ch1:");
			oled_6x8_number(25, 0, 4, rc_raw_data[0]);
			oled_6x8_number(55, 0, 4, rc_calibration_data[0].max);
			oled_6x8_number(90, 0, 4, rc_calibration_data[0].min);
			oled_clear_line(0, 1);
			oled_6x8_str(0, 1, "ch2:");
			oled_6x8_number(25, 1, 4, rc_raw_data[1]);
			oled_6x8_number(55, 1, 4, rc_calibration_data[1].max);
			oled_6x8_number(90, 1, 4, rc_calibration_data[1].min);
			oled_clear_line(0, 2);
			oled_6x8_str(0, 2, "ch3:");
			oled_6x8_number(25, 2, 4, rc_raw_data[2]);
			oled_6x8_number(55, 2, 4, rc_calibration_data[2].max);
			oled_6x8_number(90, 2, 4, rc_calibration_data[2].min);
			oled_clear_line(0, 3);
			oled_6x8_str(0, 3, "ch4:");
			oled_6x8_number(25, 3, 4, rc_raw_data[3]);
			oled_6x8_number(55, 3, 4, rc_calibration_data[3].max);
			oled_6x8_number(90, 3, 4, rc_calibration_data[3].min);
			oled_clear_line(0, 4);
			oled_6x8_str(0, 4, "ch5:");
			oled_6x8_number(25, 4, 4, rc_raw_data[4]);
			oled_6x8_number(55, 4, 4, rc_calibration_data[4].max);
			oled_6x8_number(90, 4, 4, rc_calibration_data[4].min);
			oled_clear_line(0, 5);
			oled_6x8_str(0, 5, "ch6:");
			oled_6x8_number(25, 5, 4, rc_raw_data[5]);
			oled_6x8_number(55, 5, 4, rc_calibration_data[5].max);
			oled_6x8_number(90, 5, 4, rc_calibration_data[5].min);
			oled_clear_line(0, 6);
			oled_6x8_str(0, 6, "ch7:");
			oled_6x8_number(25, 6, 4, rc_raw_data[6]);
			oled_6x8_number(55, 6, 4, rc_calibration_data[6].max);
			oled_6x8_number(90, 6, 4, rc_calibration_data[6].min);
			oled_clear_line(0, 7);
			oled_6x8_str(0, 7, "ch8:");
			oled_6x8_number(25, 7, 4, rc_raw_data[7]);
			oled_6x8_number(55, 7, 4, rc_calibration_data[7].max);
			oled_6x8_number(90, 7, 4, rc_calibration_data[7].min);
		//电调校准显示页
		} else if(page_number == 15) {
			oled_clear_line(0, 0);
			oled_clear_line(0, 1);
			oled_8x16_str(0, 0, "Please Move Thr");
			oled_clear_line(0, 2);
			oled_clear_line(0, 3);
			oled_8x16_str(0, 2, "Down When ESC");
			oled_clear_line(0, 4);
			oled_clear_line(0, 5);
			oled_8x16_str(0, 4, "Beep Beep");
			oled_6x8_str(80, 4, "Thr:");
			oled_6x8_number(80, 5, 4, rc_raw_data[2]);
			oled_clear_line(0, 6);
			oled_6x8_str(0, 6, "Repower When Set Up");

			oled_clear_line(0, 7);
			oled_6x8_str(0, 7, "SysT:");
			oled_6x8_number(30, 7, 2, Time_Sys.hour);
			oled_6x8_str(45, 7, ":");
			oled_6x8_number(55, 7, 2, Time_Sys.minute);
			oled_6x8_str(70, 7, ":");
			oled_6x8_number(80, 7, 2, Time_Sys.second);
			oled_6x8_str(95, 7, ":");
			oled_6x8_number(105, 7, 2, Time_Sys.microsecond);
		//电调行程校准显示页
		} else if(page_number == 16) {
			oled_clear_line(0, 0);
			oled_clear_line(0, 1);
			oled_8x16_str(0, 0, "Please Move Thr");
			oled_clear_line(0, 2);
			oled_clear_line(0, 3);
			oled_8x16_str(0, 2, "Up And Pull Out");
			oled_clear_line(0, 4);
			oled_clear_line(0, 5);
			oled_8x16_str(0, 4, "The Power Supply");
			oled_clear_line(0, 6);
			oled_clear_line(0, 7);
			oled_8x16_str(0, 6, "ESC Calibration");
		//遥控器未连接
		} else if(page_number == 17) {
			oled_clear_line(0, 0);
			oled_clear_line(0, 1);
			oled_8x16_str(0, 0, "Remote Control");
			oled_clear_line(0, 2);
			oled_clear_line(0, 3);
			oled_8x16_str(0, 2, "Is Not Connect");
			oled_clear_line(0, 4);
			oled_clear_line(0, 5);
			oled_clear_line(0, 6);
			oled_clear_line(0, 7);
		//加速计校准界面
		} else if (page_number == 18) {
			oled_clear_line(0, 0);
			oled_6x8_str(10, 0, "Accel_Correct");
			oled_6x8_number(100, 0, 1, acce_calibration_flag + 1);
			oled_clear_line(0, 1);
			oled_6x8_number(0, 1, 6, acce_calibration_data[0].x);
			oled_6x8_number(42, 1, 6, acce_calibration_data[0].y);
			oled_6x8_number(84, 1, 6, acce_calibration_data[0].z);
			oled_clear_line(0, 2);
			oled_6x8_number(0, 2, 6, acce_calibration_data[1].x);
			oled_6x8_number(42, 2, 6, acce_calibration_data[1].y);
			oled_6x8_number(84, 2, 6, acce_calibration_data[1].z);
			oled_clear_line(0, 3);
			oled_6x8_number(0, 3, 6, acce_calibration_data[2].x);
			oled_6x8_number(42, 3, 6, acce_calibration_data[2].y);
			oled_6x8_number(84, 3, 6, acce_calibration_data[2].z);
			oled_clear_line(0, 4);
			oled_6x8_number(0, 4, 6, acce_calibration_data[3].x);
			oled_6x8_number(42, 4, 6, acce_calibration_data[3].y);
			oled_6x8_number(84, 4, 6, acce_calibration_data[3].z);
			oled_clear_line(0, 5);
			oled_6x8_number(0, 5, 6, acce_calibration_data[4].x);
			oled_6x8_number(42, 5, 6, acce_calibration_data[4].y);
			oled_6x8_number(84, 5, 6, acce_calibration_data[4].z);
			oled_clear_line(0, 6);
			oled_6x8_number(0, 6, 6, acce_calibration_data[5].x);
			oled_6x8_number(42, 6, 6, acce_calibration_data[5].y);
			oled_6x8_number(84, 6, 6, acce_calibration_data[5].z);
		}
		vTaskDelay(100);
	}
}

/**********************************************************************************************************
*函 数 名: display_task_create
*功能说明: 显示相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void display_task_create(void)
{
	display_task_exit = 0;
    xTaskCreate(display_task, "display_task", DISPLAY_TASK_STACK, NULL, DISPLAY_TASK_PRIORITY, &display_task_handle);
}
