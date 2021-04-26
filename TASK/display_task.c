#include "display_task.h"
#include "oled.h"
#include "ppm.h"
#include "remote_control.h"
#include "time_cnt.h"
#include "imu.h"
#include "ahrs_aux.h"

#include "FreeRTOS.h"
#include "task.h"

//任务堆栈大小
#define DISPLAY_TASK_STACK            256
//任务优先级
#define DISPLAY_TASK_PRIORITY         6

//声明任务句柄
static xTaskHandle display_task_handle;
//任务退出标志
uint8_t display_task_exit;
//当前页
int16_t page_number;

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
		//遥控器矫正显示页
		if(page_number == 1) {
			oled_clear_line(0, 0);
			oled_6x8_str(0, 0, (uint8_t *)"Basic");
			oled_6x8_number(70, 0, page_number+1);
			oled_clear_line(0, 1);
			oled_6x8_str(0, 1, (uint8_t *)"Yaw:");
			oled_6x8_number(40, 1, Yaw);
			oled_6x8_number(90, 1, gyroDataFilter.z * GYRO_CALIBRATION_COFF);
			oled_clear_line(0, 2);
			oled_6x8_str(0, 2, (uint8_t *)"Pitch:");
			oled_6x8_number(40, 2, Pitch);
			oled_6x8_number(90, 2, gyroDataFilter.x * GYRO_CALIBRATION_COFF);
			oled_clear_line(0, 3);
			oled_6x8_str(0, 3, (uint8_t *)"Roll:");
			oled_6x8_number(40, 3, Roll);
			oled_6x8_number(90, 3, gyroDataFilter.y * GYRO_CALIBRATION_COFF);
			oled_clear_line(0, 4);
		} else if(page_number == 14) {
			oled_clear_line(0, 0);
			oled_6x8_str(0, 0, (uint8_t *)"ch1:");
			oled_6x8_number(25, 0, rc_raw_data[0]);
			oled_6x8_number(55, 0, rc_calibration_data[0].max);
			oled_6x8_number(90, 0, rc_calibration_data[0].min);
			oled_clear_line(0, 1);
			oled_6x8_str(0, 1, (uint8_t *)"ch2:");
			oled_6x8_number(25, 1, rc_raw_data[1]);
			oled_6x8_number(55, 1, rc_calibration_data[1].max);
			oled_6x8_number(90, 1, rc_calibration_data[1].min);
			oled_clear_line(0, 2);
			oled_6x8_str(0, 2, (uint8_t *)"ch3:");
			oled_6x8_number(25, 2, rc_raw_data[2]);
			oled_6x8_number(55, 2, rc_calibration_data[2].max);
			oled_6x8_number(90, 2, rc_calibration_data[2].min);
			oled_clear_line(0, 3);
			oled_6x8_str(0, 3, (uint8_t *)"ch4:");
			oled_6x8_number(25, 3, rc_raw_data[3]);
			oled_6x8_number(55, 3, rc_calibration_data[3].max);
			oled_6x8_number(90, 3, rc_calibration_data[3].min);
			oled_clear_line(0, 4);
			oled_6x8_str(0, 4, (uint8_t *)"ch5:");
			oled_6x8_number(25, 4, rc_raw_data[4]);
			oled_6x8_number(55, 4, rc_calibration_data[4].max);
			oled_6x8_number(90, 4, rc_calibration_data[4].min);
			oled_clear_line(0, 5);
			oled_6x8_str(0, 5, (uint8_t *)"ch6:");
			oled_6x8_number(25, 5, rc_raw_data[5]);
			oled_6x8_number(55, 5, rc_calibration_data[5].max);
			oled_6x8_number(90, 5, rc_calibration_data[5].min);
			oled_clear_line(0, 6);
			oled_6x8_str(0, 6, (uint8_t *)"ch7:");
			oled_6x8_number(25, 6, rc_raw_data[6]);
			oled_6x8_number(55, 6, rc_calibration_data[6].max);
			oled_6x8_number(90, 6, rc_calibration_data[6].min);
			oled_clear_line(0, 7);
			oled_6x8_str(0, 7, (uint8_t *)"ch8:");
			oled_6x8_number(25, 7, rc_raw_data[7]);
			oled_6x8_number(55, 7, rc_calibration_data[7].max);
			oled_6x8_number(90, 7, rc_calibration_data[7].min);
		//电调校准显示页
		} else if(page_number == 15) {
			oled_clear_line(0, 0);
			oled_clear_line(0, 1);
			oled_8x16_str(0, 0, (uint8_t *)"Please Move Thr");
			oled_clear_line(0, 2);
			oled_clear_line(0, 3);
			oled_8x16_str(0, 2, (uint8_t *)"Down When ESC");
			oled_clear_line(0, 4);
			oled_clear_line(0, 5);
			oled_8x16_str(0, 4, (uint8_t *)"Beep Beep");
			oled_6x8_str(80, 4, (uint8_t *)"Thr:");
			oled_6x8_number(80, 5, rc_raw_data[2]);
			oled_clear_line(0, 6);
			oled_6x8_str(0, 6, (uint8_t *)"Repower When Set Up");

			oled_clear_line(0, 7);
			oled_6x8_str(0, 7, (uint8_t *)"SysT:");
			oled_6x8_number(30, 7, Time_Sys.hour);
			oled_6x8_str(45, 7, (uint8_t *)":");
			oled_6x8_number(55, 7, Time_Sys.minute);
			oled_6x8_str(70, 7, (uint8_t *)":");
			oled_6x8_number(80, 7, Time_Sys.second);
			oled_6x8_str(95, 7, (uint8_t *)":");
			oled_6x8_number(105, 7, Time_Sys.microsecond);
		//电调行程校准显示页
		} else if(page_number == 16) {
			oled_clear_line(0, 0);
			oled_clear_line(0, 1);
			oled_8x16_str(0, 0, (uint8_t *)"Please Move Thr");
			oled_clear_line(0, 2);
			oled_clear_line(0, 3);
			oled_8x16_str(0, 2, (uint8_t *)"Up And Pull Out");
			oled_clear_line(0, 4);
			oled_clear_line(0, 5);
			oled_8x16_str(0, 4, (uint8_t *)"The Power Supply");
			oled_clear_line(0, 6);
			oled_clear_line(0, 7);
			oled_8x16_str(0, 6, (uint8_t *)"ESC Calibration");
		//遥控器未连接
		} else if(page_number == 17) {
			oled_clear_line(0, 0);
			oled_clear_line(0, 1);
			oled_8x16_str(0, 0, (uint8_t *)"Remote Control");
			oled_clear_line(0, 2);
			oled_clear_line(0, 3);
			oled_8x16_str(0, 2, (uint8_t *)"Is Not Connect");
			oled_clear_line(0, 4);
			oled_clear_line(0, 5);
			oled_clear_line(0, 6);
			oled_clear_line(0, 7);
		//加速计校准界面
		} else if (page_number == 18) {
			oled_clear_line(0, 0);
			oled_6x8_str(10, 0, (uint8_t *)"Accel_Correct");
			oled_6x8_number(100, 0, acce_calibration_flag + 1);
			oled_6x8_number(110, 0, page_number + 1);
			oled_clear_line(0, 1);
			oled_6x8_number(0, 1, acce_calibration_data[0].x);
			oled_6x8_number(42, 1, acce_calibration_data[0].y);
			oled_6x8_number(84, 1, acce_calibration_data[0].z);
			oled_clear_line(0, 2);
			oled_6x8_number(0, 2, acce_calibration_data[1].x);
			oled_6x8_number(42, 2, acce_calibration_data[1].y);
			oled_6x8_number(84, 2, acce_calibration_data[1].z);
			oled_clear_line(0, 3);
			oled_6x8_number(0, 3, acce_calibration_data[2].x);
			oled_6x8_number(42, 3, acce_calibration_data[2].y);
			oled_6x8_number(84, 3, acce_calibration_data[2].z);
			oled_clear_line(0, 4);
			oled_6x8_number(0, 4, acce_calibration_data[3].x);
			oled_6x8_number(42, 4, acce_calibration_data[3].y);
			oled_6x8_number(84, 4, acce_calibration_data[3].z);
			oled_clear_line(0, 5);
			oled_6x8_number(0, 5, acce_calibration_data[4].x);
			oled_6x8_number(42, 5, acce_calibration_data[4].y);
			oled_6x8_number(84, 5, acce_calibration_data[4].z);
			oled_clear_line(0, 6);
			oled_6x8_number(0, 6, acce_calibration_data[5].x);
			oled_6x8_number(42, 6, acce_calibration_data[5].y);
			oled_6x8_number(84, 6, acce_calibration_data[5].z);
		//磁力计校准界面
		} else if (page_number == 19) {
			uint8_t i;
			oled_clear_line(0, 0);
			oled_6x8_str(10, 0, (uint8_t *)"Mag_Correct");
			oled_6x8_number(90, 0, mag_calibration_flag);
			oled_6x8_number(100, 0, page_number + 1);
//			oled_clear_line(0, 1);
//			oled_6x8_number(0, 1, Mag.x_offset);
//			oled_6x8_number(40, 1, Mag.y_offset);
//			oled_6x8_number(70, 1, Mag.z_offset);
			oled_clear_line(0, 2);
			oled_6x8_str(0, 2, (uint8_t *)"0 To 360");
			oled_6x8_number(70, 2, mag_correct_yaw);
			oled_clear_line(0, 3);
//			oled_6x8_str(0, 3, (uint8_t *)"Mag Is Okay?");
//			oled_6x8_number(80, 3, Mag_Is_Okay_Flag[0]);
//			oled_6x8_number(90, 3, Mag_Is_Okay_Flag[1]);
//			oled_6x8_number(105, 3, Mag_Is_Okay_Flag[2]);
			oled_clear_line(0, 4);
			for(i = 0; i < 12; i++) {
			  oled_6x8_number(10*i, 4, mag_360_flag[0][3*i]);
			}
			oled_clear_line(0, 5);
			for(i = 0; i < 12; i++) {
			  oled_6x8_number(10*i, 5, mag_360_flag[1][3*i]);
			}
			oled_clear_line(0, 6);
			for(i = 0; i < 12; i++) {
			  oled_6x8_number(10*i, 6, mag_360_flag[2][3*i]);
			}
			oled_clear_line(0, 7);
			if(mag_calibration_flag == 0)
				oled_6x8_str(0, 7, (uint8_t *)"Make Z+ Upside Sky");
			else if(mag_calibration_flag == 1)
				oled_6x8_str(0, 7, (uint8_t *)"Make Y+ Upside Sky");
			else if(mag_calibration_flag == 2)
				oled_6x8_str(0, 7, (uint8_t *)"Make X+ Upside Sky");
			else if(mag_calibration_flag == 3)
				oled_6x8_str(0, 7, (uint8_t *)"Start With Yaw Move");
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
