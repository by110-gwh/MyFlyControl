#include "main_task.h"
#include "remote_control.h"
#include "key.h"
#include "safe_task.h"
#include "display_task.h"
#include "fly_task.h"
#include "esc_task.h"
#include "usmart_task.h"
#include "beep_task.h"
#include "imu.h"
#include "i2c.h"
#include "i2c1.h"
#include "spi0.h"
#include "spi1.h"
#include "uart7.h"
#include "w25qxx.h"
#include "motor_output.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "controller.h"
#include "openmv.h"
#include "sr04.h"

#include "FreeRTOS.h"
#include "task.h"

//任务堆栈大小
#define MAIN_TASK_STACK            512
//任务优先级
#define MAIN_TASK_PRIORITY         5

//声明任务句柄
xTaskHandle main_task_handle;
//任务退出标志
volatile uint8_t main_task_exit;
uint8_t fly_task_num;

/**********************************************************************************************************
*函 数 名: fly_enter
*功能说明: 进入飞行任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void fly_enter(void)
{
    //陀螺仪校准
    page_number = 2;
    gyro_calibration();
    //飞行任务创建
    page_number = 3;
    beep_duty = 5;
    beep_cycle = 100;
    beep_time = 0xFF;
    fly_task_create();
    vTaskDelay(2000);
    page_number = 4;
    safe_task_create();
    //清PID积分
    controller_init();
    //电机解锁
    motor_output_unlock();
    //等待电机启动时间姿态融合完毕后，更新偏航期待
    yaw_angle_pid_data.short_circuit_flag = 1;
    page_number = 1;
}

/**********************************************************************************************************
*函 数 名: fly_exit
*功能说明: 退出飞行任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void fly_exit(void)
{
    safe_task_exit = 1;
    fly_task_exit = 1;
    vTaskDelay(5);
    motor_output_lock();
    beep_time = 0;
    page_number = 0;
}

/**********************************************************************************************************
*函 数 名: main_task
*功能说明: 主任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(main_task, parameters)
{
	key_init();
	rc_init();
	i2c_init();
	i2c1_init();
    spi0_init();
    spi1_init();
    w25qxx_init();
	motor_output_init();
	usmart_task_create();
    beep_task_create();
    uart7_init();
    openmv_init();
    sr04_init();
    
	//检测遥控器是否连接
	page_number = 17;
	while (!rc_is_on())
		vTaskDelay(1);
    while (!main_task_exit) {
        uint8_t key;
        page_number = 0;
		key = key_scan();
		//key0长按进行遥控器行程校准
		if ((key & (KEY0 << 1)) && key & 1) {
			page_number = 14;
			rc_calibration_task();
			page_number = 0;
		} else if ((key & (KEY1 << 1)) && key & 1) {
            uint32_t i = 5 * 100;
            while (i--) {
                page_number = 5;
                key = key_scan();
                //通过按键确定执行哪个任务
                if ((key & (KEY0 << 1)) && key & 1) {
                    fly_task_num = 1;
                }
                if ((key & (KEY1 << 1)) && key & 1) {
                    fly_task_num = 2;
                }
                //启动飞行任务
                if (((key & (KEY0 << 1))  && key & 1) || ((key & (KEY1 << 1)) && key & 1)) {
                    vTaskDelay(3000);
                    fly_enter();
                    while(1) {
                        key = rc_scan();
                        if (key == 0x08) {
                            break;
                        }
                        vTaskDelay(50);
                    }
                    fly_exit();
                    break;
                }
                vTaskDelay(10);
            }
        }
		
		key = rc_scan();
		//上内八进行电调校准
		if (key == 0x0E) {
			page_number = 16;
			esc_calibration();
		//下外八进行加速计校准
		} else if (key == 0x01) {
			page_number = 18;
			accel_calibration();
			page_number = 0;
		//上外八进行磁力计校准
		} else if (key == 0x07) {
			page_number = 19;
			mag_calibration();
			page_number = 0;
		//下内八进行解锁
		} else if (key == 0x08) {
            //等到遥控器方向遥感归位
            while (rc_direct_is_reset() == 0);
			fly_enter();
			while(1) {
				key = rc_scan();
				if (key == 0x08) {
					break;
				}
				vTaskDelay(50);
			}
            fly_exit();
		}
		vTaskDelay(6);
    }
	vTaskDelete(NULL);
}

/**********************************************************************************************************
*函 数 名: main_task_create
*功能说明: 主函数相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void main_task_create(void)
{
	main_task_exit = 0;
    xTaskCreate(main_task, "main_task", MAIN_TASK_STACK, NULL, MAIN_TASK_PRIORITY, &main_task_handle);
}
