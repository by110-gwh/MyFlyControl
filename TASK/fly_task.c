#include "fly_task.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "ppm.h"
#include "imu.h"
#include "vl53l1x.h"
#include "remote_control.h"
#include "controller.h"
#include "motor_output.h"
#include "optical_flow_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stdio.h"

//�����ջ��С
#define FLY_TASK_STACK 512
//�������ȼ�
#define FLY_TASK_PRIORITY 13

//����������
xTaskHandle fly_task_handle;
//�����˳���־
volatile uint8_t fly_task_exit = 1;
//����������±�־�����ڰ�ȫ���
volatile uint8_t fly_task_updata;
#include "gyro_control.h"
#include "angle_control.h"
#include "high_control.h"
#include "horizontal_control.h"
#include "openmv.h"
#include "sr04.h"
/**********************************************************************************************************
*�� �� ��: fly_task
*����˵��: ���ж�ʱ����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
#include <math.h>
portTASK_FUNCTION(fly_task, pvParameters)
{
    portTickType xLastWakeTime;
	
    optical_flow_task_create();
	imu_init();
	ahrs_init();
    controller_init();
    vl53l1x_init();
    navigation_init();

    xLastWakeTime = xTaskGetTickCount();
    while (!fly_task_exit) {
		//��ȡimu����
		get_imu_data();
        //��ȡ�߶�
        vl53l1x_task();
        //��̬����
		ahrs_update();
        //�������ٶȼ���
        navigation_prepare();
        //�߶�λ�ù���
        high_filter();
        pos_filter();
        //������
        controller_run();
        //���������
		motor_output_output();
        
        static uint8_t aa;
        if (aa == 3) {
//            printf("%0.1f, %0.1f, %0.1f,",
//                acce_z, speed_z, pos_z);
            printf("%0.1f,%0.1f,%0.1f,%0.1f,%0.1f,",
                high_pos_pid_data.expect,
                high_pos_pid_data.feedback,
                high_speed_pid_data.expect,
                high_speed_pid_data.feedback,
                high_speed_pid_data.control_output); 
//            printf("%0.1f,%0.1f,%0.1f,%0.1f,",
//                high_pos_pid_data.expect,
//                horizontal_pos_x_pid_data.expect,
//                horizontal_pos_y_pid_data.expect,
//                yaw_angle_pid_data.expect);
            aa++;
        } else if (aa == 6) {
            extern float tof_speed_err_z;
            extern float tof_pos_err_z;
//            printf("%0.1f, %0.1f, %0.1f, %0.1f\r\n",
//                tof_speed_err_z, tof_pos_err_z, high_speed_raw_data, high_raw_data);
            printf("%0.1f,%0.1f,%0.1f,%0.1f,%0.1f\r\n",
                yaw_angle_pid_data.expect,
                yaw_angle_pid_data.feedback,
                yaw_gyro_pid_data.expect,
                yaw_gyro_pid_data.feedback,
                yaw_gyro_pid_data.control_output); 
//            printf("%d\r\n", throttle_motor_output);
//            printf("%d,%d,%d\r\n",
//                pole_distance,
//                throttle_motor_output,
//                sr04_distance);
            aa = 1;
        } else {
            aa++;
        }
//        printf("%0.3f,%0.3f,%0.3f,%0.3f\r\n", Pitch, Roll, (float)atan2(accDataFilter.y, accDataFilter.z) * 57.3, -(float)atan2(accDataFilter.x, accDataFilter.z) * 57.3);
//        printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n", speed_z, high_raw_data / 10., pos_z, acce_z, navigation_acce.z); 
//        printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n", high_pos_pid_data.expect, high_pos_pid_data.feedback, high_speed_pid_data.expect, high_speed_pid_data.feedback, high_speed_pid_data.control_output); 
//        printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n", horizontal_pos_y_pid_data.expect, horizontal_pos_y_pid_data.feedback, horizontal_speed_y_pid_data.expect,
//            horizontal_speed_y_pid_data.feedback, horizontal_speed_y_pid_data.control_output, pitch_angle_pid_data.expect); 
//        printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n", navigation_acce.y / 10, speed_y, pos_y, optical_flow_pos_y, optical_flow_speed_y); 
//        printf("%0.3f\r\n", yaw_angle_pid_data.feedback); 
        //printf("%d\r\n", throttle_motor_output);
        fly_task_updata = 1;
        //˯��5ms
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
    optical_flow_task_exit = 1;
    vTaskDelete(NULL);
}


/**********************************************************************************************************
*�� �� ��: fly_task_create
*����˵��: ���������������񴴽�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void fly_task_create(void)
{
	fly_task_exit = 0;
    xTaskCreate(fly_task, "fly_task", FLY_TASK_STACK, NULL, FLY_TASK_PRIORITY, &fly_task_handle);
}
