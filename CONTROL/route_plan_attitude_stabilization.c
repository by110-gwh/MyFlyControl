#include "route_plan_attitude_stabilization.h"
#include "remote_control.h"
#include "motor_output.h"
#include "angle_control.h"
#include "gyro_control.h"
#include "ahrs_aux.h"
#include "navigation.h"
#include "high_control.h"
#include "horizontal_control.h"
#include "route_plan_task.h"
#include "controller.h"

#define HORIZONTAL_SPEED_MAX 250

//·���˳���־λ
uint8_t route_plan_finish; 
//����·��ʱ�����Ÿ˵�λ��
uint16_t save_throttle_control;
//�˳�·��Ҫ����߶�
uint16_t save_high_expect;
//·�����Զ�������ϱ�־
uint8_t route_plan_stop_flag;

uint16_t start_time_cnt;
/**********************************************************************************************************
*�� �� ��: route_plan_attitude_stabilization_init
*����˵��: ·����������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void route_plan_attitude_stabilization_init(void)
{
    start_time_cnt = 0;
    high_pos_pid_integrate_reset();
    high_speed_pid_integrate_reset();
    horizontal_pos_x_pid_integrate_reset();
    horizontal_pos_y_pid_integrate_reset();
    horizontal_speed_x_pid_integrate_reset();
    horizontal_speed_y_pid_integrate_reset();
    horizontal_pos_x_pid_data.short_circuit_flag = 1;
    horizontal_pos_y_pid_data.short_circuit_flag = 1;
}

/**********************************************************************************************************
*�� �� ��: route_plan_attitude_stabilization_control
*����˵��: ·��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void route_plan_attitude_stabilization_control(void)
{
    //�����ת3�뵽��ͣ����
    if (start_time_cnt < 3 * 200) {
        pitch_gyro_pid_data.control_output = 0;
        roll_gyro_pid_data.control_output = 0;
        yaw_gyro_pid_data.control_output = 0;
        throttle_motor_output = start_time_cnt * HOLD_THROTTLE / (3 * 200) + 1000;
        start_time_cnt++;
    //����һЩ����
    } else if (start_time_cnt == 3 * 200) {
        high_pos_pid_data.expect = 20;
        yaw_angle_pid_data.short_circuit_flag = 0;
        yaw_angle_pid_data.expect = Yaw;
        horizontal_pos_y_pid_data.short_circuit_flag = 0;
        horizontal_pos_y_pid_data.expect = pos_y;
        horizontal_pos_x_pid_data.short_circuit_flag = 0;
        horizontal_pos_x_pid_data.expect = pos_x;
        save_throttle_control = Throttle_Control;
        route_plan_finish = 0;
        route_plan_stop_flag = 0;
        route_plan_task_create();
        start_time_cnt++;
    } else {
        //���·���滮����
        if (route_plan_finish) {
            if (Pitch_Control == 0) {
                //����ʱ��yλ������ֵ
                if (horizontal_pos_y_pid_data.short_circuit_flag == 1) {
                    //����û���ٶ�
                    if (speed_y < 40 && speed_y > -40) {
                        horizontal_pos_y_pid_data.expect = pos_y;
                        //ʹ��ˮƽy����pid����
                        horizontal_pos_y_pid_data.short_circuit_flag = 0;
                    } else {
                        //ɲ������
                        horizontal_pos_y_pid_data.expect = 0;
                    }
                }
            //����ˮƽy����˺�ֻ�����ڻ����ٶȿ���
            } else {
                //�ر�ˮƽy����pid����
                horizontal_pos_y_pid_data.short_circuit_flag = 1;
                //ˮƽy����������0,������ˮƽy�������
                horizontal_pos_y_pid_data.expect = -Pitch_Control * HORIZONTAL_SPEED_MAX / Pit_Rol_Max;
            }
        } else if (Pitch_Control != 0) {
            //�ر�ˮƽy����pid����
            horizontal_pos_y_pid_data.short_circuit_flag = 1;
            //ˮƽy����������0,������ˮƽy�������
            horizontal_pos_y_pid_data.expect = -Pitch_Control * HORIZONTAL_SPEED_MAX / Pit_Rol_Max;
            //���ֵ�ǰ���Ÿ�λ�ú͵�ǰ�߶�
            save_throttle_control = Throttle_Control;
            save_high_expect = high_pos_pid_data.expect;
            //�˳�·���滮
            route_plan_task_delete();
            route_plan_finish = 1;
        } else {
        
        }
        
        //���·���滮����
        if (route_plan_finish) {
            if (Roll_Control == 0) {
                //����ʱ��yλ������ֵ
                if (horizontal_pos_x_pid_data.short_circuit_flag == 1) {
                    //����û���ٶ�
                    if (speed_x < 40 && speed_x > -40) {
                        horizontal_pos_x_pid_data.expect = pos_x;
                        //ʹ��ˮƽx����pid����
                        horizontal_pos_x_pid_data.short_circuit_flag = 0;
                    } else {
                        //ɲ������
                        horizontal_pos_x_pid_data.expect = 0;
                    }
                }
            //����ˮƽx����˺�ֻ�����ڻ����ٶȿ���
            } else {
                //�ر�ˮƽx����pid����
                horizontal_pos_x_pid_data.short_circuit_flag = 1;
                //ˮƽx����������0,������ˮƽx�������
                horizontal_pos_x_pid_data.expect = Roll_Control * HORIZONTAL_SPEED_MAX / Pit_Rol_Max;
            }
        } else if (Roll_Control != 0) {
            //�ر�ˮƽx����pid����
            horizontal_pos_x_pid_data.short_circuit_flag = 1;
            //ˮƽx����������0,������ˮƽx�������
            horizontal_pos_x_pid_data.expect = Roll_Control * HORIZONTAL_SPEED_MAX / Pit_Rol_Max;
            //���ֵ�ǰ���Ÿ�λ�ú͵�ǰ�߶�
            save_throttle_control = Throttle_Control;
            save_high_expect = high_pos_pid_data.expect;
            //�˳�·���滮
            route_plan_task_delete();
            route_plan_finish = 1;
        } else {
        
        }

        //���·���滮����
        if (route_plan_finish) {
            //ƫ����������λ
            if (Yaw_Control == 0) {
                //����ʱ���Ƕ�����ֵ
                if (yaw_angle_pid_data.short_circuit_flag == 1) {
                    yaw_angle_pid_data.expect = Yaw;
                    //ʹ��ƫ��pid����
                    yaw_angle_pid_data.short_circuit_flag = 0;
                }
            //����ƫ������˺�ֻ�����ڻ����ٶȿ���
            } else {
                //�ر�ƫ��pid����
                yaw_angle_pid_data.short_circuit_flag = 1;
                //ƫ����������0,�����нǶȿ���
                yaw_angle_pid_data.expect = Yaw_Control;
            }
        } else if (Yaw_Control != 0) {
            //�ر�ƫ��pid����
            yaw_angle_pid_data.short_circuit_flag = 1;
            //ƫ����������0,�����нǶȿ���
            yaw_angle_pid_data.expect = Yaw_Control;
            //���ֵ�ǰ���Ÿ�λ�ú͵�ǰ�߶�
            save_throttle_control = Throttle_Control;
            save_high_expect = high_pos_pid_data.expect;
            //�˳�·���滮
            route_plan_task_delete();
            route_plan_finish = 1;
        } else {
        
        }
        
        //���·���滮����
        if (route_plan_finish) {
            //�������������Ÿ�����λ��
            high_pos_pid_data.expect = (Throttle_Control - save_throttle_control) / (float)(1000 - 500) * 200 + save_high_expect;
        //ִ��·���в��������Ÿ�
        } else if (save_throttle_control - Throttle_Control > 50 || 
            Throttle_Control - save_throttle_control > 50) {
            //���ֵ�ǰ���Ÿ�λ�ú͵�ǰ�߶�
            save_throttle_control = Throttle_Control;
            save_high_expect = high_pos_pid_data.expect;
            //�˳�·���滮
            route_plan_task_delete();
            route_plan_finish = 1;
        } else {
            
        }
        
        //���·���Զ��������
        if (route_plan_stop_flag) {
            //���ͣת
            throttle_motor_output = 1000;
            yaw_gyro_pid_data.control_output = 0;
            pitch_gyro_pid_data.control_output = 0;
            roll_gyro_pid_data.control_output = 0;
        } else {
            //λ�ÿ�����
            horizontal_control();
            //�߶Ȼ�������
            high_control();
            //�ǶȻ�������
            angle_control();
            //���ٶȿ�����
            gyro_control();
            //���Ų���
            throttle_motor_output = throttle_angle_compensate(high_speed_pid_data.control_output + HOLD_THROTTLE + 1000);
        }
    }
}
