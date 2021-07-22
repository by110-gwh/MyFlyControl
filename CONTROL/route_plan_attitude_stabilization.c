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

#define HORIZONTAL_SPEED_MAX 250

//路径退出标志位
uint8_t route_plan_finish; 
//进入路径时的油门杆的位置
uint16_t save_throttle_control;
//退出路径要保存高度
uint16_t save_high_expect;
//路径已自动运行完毕标志
uint8_t route_plan_stop_flag;

/**********************************************************************************************************
*函 数 名: route_plan_attitude_stabilization_control
*功能说明: 路径控制器
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void route_plan_attitude_stabilization_control()
{
    //如果路径规划结束
    if (route_plan_finish) {
        if (Pitch_Control == 0) {
            //回中时赋y位置期望值
            if (horizontal_pos_y_pid_data.short_circuit_flag == 1) {
                //基本没有速度
                if (speed_y < 40 && speed_y > -40) {
                    horizontal_pos_y_pid_data.expect = pos_y;
                    //使能水平y方向pid计算
                    horizontal_pos_y_pid_data.short_circuit_flag = 0;
                } else {
                    //刹车控制
                    horizontal_pos_y_pid_data.expect = 0;
                }
            }
        //波动水平y方向杆后，只进行内环角速度控制
        } else {
            //关闭水平y方向pid计算
            horizontal_pos_y_pid_data.short_circuit_flag = 1;
            //水平y方向期望给0,不进行水平y方向控制
            horizontal_pos_y_pid_data.expect = -Pitch_Control * HORIZONTAL_SPEED_MAX / Pit_Rol_Max;
        }
    } else if (Pitch_Control != 0) {
        //关闭水平y方向pid计算
        horizontal_pos_y_pid_data.short_circuit_flag = 1;
        //水平y方向期望给0,不进行水平y方向控制
        horizontal_pos_y_pid_data.expect = -Pitch_Control * HORIZONTAL_SPEED_MAX / Pit_Rol_Max;
        //保持当前油门杆位置和当前高度
        save_throttle_control = Throttle_Control;
        save_high_expect = high_pos_pid_data.expect;
        //退出路径规划
        route_plan_task_delete();
        route_plan_finish = 1;
    } else {
    
    }
    
    //如果路径规划结束
    if (route_plan_finish) {
        if (Roll_Control == 0) {
            //回中时赋y位置期望值
            if (horizontal_pos_x_pid_data.short_circuit_flag == 1) {
                //基本没有速度
                if (speed_x < 40 && speed_x > -40) {
                    horizontal_pos_x_pid_data.expect = pos_x;
                    //使能水平x方向pid计算
                    horizontal_pos_x_pid_data.short_circuit_flag = 0;
                } else {
                    //刹车控制
                    horizontal_pos_x_pid_data.expect = 0;
                }
            }
        //波动水平x方向杆后，只进行内环角速度控制
        } else {
            //关闭水平x方向pid计算
            horizontal_pos_x_pid_data.short_circuit_flag = 1;
            //水平x方向期望给0,不进行水平x方向控制
            horizontal_pos_x_pid_data.expect = Roll_Control * HORIZONTAL_SPEED_MAX / Pit_Rol_Max;
        }
    } else if (Roll_Control != 0) {
        //关闭水平x方向pid计算
        horizontal_pos_x_pid_data.short_circuit_flag = 1;
        //水平x方向期望给0,不进行水平x方向控制
        horizontal_pos_x_pid_data.expect = Roll_Control * HORIZONTAL_SPEED_MAX / Pit_Rol_Max;
        //保持当前油门杆位置和当前高度
        save_throttle_control = Throttle_Control;
        save_high_expect = high_pos_pid_data.expect;
        //退出路径规划
        route_plan_task_delete();
        route_plan_finish = 1;
    } else {
    
    }

    //如果路径规划结束
    if (route_plan_finish) {
        //偏航杆置于中位
        if (Yaw_Control == 0) {
            //回中时赋角度期望值
            if (yaw_angle_pid_data.short_circuit_flag == 1) {
                yaw_angle_pid_data.expect = Yaw;
                //使能偏航pid计算
                yaw_angle_pid_data.short_circuit_flag = 0;
            }
        //波动偏航方向杆后，只进行内环角速度控制
        } else {
            //关闭偏航pid计算
            yaw_angle_pid_data.short_circuit_flag = 1;
            //偏航角期望给0,不进行角度控制
            yaw_angle_pid_data.expect = Yaw_Control;
        }
    } else if (Yaw_Control != 0) {
        //关闭偏航pid计算
        yaw_angle_pid_data.short_circuit_flag = 1;
        //偏航角期望给0,不进行角度控制
        yaw_angle_pid_data.expect = Yaw_Control;
        //保持当前油门杆位置和当前高度
        save_throttle_control = Throttle_Control;
        save_high_expect = high_pos_pid_data.expect;
        //退出路径规划
        route_plan_task_delete();
        route_plan_finish = 1;
    } else {
    
    }
    
    //如果路径规划结束
    if (route_plan_finish) {
        //油门来自于油门杆上下位移
        high_pos_pid_data.expect = (Throttle_Control - save_throttle_control) / (float)(1000 - 500) * 200 + save_high_expect;
    //执行路径中操纵了油门杆
    } else if (save_throttle_control - Throttle_Control > 50 || 
        Throttle_Control - save_throttle_control > 50) {
        //保持当前油门杆位置和当前高度
        save_throttle_control = Throttle_Control;
        save_high_expect = high_pos_pid_data.expect;
        //退出路径规划
        route_plan_task_delete();
        route_plan_finish = 1;
    } else {
        
    }
    
}
