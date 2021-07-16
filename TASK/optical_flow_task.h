#ifndef __OPTICAL_FLOW_TASK_H
#define __OPTICAL_FLOW_TASK_H

#include <stdint.h>

extern float optical_flow_pos_x;
extern float optical_flow_pos_y;
extern float optical_flow_speed_x;
extern float optical_flow_speed_y;
extern volatile uint8_t optical_flow_task_exit;

void optical_flow_task_motor_stop(void);
void optical_flow_task_create(void);

#endif
