#ifndef __LASER_TASK_H
#define __LASER_TASK_H

#include <stdint.h>

extern uint8_t laser_duty;
extern uint16_t laser_cycle;
extern uint8_t laser_time;

extern volatile uint8_t laser_task_exit;

void laser_task_create(void);

#endif
