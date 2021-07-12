#ifndef __BEEP_TASK_H
#define __BEEP_TASK_H

#include <stdint.h>

extern uint8_t beep_duty;
extern uint8_t beep_cycle;
extern uint8_t beep_time;

void beep_task_motor_stop(void);
void beep_task_create(void);

#endif
