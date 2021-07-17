#ifndef __SAFE_TASK_H
#define __SAFE_TASK_H

#include <stdint.h>

extern volatile uint8_t safe_task_exit;

void safe_task_motor_stop(void);
void safe_task_create(void);

#endif
