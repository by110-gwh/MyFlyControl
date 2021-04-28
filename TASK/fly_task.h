#ifndef __FLY_TASK_H
#define __FLY_TASK_H

#include <stdint.h>

extern volatile uint8_t fly_task_exit;

void fly_task_create(void);

#endif
