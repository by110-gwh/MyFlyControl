#ifndef _SR04_H
#define _SR04_H

#include <stdint.h>

extern volatile uint8_t sr04_task_exit;
extern uint16_t high_raw_data;

void sr04_task_create(void);
void sr04_exit_callback(void);

#endif
