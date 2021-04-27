#ifndef _MOTOR_OUTPUT_H
#define _MOTOR_OUTPUT_H

#include <stdint.h>

void motor_output_init(void);
void motor_output_unlock(void);
void motor_output_output(void);
void motor_output_lock(void);

#endif
