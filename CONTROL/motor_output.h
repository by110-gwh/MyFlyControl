#ifndef _MOTOR_OUTPUT_H
#define _MOTOR_OUTPUT_H

#include <stdint.h>

extern uint16_t throttle_motor_output;
extern int16_t roll_motor_output;
extern int16_t pitch_motor_output;
extern int16_t yaw_motor_output;

void motor_output_init(void);
void motor_output_unlock(void);
void motor_output_output(void);
void motor_output_lock(void);

#endif
