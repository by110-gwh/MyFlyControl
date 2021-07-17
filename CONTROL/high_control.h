#ifndef _HIGH_CONTROL_H
#define _HIGH_CONTROL_H

#include "pid.h"

extern pid_data_t high_pos_pid_data;
extern pid_data_t high_speed_pid_data;

void high_control_init(void);
void high_pos_pid_integrate_reset(void);
void high_speed_pid_integrate_reset(void);
void high_control(void);

#endif
