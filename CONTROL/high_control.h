#ifndef _HIGH_CONTROL_H
#define _HIGH_CONTROL_H

#include "pid.h"

extern pid_controler_t high_pos_pid;
extern pid_controler_t high_vel_pid;

void high_control_init(void);
void high_pos_pid_integrate_reset(void);
void high_vel_pid_integrate_reset(void);
void high_control(void);

#endif
