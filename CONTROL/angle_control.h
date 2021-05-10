#ifndef _ANGLE_CONTROL_H
#define _ANGLE_CONTROL_H

#include "pid.h"

extern pid_controler_t pitch_angle_pid;
extern pid_controler_t roll_angle_pid;
extern pid_controler_t yaw_angle_pid;

void angle_pid_integrate_reset(void);
void angle_control_init(void);
void angle_control(void);

#endif
