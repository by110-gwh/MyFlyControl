#ifndef _GYRO_CONTROL_H
#define _GYRO_CONTROL_H

#include "pid.h"

extern pid_controler_t pitch_gyro_pid;
extern pid_controler_t roll_gyro_pid;
extern pid_controler_t yaw_gyro_pid;

void gyro_control_init(void);
void gyro_control(void);

#endif
