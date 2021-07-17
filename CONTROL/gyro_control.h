#ifndef _GYRO_CONTROL_H
#define _GYRO_CONTROL_H

#include "pid.h"

extern pid_data_t pitch_gyro_pid_data;
extern pid_data_t roll_gyro_pid_data;
extern pid_data_t yaw_gyro_pid_data;

void gyro_control_init(void);
void gyro_pid_integrate_reset(void);
void gyro_control(void);

#endif
