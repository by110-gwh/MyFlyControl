#ifndef _HORIZONTAL_CONTROL_H
#define _HORIZONTAL_CONTROL_H

#include "pid.h"

extern pid_data_t horizontal_pos_x_pid_data;
extern pid_data_t horizontal_speed_x_pid_data;
extern pid_data_t horizontal_pos_y_pid_data;
extern pid_data_t horizontal_speed_y_pid_data;

void horizontal_pos_x_control_pid_set(uint8_t p, uint8_t i, uint8_t d);
void horizontal_pos_y_control_pid_set(uint8_t p, uint8_t i, uint8_t d);
void horizontal_speed_x_control_pid_set(uint8_t p, uint8_t i, uint8_t d);
void horizontal_speed_y_control_pid_set(uint8_t p, uint8_t i, uint8_t d);
void horizontal_control_init(void);
void horizontal_pos_x_pid_integrate_reset(void);
void horizontal_pos_y_pid_integrate_reset(void);
void horizontal_speed_x_pid_integrate_reset(void);
void horizontal_speed_y_pid_integrate_reset(void);
void horizontal_control(void);
void horizontal_speed_to_angles(float horizontal_speed_x, float horizontal_speed_y, float *pitch_angle, float *roll_angle);

#endif
