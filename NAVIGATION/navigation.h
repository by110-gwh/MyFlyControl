#ifndef _NAVIGATION_H
#define _NAVIGATION_H

#include "vector3.h"

extern float navigation_acce_length;
extern Vector3f_t navigation_acce;

extern float pos_x, pos_y, pos_z;
extern float speed_x, speed_y, speed_z;
extern float acce_x, acce_y, pos_z;

void navigation_init(void);
void navigation_prepare(void);
void high_kalman_filter(void);
void pos_filter(void);

#endif
