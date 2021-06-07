#ifndef _NAVIGATION_H
#define _NAVIGATION_H

#include "vector3.h"

extern float navigation_acce_length;
extern Vector3f_t navigation_acce;

extern float high_acce;
extern float high_pos;
extern float high_vel;

void navigation_init(void);
void navigation_prepare(void);
void high_kalman_filter(void);

#endif
