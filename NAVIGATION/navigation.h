#ifndef _NAVIGATION_H
#define _NAVIGATION_H

#include "vector3.h"

extern float navigation_acce_length;
extern Vector3f_t navigation_acce;

void navigation_init(void);
void navigation_prepare(void);

#endif
