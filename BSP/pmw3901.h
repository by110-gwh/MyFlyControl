#ifndef __PMW3901_H
#define __PMW3901_H

#include <stdint.h>

//1m高度下 1个像素对应的位移，单位cm
#define OPTICAL_SCALS 0.2131946f

void pmw3901_init(void);
void pmw3901_read_motion(int16_t *dx, int16_t *dy, uint16_t *qual);

#endif
