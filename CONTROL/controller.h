#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include <stdint.h>

#define HOLD_THROTTLE 500

void controller_init(void);
void controller_run(void);
uint16_t throttle_angle_compensate(uint16_t throttle);

#endif
