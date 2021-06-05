#ifndef _VL53L1X_H
#define	_VL53L1X_H

#include <stdint.h>

extern uint16_t high_raw_data;

void vl53l1x_init(void);
void vl53l1x_task(void);

#endif
