#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include <stdint.h>

extern uint8_t controller_state;

void controller_init(void);
void controller_run(void);

#endif
