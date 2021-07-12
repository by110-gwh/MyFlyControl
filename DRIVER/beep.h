#ifndef __BEEP_H
#define __BEEP_H

#include <stdint.h>

void beep_init(void);
void beep_control(uint8_t is_on);

#endif
