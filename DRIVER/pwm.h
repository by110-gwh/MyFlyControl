#ifndef _PWM_H
#define _PWM_H

#include <stdint.h>

void pwm_init(void);
void pwm_set(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4);

#endif
