#ifndef _ROUTE_PLAN_ATTITUDE_STABILIZATION_H
#define _ROUTE_PLAN_ATTITUDE_STABILIZATION_H

#include <stdint.h>

extern uint8_t route_plan_finish; 
extern uint16_t save_high_expect;
extern uint16_t save_throttle_control;
extern uint8_t route_plan_stop_flag;

void route_plan_attitude_stabilization_control(void);

#endif
