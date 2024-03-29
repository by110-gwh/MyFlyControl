#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include <stdint.h>

typedef struct
{
	uint16_t max;
	uint16_t min;
	uint16_t middle;
	uint16_t deadband;
	uint16_t deadband_top;
	uint16_t deadband_buttom;
} rc_calibration_data_t;

//最大俯仰、横滚期望
#define Pit_Rol_Max 20
//最大偏航期望
#define Yaw_Max     90

#define RC_RESET_DEFAULT     1500
#define RC_DEADBAND_PERCENT  0.1
#define RC_DEADBAND_CHANNEL  8

extern rc_calibration_data_t rc_calibration_data[RC_DEADBAND_CHANNEL];
extern uint16_t rc_raw_data[RC_DEADBAND_CHANNEL];
extern uint16_t Throttle_Control;
extern float Pitch_Control, Roll_Control, Yaw_Control, High_Control;
extern volatile uint8_t remote_control_updata;

void rc_init(void);
void rc_callback(uint16_t buf[8]);
uint8_t rc_scan(void);
uint8_t rc_direct_is_reset(void);
void rc_calibration_task(void);
uint8_t rc_is_on(void);

#endif
