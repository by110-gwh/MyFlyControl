#ifndef __PARAMER_SAVE_H
#define __PARAMER_SAVE_H

#include <stdint.h>

#define PARAMETER_SAVE_ADDR 0x0803F000

typedef struct {
	uint16_t rc_ch1_max;
	uint16_t rc_ch1_min;
	uint16_t rc_ch2_max;
	uint16_t rc_ch2_min;
	uint16_t rc_ch3_max;
	uint16_t rc_ch3_min;
	uint16_t rc_ch4_max;
	uint16_t rc_ch4_min;
	uint16_t rc_ch5_max;
	uint16_t rc_ch5_min;
	uint16_t rc_ch6_max;
	uint16_t rc_ch6_min;
	uint16_t rc_ch7_max;
	uint16_t rc_ch7_min;
	uint16_t rc_ch8_max;
	uint16_t rc_ch8_min;
	float accel_x_offset;
	float accel_y_offset;
	float accel_z_offset;
	float accel_x_scale;
	float accel_y_scale;
	float accel_z_scale;
	float mag_x_offset;
	float mag_y_offset;
	float mag_z_offset;
	short gyro_x_offset;
	short gyro_y_offset;
	short gyro_z_offset;
	uint8_t esc_calibration_flag;
	uint8_t inited;
} paramer_save_t;

extern volatile paramer_save_t paramer_save_data;

void write_save_paramer(void);
void read_save_paramer(void);

#endif
