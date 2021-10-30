#ifndef _OPENMV_H
#define _OPENMV_H 

#include <stdint.h>

extern volatile uint32_t openmv_updata_flag;
extern volatile int16_t pole_distance;
extern volatile int16_t line_high;
extern volatile int16_t front_line_offset;
extern volatile int16_t right_line_offset;
extern volatile int16_t left_line_offset;
extern volatile int16_t back_line_offset;

void openmv_init(void);
void openmv_rec_callback(uint8_t data);

#endif
