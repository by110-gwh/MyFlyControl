#ifndef _OPENMV_H
#define _OPENMV_H 

#include <stdint.h>

extern volatile int16_t land_x;
extern volatile int16_t land_y;
extern volatile int16_t line_fr;
extern volatile int16_t line_fl;
extern volatile int16_t line_rf;
extern volatile int16_t line_rb;
extern volatile int16_t line_lf;
extern volatile int16_t line_lb;
extern volatile int16_t line_bl;
extern volatile int16_t line_br;
extern volatile uint32_t openmv_updata_flag;

void openmv_init(void);
void openmv_rec_callback(uint8_t data);

#endif
