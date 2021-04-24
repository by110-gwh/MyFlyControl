#ifndef _KEY_H
#define _KEY_H

#include <stdint.h>

#define KEY0 1
#define KEY1 2

void key_init(void);
uint8_t key_scan(void);

#endif
