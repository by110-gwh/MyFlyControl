#ifndef __PMW3901_H
#define __PMW3901_H

#include <stdint.h>

//1cm�߶��� 1�����ض�Ӧ��λ�ƣ���λcm
#define OPTICAL_SCALS 0.002131946f

void pmw3901_init(void);
void pmw3901_read_motion(int16_t *dx, int16_t *dy, uint16_t *qual);

#endif
