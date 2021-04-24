#ifndef _AHRSAUX_H
#define _AHRSAUX_H

#include <stdint.h>
#include "vector3.h"

extern float Yaw, Pitch, Roll;
extern float Sin_Pitch, Sin_Roll, Sin_Yaw;
extern float Cos_Pitch, Cos_Roll, Cos_Yaw;

void ahrs_init(void);
void ahrs_update(void);
void Vector_From_BodyFrame2EarthFrame(Vector3f_t *bf, Vector3f_t *ef);
void Vector_From_EarthFrame2BodyFrame(Vector3f_t *ef, Vector3f_t *bf);

#endif
