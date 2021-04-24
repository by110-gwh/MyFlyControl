#ifndef _IST8310_H
#define	_IST8310_H

//#include "mathTool.h"
#include <stdint.h>
#include "vector3.h"

uint8_t IST8310_Detect(void);
void IST8310_Init(void);
void IST8310_ReadMag(Vector3i_t* mag);
void IST8310_Single_Measurement(void);

#endif








