#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>
#include "vector3.h"
#include "mpu6050.h"

//采样频率 200Hz
#define Sampling_Freq 200

//传感器滤波后数据
extern Vector3i_t accDataFilter;
extern Vector3i_t gyroDataFilter;
extern Vector3i_t gyroDataFilterOptical;
extern Vector3i_t acceCorrectFilter;
extern float tempDataFilter;

extern Vector3f_t acce_calibration_data[6];
extern uint8_t acce_calibration_flag;

void imu_init(void);
void get_imu_data(void);
void accel_calibration(void);
void gyro_calibration(void);

#endif
