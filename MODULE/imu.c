#include "imu.h"
#include "mpu6050.h"
#include "paramer_save.h"
#include <string.h>
#include "Filter.h"
#include "remote_control.h"
#include "math.h"
#include "ahrs_aux.h"
#include "time_cnt.h"

#include "FreeRTOS.h"
#include "task.h"

//巴特沃斯滤波参数
static Butter_Parameter Gyro_Parameter;
static Butter_Parameter Gyro_Parameter_Optical;
static Butter_Parameter Accel_Parameter;
static Butter_Parameter Acce_Correct_Parameter;
//巴特沃斯滤波内部数据
static Butter_BufferData Gyro_BufferData[3];
static Butter_BufferData Gyro_BuffeData_Optical[3];
static Butter_BufferData Accel_BufferData[3];
static Butter_BufferData Butter_Buffer_Correct[3];
//传感器原始数据
Vector3i_t accDataFilter;
Vector3i_t gyroDataFilter;
Vector3i_t gyroDataFilterOptical;
Vector3i_t acceCorrectFilter;
float tempDataFilter;

//加速计校准，保存6面待矫正数据
Vector3f_t acce_calibration_data[6];
//加速计校准状态
uint8_t acce_calibration_flag;
//陀螺仪校准状态
uint16_t gyro_calibration_flag;

/**********************************************************************************************************
*函 数 名: imu_init
*功能说明: IMU初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void imu_init()
{
	//设置传感器滤波参数
	Set_Cutoff_Frequency(Sampling_Freq, 50,&Gyro_Parameter);
	Set_Cutoff_Frequency(Sampling_Freq, 3.8,&Gyro_Parameter_Optical);
	Set_Cutoff_Frequency(Sampling_Freq, 30,&Accel_Parameter);
	Set_Cutoff_Frequency(Sampling_Freq, 1,&Acce_Correct_Parameter);
	//MPU6050初始化
	MPU6050_Detect();
    MPU6050_Init();
}

/**********************************************************************************************************
*函 数 名: get_imu_data
*功能说明: 获取IMU数据并滤波
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void get_imu_data()
{
	Vector3i_t accRawData;
	Vector3i_t gyroRawData;
	float tempRawData;
	
	//读取加速度传感器
	MPU6050_ReadAcc(&accRawData);
	//读取陀螺仪传感器
	MPU6050_ReadGyro(&gyroRawData);
	//读取温度传感器
	MPU6050_ReadTemp(&tempRawData);
	
	//数据校准
	accRawData.x = paramer_save_data.accel_x_scale * accRawData.x - paramer_save_data.accel_x_offset / ACCEL_SCALE;
	accRawData.y = paramer_save_data.accel_y_scale * accRawData.y - paramer_save_data.accel_y_offset / ACCEL_SCALE;
	accRawData.z = paramer_save_data.accel_z_scale * accRawData.z - paramer_save_data.accel_z_offset / ACCEL_SCALE;
	gyroRawData.x = gyroRawData.x - paramer_save_data.gyro_x_offset;
	gyroRawData.y = gyroRawData.y - paramer_save_data.gyro_y_offset;
	gyroRawData.z = gyroRawData.z - paramer_save_data.gyro_z_offset;
	
    //陀螺仪数据滤波
	gyroDataFilter.x = Butterworth_Filter(gyroRawData.x, &Gyro_BufferData[0], &Gyro_Parameter);
	gyroDataFilter.y = Butterworth_Filter(gyroRawData.y, &Gyro_BufferData[1], &Gyro_Parameter);
	gyroDataFilter.z = Butterworth_Filter(gyroRawData.z, &Gyro_BufferData[2], &Gyro_Parameter);
	
    //光流补偿陀螺仪数据滤波
	gyroDataFilterOptical.x = Butterworth_Filter(gyroRawData.x, &Gyro_BuffeData_Optical[0], &Gyro_Parameter_Optical);
	gyroDataFilterOptical.y = Butterworth_Filter(gyroRawData.y, &Gyro_BuffeData_Optical[1], &Gyro_Parameter_Optical);
	gyroDataFilterOptical.z = Butterworth_Filter(gyroRawData.z, &Gyro_BuffeData_Optical[2], &Gyro_Parameter_Optical);
    
	//加速计矫正数据滤波
	acceCorrectFilter.x = Butterworth_Filter(accRawData.x, &Butter_Buffer_Correct[0], &Acce_Correct_Parameter);
	acceCorrectFilter.y = Butterworth_Filter(accRawData.y, &Butter_Buffer_Correct[1], &Acce_Correct_Parameter);
	acceCorrectFilter.z = Butterworth_Filter(accRawData.z, &Butter_Buffer_Correct[2], &Acce_Correct_Parameter);
    
    //加速度数据滤波
	accDataFilter.x = Butterworth_Filter(accRawData.x, &Accel_BufferData[0], &Accel_Parameter);
	accDataFilter.y = Butterworth_Filter(accRawData.y, &Accel_BufferData[1], &Accel_Parameter);
	accDataFilter.z = Butterworth_Filter(accRawData.z, &Accel_BufferData[2], &Accel_Parameter);
	
	//温度数据不滤波
	tempDataFilter = tempRawData;
}

/**********************************************************************************************************
*函 数 名: Calibrate_Reset_Matrices
*功能说明: 初始化矩阵变量
*形    参: 梯度矩阵 Hessian矩阵
*返 回 值: 无
**********************************************************************************************************/
static void Calibrate_Reset_Matrices(float dS[6], float JS[6][6])
{
	int16_t j, k;
	for (j = 0; j < 6; j++) {
		dS[j] = 0.0f;
		for (k = 0; k < 6; k++) {
			JS[j][k] = 0.0f;
		}
	}
}

/**********************************************************************************************************
*函 数 名: Calibrate_Find_Delta
*功能说明: 使用高斯消元法求解△
*形    参: 梯度矩阵 Hessian矩阵 迭代步长
*返 回 值: 无
**********************************************************************************************************/
static void Calibrate_Find_Delta(float dS[6], float JS[6][6], float delta[6])
{
	int16_t i, j, k;
	float mu;
	//逐次消元，将线性方程组转换为上三角方程组
	for (i = 0; i < 6; i++) {
		//若JtJ[i][i]不为0，将该列在JtJ[i][i]以下的元素消为0
		for (j = i + 1; j < 6; j++) {
			mu = JS[i][j] / JS[i][i];
			if (mu != 0.0f) {
				dS[j] -= mu * dS[i];
				for (k = j; k < 6; k++) {
					JS[k][j] -= mu * JS[k][i];
				}
			}
		}
	}
	//回代得到方程组的解
	for (i = 5; i >= 0; i--)
	{
		dS[i] /= JS[i][i];
		JS[i][i] = 1.0f;

		for (j = 0; j < i; j++) {
			mu = JS[i][j];
			dS[j] -= mu * dS[i];
			JS[i][j] = 0.0f;
		}
	}
	for (i = 0; i < 6; i++) {
		delta[i] = dS[i];
	}
}

/**********************************************************************************************************
*函 数 名: Calibrate_Update_Matrices
*功能说明: 算求解△所用到的矩阵
*形    参: 梯度矩阵 Hessian矩阵 方程解 数据
*返 回 值: 无
**********************************************************************************************************/
static void Calibrate_Update_Matrices(float dS[6], float JS[6][6], float beta[6], float data[3])
{
	int16_t j, k;
	float dx, b;
	float residual = 1.0;
	float jacobian[6];
	for (j = 0; j < 3; j++) {
		b = beta[3 + j];
		dx = (float)data[j] - beta[j];
		//计算残差 (传感器误差方程的误差)
		residual -= b * b * dx * dx;
		//计算雅可比矩阵
		jacobian[j] = 2.0f * b * b * dx;
		jacobian[3 + j] = -2.0f * b * dx * dx;
	}

	for (j = 0; j < 6; j++) {
		//计算函数梯度
		dS[j] += jacobian[j] * residual;
		for (k = 0; k < 6; k++) {
			//计算Hessian矩阵（简化形式，省略二阶偏导），即雅可比矩阵与其转置的乘积
			JS[j][k] += jacobian[j] * jacobian[k];
		}
	}
}

/**********************************************************************************************************
*函 数 名: Calibrate_accel
*功能说明: 高斯牛顿法求解传感器误差方程，得到校准参数
*形    参: 传感器采样数据（6组） 零偏误差指针 比例误差指针
*返 回 值: 0：失败 1：成功
**********************************************************************************************************/
static uint8_t Calibrate_accel(Vector3f_t accel_sample[6], Vector3f_t *accel_offsets, Vector3f_t *accel_scale)
{
	int16_t i;
	int16_t num_iterations = 0;
	float eps = 0.000000001;
	float change = 100.0;
	float data[3] = {0};
	//方程解
	float beta[6] = {0};
	//迭代步长
	float delta[6] = {0};
	//梯度矩阵
	float ds[6] = {0};
	//Hessian矩阵
	float JS[6][6] = {0};
	//设定方程解初值
	beta[0] = beta[1] = beta[2] = 0;
	beta[3] = beta[4] = beta[5] = 1.0f / GRAVITY_MSS;
	//开始迭代，当迭代步长小于eps时结束计算，得到方程近似最优解
	while (num_iterations < 20 && change > eps)
	{
		num_iterations++;
		//矩阵初始化
		Calibrate_Reset_Matrices(ds, JS);

		//计算误差方程函数的梯度JtR和Hessian矩阵JtJ
		for (i = 0; i < 6; i++) {
			data[0] = accel_sample[i].x;
			data[1] = accel_sample[i].y;
			data[2] = accel_sample[i].z;
			Calibrate_Update_Matrices(ds, JS, beta, data);
		}
		//高斯消元法求解方程：JtJ * delta = JtR，得到delta
		Calibrate_Find_Delta(ds, JS, delta);
		//计算迭代步长
		change = delta[0] * delta[0] +
			delta[0] * delta[0] +
			delta[1] * delta[1] +
			delta[2] * delta[2] +
			delta[3] * delta[3] / (beta[3] * beta[3]) +
			delta[4] * delta[4] / (beta[4] * beta[4]) +
			delta[5] * delta[5] / (beta[5] * beta[5]);
		//更新方程解
		for (i = 0; i < 6; i++) {
			beta[i] -= delta[i];
		}
	}
	//更新校准参数
	accel_scale->x = beta[3] * GRAVITY_MSS;
	accel_scale->y = beta[4] * GRAVITY_MSS;
	accel_scale->z = beta[5] * GRAVITY_MSS;
	accel_offsets->x = beta[0] * accel_scale->x;
	accel_offsets->y = beta[1] * accel_scale->y;
	accel_offsets->z = beta[2] * accel_scale->z;

	//较准参数检查
	if (fabsf(accel_scale->x - 1.0f) > 0.5f || fabsf(accel_scale->y - 1.0f) > 0.5f || fabsf(accel_scale->z - 1.0f) > 0.5f) {
		return 0;
	}
	if (fabsf(accel_offsets->x) > 5.0f || fabsf(accel_offsets->y) > 5.0f || fabsf(accel_offsets->z) > 5.0f) {
		return 0;
	}
	return 1;
}

/**********************************************************************************************************
*函 数 名: accel_calibration
*功能说明: 加速计校准
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void accel_calibration(void)
{
	uint8_t i;
    portTickType xLastWakeTime;
	UBaseType_t this_task_priority;
	Vector3f_t acce_sample_sum;
	Vector3f_t new_offset;
	Vector3f_t new_scales;
	
	//清空相关参数
	acce_calibration_flag = 0;
	for(i = 0; i < 6; i++) {
		acce_calibration_data[i].x = 0;
		acce_calibration_data[i].y = 0;
		acce_calibration_data[i].z = 0;
    }
	paramer_save_data.accel_x_offset = 0;
	paramer_save_data.accel_y_offset = 0;
	paramer_save_data.accel_z_offset = 0;
	paramer_save_data.accel_x_scale = 1;
	paramer_save_data.accel_y_scale = 1;
	paramer_save_data.accel_z_scale = 1;
	
	//初始化imu
	imu_init();
	
	//第一面飞控平放，Z轴正向朝着正上方
	//第二面飞控平放，X轴正向朝着正上方
	//第三面飞控平放，X轴正向朝着正下方
	//第四面飞控平放，Y轴正向朝着正下方
	//第五面飞控平放，Y轴正向朝着正上方
	//第六面飞控平放，Z轴正向朝着正下方
	while (acce_calibration_flag < 6) {
		uint8_t key;
		//等待遥控器指令
		key = rc_scan();
		if (key == 0x01) {
			uint16_t num_samples=0;
			
			//提高本任务优先级
			this_task_priority = uxTaskPriorityGet(NULL);
			vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
			
			//累计归零
			acce_sample_sum.x = 0;
			acce_sample_sum.y = 0;
			acce_sample_sum.z = 0;
			
			//初始化时间
			xLastWakeTime = xTaskGetTickCount();
			while(num_samples<1000) {
				get_imu_data();
				//加速度计转化为1g量程下
				acce_sample_sum.x += acceCorrectFilter.x * ACCEL_SCALE;
				acce_sample_sum.y += acceCorrectFilter.y * ACCEL_SCALE;
				acce_sample_sum.z += acceCorrectFilter.z * ACCEL_SCALE;
				num_samples++;
				//5ms周期定时
				vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
			}
			//保存对应面的加速度计量
			acce_calibration_data[acce_calibration_flag].x = acce_sample_sum.x / num_samples;
			acce_calibration_data[acce_calibration_flag].y = acce_sample_sum.y / num_samples;
			acce_calibration_data[acce_calibration_flag].z = acce_sample_sum.z / num_samples;
			acce_calibration_flag++;
			//恢复本任务优先级
			vTaskPrioritySet(NULL, this_task_priority);
		}
	}
	//用所得6面数据计算加速度校准数据
	if(Calibrate_accel(acce_calibration_data, &new_offset, &new_scales)) {
		//参数保存
		paramer_save_data.accel_x_offset = new_offset.x;
		paramer_save_data.accel_y_offset = new_offset.y;
		paramer_save_data.accel_z_offset = new_offset.z;
		paramer_save_data.accel_x_scale = new_scales.x;
		paramer_save_data.accel_y_scale = new_scales.y;
		paramer_save_data.accel_z_scale = new_scales.z;
		write_save_paramer();
	}
}

/**********************************************************************************************************
*函 数 名: gyro_calibration
*功能说明: 陀螺仪校准校准
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void gyro_calibration()
{
	Vector3l_t gyroSumData;
	Vector3i_t gyroRawData;
	UBaseType_t this_task_priority;
    portTickType xLastWakeTime;
	
    gyroSumData.x = 0;
    gyroSumData.y = 0;
    gyroSumData.z = 0;
	//IMU初始化
	imu_init();
    //等待IMU初始化
    vTaskDelay(100);
	//提高本任务优先级
	this_task_priority = uxTaskPriorityGet(NULL);
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
	//得到初始时间
	xLastWakeTime = xTaskGetTickCount();
	for (gyro_calibration_flag = 0; gyro_calibration_flag < 400; gyro_calibration_flag++) {
		//读取陀螺仪传感器
		MPU6050_ReadGyro(&gyroRawData);
		//将陀螺仪数据累加
		gyroSumData.x += gyroRawData.x;
		gyroSumData.y += gyroRawData.y;
		gyroSumData.z += gyroRawData.z;
		//5ms周期定时
		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
	}
	//取平均
	gyroSumData.x = gyroSumData.x / 400;
	gyroSumData.y = gyroSumData.y / 400;
	gyroSumData.z = gyroSumData.z / 400;
	//恢复本任务优先级
	vTaskPrioritySet(NULL, this_task_priority);
	//保存参数
	paramer_save_data.gyro_x_offset = gyroSumData.x;
	paramer_save_data.gyro_y_offset = gyroSumData.y;
	paramer_save_data.gyro_z_offset = gyroSumData.z;
	//write_save_paramer();
}
