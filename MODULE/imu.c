#include "imu.h"
#include "ist8310.h"
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

//������˹�˲����� 200hz---30hz-98hz ����-���
static Butter_Parameter Bandstop_Filter_Parameter_30_98={
	1,					0.6270403082828,	-0.2905268567319,
	0.354736571634,		0.6270403082828,	0.354736571634
};
//������˹�˲����� 200hz---30hz-94hz  ����-���
static Butter_Parameter Bandstop_Filter_Parameter_30_94={
	1,   				0.5334540355829,	-0.2235264828971,
	0.3882367585514,	0.5334540355829,	0.3882367585514
};
//������˹�˲�����
static Butter_Parameter Gyro_Parameter;
static Butter_Parameter Accel_Parameter;
static Butter_Parameter Acce_Correct_Parameter;
//������˹�˲��ڲ�����
static Butter_BufferData Gyro_BufferData[3];
static Butter_BufferData Gyro_BufferData_BPF[3];
static Butter_BufferData Accel_BufferData_BPF[3];
static Butter_BufferData Accel_BufferData[3];
static Butter_BufferData Butter_Buffer_Correct[3];
//�����˲��������д�С
#define Filter_Data_SIZE 5
//�����˲���������
static float Filter_Data_X[Filter_Data_SIZE];
static float Filter_Data_Y[Filter_Data_SIZE];
static float Filter_Data_Z[Filter_Data_SIZE];
//������ԭʼ����
Vector3i_t accDataFilter;
Vector3i_t gyroDataFilter;
Vector3i_t acceCorrectFilter;
Vector3i_t MagDataFilter;
float tempDataFilter;

//���ټ�У׼������6�����������
Vector3f_t acce_calibration_data[6];
//���ټ�У׼״̬
uint8_t acce_calibration_flag;
//������У׼�����״̬
uint8_t mag_360_flag[3][36];
//������У׼״̬
uint8_t mag_calibration_flag;
//������У׼��ǰ�Ƕ�
float mag_correct_yaw;
//������У׼״̬
uint16_t gyro_calibration_flag;
//�����ƽ��������Ƕȣ�ȷ�����ݲɼ����
const int16_t mag_360_define[36]={
  0,10,20,30,40,50,60,70,80,90,
  100,110,120,130,140,150,160,170,180,190,
  200,210,220,230,240,250,260,270,280,290,
  300,310,320,330,340,350
};

/**********************************************************************************************************
*�� �� ��: imu_init
*����˵��: IMU��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void imu_init()
{
	//���ô������˲�����
	Set_Cutoff_Frequency(Sampling_Freq, 50,&Gyro_Parameter);
	Set_Cutoff_Frequency(Sampling_Freq, 60,&Accel_Parameter);
	Set_Cutoff_Frequency(Sampling_Freq, 1,&Acce_Correct_Parameter);
	//MPU6050��ʼ��
	MPU6050_Detect();
    MPU6050_Init();
	//IST83100��ʼ��
	IST8310_Detect();
    IST8310_Init();
}

/**********************************************************************************************************
*�� �� ��: get_imu_data
*����˵��: ��ȡIMU���ݲ��˲�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void get_imu_data()
{
	Vector3i_t accRawData;
	Vector3i_t gyroRawData;
	float tempRawData;
	static uint8_t IST8310_Sample_Cnt = 0;
	
	//��ȡ�����ƴ�����
	static Vector3i_t MagRawData;
	//��ȡ���ٶȴ�����
	MPU6050_ReadAcc(&accRawData);
	//��ȡ�����Ǵ�����
	MPU6050_ReadGyro(&gyroRawData);
	//��ȡ�¶ȴ�����
	MPU6050_ReadTemp(&tempRawData);
	//��ȡ�����ƴ�����
	IST8310_Sample_Cnt++;
	if(IST8310_Sample_Cnt == 1) {
		//���β���ģʽ�����ټ��6ms
		IST8310_Single_Measurement();
	} else if(IST8310_Sample_Cnt==4) {
		//��ȡ�����ƴ�����
		IST8310_ReadMag(&MagRawData);
		IST8310_Sample_Cnt=0;
	}
	
	//����У׼
	accRawData.x = paramer_save_data.accel_x_scale * accRawData.x - paramer_save_data.accel_x_offset / ACCEL_SCALE;
	accRawData.y = paramer_save_data.accel_y_scale * accRawData.y - paramer_save_data.accel_y_offset / ACCEL_SCALE;
	accRawData.z = paramer_save_data.accel_z_scale * accRawData.z - paramer_save_data.accel_z_offset / ACCEL_SCALE;
	MagRawData.x = MagRawData.x - paramer_save_data.mag_x_offset;
	MagRawData.y = MagRawData.y - paramer_save_data.mag_y_offset;
	MagRawData.z = MagRawData.z - paramer_save_data.mag_z_offset;
	gyroRawData.x = gyroRawData.x - paramer_save_data.gyro_x_offset;
	gyroRawData.y = gyroRawData.y - paramer_save_data.gyro_y_offset;
	gyroRawData.z = gyroRawData.z - paramer_save_data.gyro_z_offset;
	
	//���������ݴ����˲�
	gyroDataFilter.x = Butterworth_Filter(gyroRawData.x, &Gyro_BufferData_BPF[0], &Bandstop_Filter_Parameter_30_98);
	gyroDataFilter.y = Butterworth_Filter(gyroRawData.y, &Gyro_BufferData_BPF[1], &Bandstop_Filter_Parameter_30_98);
	gyroDataFilter.z = Butterworth_Filter(gyroRawData.z, &Gyro_BufferData_BPF[2], &Bandstop_Filter_Parameter_30_98);
	
	gyroRawData.x = Butterworth_Filter(gyroRawData.x, &Gyro_BufferData[0], &Gyro_Parameter);
	gyroRawData.y = Butterworth_Filter(gyroRawData.y, &Gyro_BufferData[1], &Gyro_Parameter);
	gyroRawData.z = Butterworth_Filter(gyroRawData.z, &Gyro_BufferData[2], &Gyro_Parameter);
	
	gyroDataFilter.x = gyroRawData.x;
	gyroDataFilter.y = gyroRawData.y;
	gyroDataFilter.z = gyroRawData.z;
	
	//���ټƽ������ݴ����˲�
	acceCorrectFilter.x = Butterworth_Filter(accRawData.x, &Butter_Buffer_Correct[0], &Acce_Correct_Parameter);
	acceCorrectFilter.y = Butterworth_Filter(accRawData.y, &Butter_Buffer_Correct[1], &Acce_Correct_Parameter);
	acceCorrectFilter.z = Butterworth_Filter(accRawData.z, &Butter_Buffer_Correct[2], &Acce_Correct_Parameter);
	
	//���ټ����ݴ����˲�
	accRawData.x = Butterworth_Filter(accRawData.x, &Accel_BufferData_BPF[0], &Bandstop_Filter_Parameter_30_94);
	accRawData.y = Butterworth_Filter(accRawData.y, &Accel_BufferData_BPF[1], &Bandstop_Filter_Parameter_30_94);
	accRawData.z = Butterworth_Filter(accRawData.z, &Accel_BufferData_BPF[2], &Bandstop_Filter_Parameter_30_94);
	
	accRawData.x = Butterworth_Filter(accRawData.x, &Accel_BufferData[0], &Accel_Parameter);
	accRawData.y = Butterworth_Filter(accRawData.y, &Accel_BufferData[1], &Accel_Parameter);
	accRawData.z = Butterworth_Filter(accRawData.z, &Accel_BufferData[2], &Accel_Parameter);
	
	accDataFilter.x = accRawData.x;
	accDataFilter.y = accRawData.y;
	accDataFilter.z = accRawData.z;
	
	//�¶����ݲ��˲�
	tempDataFilter = tempRawData;
	
	//�����ƴ���ƽ���˲���������ƽ��
	MagDataFilter.x = GildeAverageValueFilter(MagRawData.x, Filter_Data_X, Filter_Data_SIZE);
	MagDataFilter.y = GildeAverageValueFilter(MagRawData.y, Filter_Data_Y, Filter_Data_SIZE);
	MagDataFilter.z = GildeAverageValueFilter(MagRawData.z, Filter_Data_Z, Filter_Data_SIZE);
}

/**********************************************************************************************************
*�� �� ��: Calibrate_Reset_Matrices
*����˵��: ��ʼ���������
*��    ��: �ݶȾ��� Hessian����
*�� �� ֵ: ��
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
*�� �� ��: Calibrate_Find_Delta
*����˵��: ʹ�ø�˹��Ԫ������
*��    ��: �ݶȾ��� Hessian���� ��������
*�� �� ֵ: ��
**********************************************************************************************************/
static void Calibrate_Find_Delta(float dS[6], float JS[6][6], float delta[6])
{
	int16_t i, j, k;
	float mu;
	//�����Ԫ�������Է�����ת��Ϊ�����Ƿ�����
	for (i = 0; i < 6; i++) {
		//��JtJ[i][i]��Ϊ0����������JtJ[i][i]���µ�Ԫ����Ϊ0
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
	//�ش��õ�������Ľ�
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
*�� �� ��: Calibrate_Update_Matrices
*����˵��: ���������õ��ľ���
*��    ��: �ݶȾ��� Hessian���� ���̽� ����
*�� �� ֵ: ��
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
		//����в� (���������̵����)
		residual -= b * b * dx * dx;
		//�����ſɱȾ���
		jacobian[j] = 2.0f * b * b * dx;
		jacobian[3 + j] = -2.0f * b * dx * dx;
	}

	for (j = 0; j < 6; j++) {
		//���㺯���ݶ�
		dS[j] += jacobian[j] * residual;
		for (k = 0; k < 6; k++) {
			//����Hessian���󣨼���ʽ��ʡ�Զ���ƫ���������ſɱȾ�������ת�õĳ˻�
			JS[j][k] += jacobian[j] * jacobian[k];
		}
	}
}

/**********************************************************************************************************
*�� �� ��: Calibrate_accel
*����˵��: ��˹ţ�ٷ���⴫�������̣��õ�У׼����
*��    ��: �������������ݣ�6�飩 ��ƫ���ָ�� �������ָ��
*�� �� ֵ: 0��ʧ�� 1���ɹ�
**********************************************************************************************************/
static uint8_t Calibrate_accel(Vector3f_t accel_sample[6], Vector3f_t *accel_offsets, Vector3f_t *accel_scale)
{
	int16_t i;
	int16_t num_iterations = 0;
	float eps = 0.000000001;
	float change = 100.0;
	float data[3] = {0};
	//���̽�
	float beta[6] = {0};
	//��������
	float delta[6] = {0};
	//�ݶȾ���
	float ds[6] = {0};
	//Hessian����
	float JS[6][6] = {0};
	//�趨���̽��ֵ
	beta[0] = beta[1] = beta[2] = 0;
	beta[3] = beta[4] = beta[5] = 1.0f / GRAVITY_MSS;
	//��ʼ����������������С��epsʱ�������㣬�õ����̽������Ž�
	while (num_iterations < 20 && change > eps)
	{
		num_iterations++;
		//�����ʼ��
		Calibrate_Reset_Matrices(ds, JS);

		//�������̺������ݶ�JtR��Hessian����JtJ
		for (i = 0; i < 6; i++) {
			data[0] = accel_sample[i].x;
			data[1] = accel_sample[i].y;
			data[2] = accel_sample[i].z;
			Calibrate_Update_Matrices(ds, JS, beta, data);
		}
		//��˹��Ԫ����ⷽ�̣�JtJ * delta = JtR���õ�delta
		Calibrate_Find_Delta(ds, JS, delta);
		//�����������
		change = delta[0] * delta[0] +
			delta[0] * delta[0] +
			delta[1] * delta[1] +
			delta[2] * delta[2] +
			delta[3] * delta[3] / (beta[3] * beta[3]) +
			delta[4] * delta[4] / (beta[4] * beta[4]) +
			delta[5] * delta[5] / (beta[5] * beta[5]);
		//���·��̽�
		for (i = 0; i < 6; i++) {
			beta[i] -= delta[i];
		}
	}
	//����У׼����
	accel_scale->x = beta[3] * GRAVITY_MSS;
	accel_scale->y = beta[4] * GRAVITY_MSS;
	accel_scale->z = beta[5] * GRAVITY_MSS;
	accel_offsets->x = beta[0] * accel_scale->x;
	accel_offsets->y = beta[1] * accel_scale->y;
	accel_offsets->z = beta[2] * accel_scale->z;

	//��׼�������
	if (fabsf(accel_scale->x - 1.0f) > 0.5f || fabsf(accel_scale->y - 1.0f) > 0.5f || fabsf(accel_scale->z - 1.0f) > 0.5f) {
		return 0;
	}
	if (fabsf(accel_offsets->x) > 5.0f || fabsf(accel_offsets->y) > 5.0f || fabsf(accel_offsets->z) > 5.0f) {
		return 0;
	}
	return 1;
}

/**********************************************************************************************************
*�� �� ��: accel_calibration
*����˵��: ���ټ�У׼
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void accel_calibration(void)
{
	uint8_t i;
    portTickType xLastWakeTime;
	UBaseType_t this_task_priority;
	Vector3f_t acce_sample_sum;
	Vector3f_t new_offset;
	Vector3f_t new_scales;
	
	//�����ز���
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
	
	//��ʼ��imu
	imu_init();
	
	//��һ��ɿ�ƽ�ţ�Z�����������Ϸ�
	//�ڶ���ɿ�ƽ�ţ�X�����������Ϸ�
	//������ɿ�ƽ�ţ�X�����������·�
	//������ɿ�ƽ�ţ�Y�����������·�
	//������ɿ�ƽ�ţ�Y�����������Ϸ�
	//������ɿ�ƽ�ţ�Z�����������·�
	while (acce_calibration_flag < 6) {
		uint8_t key;
		//�ȴ�ң����ָ��
		key = rc_scan();
		if (key == 0x01) {
			uint16_t num_samples=0;
			
			//��߱��������ȼ�
			this_task_priority = uxTaskPriorityGet(NULL);
			vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
			
			//�ۼƹ���
			acce_sample_sum.x = 0;
			acce_sample_sum.y = 0;
			acce_sample_sum.z = 0;
			
			//��ʼ��ʱ��
			xLastWakeTime = xTaskGetTickCount();
			while(num_samples<1000) {
				get_imu_data();
				//���ٶȼ�ת��Ϊ1g������
				acce_sample_sum.x += acceCorrectFilter.x * ACCEL_SCALE;
				acce_sample_sum.y += acceCorrectFilter.y * ACCEL_SCALE;
				acce_sample_sum.z += acceCorrectFilter.z * ACCEL_SCALE;
				num_samples++;
				//5ms���ڶ�ʱ
				vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
			}
			//�����Ӧ��ļ��ٶȼ���
			acce_calibration_data[acce_calibration_flag].x = acce_sample_sum.x / num_samples;
			acce_calibration_data[acce_calibration_flag].y = acce_sample_sum.y / num_samples;
			acce_calibration_data[acce_calibration_flag].z = acce_sample_sum.z / num_samples;
			acce_calibration_flag++;
			//�ָ����������ȼ�
			vTaskPrioritySet(NULL, this_task_priority);
		}
	}
	//������6�����ݼ�����ٶ�У׼����
	if(Calibrate_accel(acce_calibration_data, &new_offset, &new_scales)) {
		//��������
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
*�� �� ��: mag_calibration_one_is_ok
*����˵��: �жϵ���������
*��    ��: ����У׼״̬λ
*�� �� ֵ: 1����� 0��δ���
**********************************************************************************************************/
static uint8_t mag_calibration_one_is_ok(uint8_t mag_360_flag[36])
{
	uint8_t i;
	for (i = 0; i < 36; i++) {
		if (mag_360_flag[i] == 0)
			return 0;
	}
	return 1;
}

Least_Squares_Intermediate_Variable Mag_LS;
void LS_Init(Least_Squares_Intermediate_Variable * pLSQ)
{
	memset(pLSQ, 0, sizeof(Least_Squares_Intermediate_Variable));
}

unsigned int LS_Accumulate(Least_Squares_Intermediate_Variable * pLSQ, float x, float y, float z)
{
	float x2 = x * x;
	float y2 = y * y;
	float z2 = z * z;

	pLSQ->x_sumplain += x;
	pLSQ->x_sumsq += x2;
	pLSQ->x_sumcube += x2 * x;

	pLSQ->y_sumplain += y;
	pLSQ->y_sumsq += y2;
	pLSQ->y_sumcube += y2 * y;

	pLSQ->z_sumplain += z;
	pLSQ->z_sumsq += z2;
	pLSQ->z_sumcube += z2 * z;

	pLSQ->xy_sum += x * y;
	pLSQ->xz_sum += x * z;
	pLSQ->yz_sum += y * z;

	pLSQ->x2y_sum += x2 * y;
	pLSQ->x2z_sum += x2 * z;

	pLSQ->y2x_sum += y2 * x;
	pLSQ->y2z_sum += y2 * z;

	pLSQ->z2x_sum += z2 * x;
	pLSQ->z2y_sum += z2 * y;

	pLSQ->size++;

	return pLSQ->size;
}


void LS_Calculate(Least_Squares_Intermediate_Variable * pLSQ,
                  unsigned int max_iterations,
                  float delta,
                  float *sphere_x, float *sphere_y, float *sphere_z,
                  float *sphere_radius)
{
	//
	//Least Squares Fit a sphere A,B,C with radius squared Rsq to 3D data
	//
	//    P is a structure that has been computed with the data earlier.
	//    P.npoints is the number of elements; the length of X,Y,Z are identical.
	//    P's members are logically named.
	//
	//    X[n] is the x component of point n
	//    Y[n] is the y component of point n
	//    Z[n] is the z component of point n
	//
	//    A is the x coordiante of the sphere
	//    B is the y coordiante of the sphere
	//    C is the z coordiante of the sphere
	//    Rsq is the radius squared of the sphere.
	//
	//This method should converge; maybe 5-100 iterations or more.
	//
	float x_sum = pLSQ->x_sumplain / pLSQ->size;        //sum( X[n] )
	float x_sum2 = pLSQ->x_sumsq / pLSQ->size;    //sum( X[n]^2 )
	float x_sum3 = pLSQ->x_sumcube / pLSQ->size;    //sum( X[n]^3 )
	float y_sum = pLSQ->y_sumplain / pLSQ->size;        //sum( Y[n] )
	float y_sum2 = pLSQ->y_sumsq / pLSQ->size;    //sum( Y[n]^2 )
	float y_sum3 = pLSQ->y_sumcube / pLSQ->size;    //sum( Y[n]^3 )
	float z_sum = pLSQ->z_sumplain / pLSQ->size;        //sum( Z[n] )
	float z_sum2 = pLSQ->z_sumsq / pLSQ->size;    //sum( Z[n]^2 )
	float z_sum3 = pLSQ->z_sumcube / pLSQ->size;    //sum( Z[n]^3 )

	float XY = pLSQ->xy_sum / pLSQ->size;        //sum( X[n] * Y[n] )
	float XZ = pLSQ->xz_sum / pLSQ->size;        //sum( X[n] * Z[n] )
	float YZ = pLSQ->yz_sum / pLSQ->size;        //sum( Y[n] * Z[n] )
	float X2Y = pLSQ->x2y_sum / pLSQ->size;    //sum( X[n]^2 * Y[n] )
	float X2Z = pLSQ->x2z_sum / pLSQ->size;    //sum( X[n]^2 * Z[n] )
	float Y2X = pLSQ->y2x_sum / pLSQ->size;    //sum( Y[n]^2 * X[n] )
	float Y2Z = pLSQ->y2z_sum / pLSQ->size;    //sum( Y[n]^2 * Z[n] )
	float Z2X = pLSQ->z2x_sum / pLSQ->size;    //sum( Z[n]^2 * X[n] )
	float Z2Y = pLSQ->z2y_sum / pLSQ->size;    //sum( Z[n]^2 * Y[n] )

	//Reduction of multiplications
	float F0 = x_sum2 + y_sum2 + z_sum2;
	float F1 =  0.5f * F0;
	float F2 = -8.0f * (x_sum3 + Y2X + Z2X);
	float F3 = -8.0f * (X2Y + y_sum3 + Z2Y);
	float F4 = -8.0f * (X2Z + Y2Z + z_sum3);

	//Set initial conditions:
	float A = x_sum;
	float B = y_sum;
	float C = z_sum;

	//First iteration computation:
	float A2 = A * A;
	float B2 = B * B;
	float C2 = C * C;
	float QS = A2 + B2 + C2;
	float QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);

	//Set initial conditions:
	float Rsq = F0 + QB + QS;

	//First iteration computation:
	float Q0 = 0.5f * (QS - Rsq);
	float Q1 = F1 + Q0;
	float Q2 = 8.0f * (QS - Rsq + QB + F0);
	float aA, aB, aC, nA, nB, nC, dA, dB, dC;

	//Iterate N times, ignore stop condition.
	unsigned int n = 0;

	while (n < max_iterations) {
		n++;

		//Compute denominator:
		aA = Q2 + 16.0f * (A2 - 2.0f * A * x_sum + x_sum2);
		aB = Q2 + 16.0f * (B2 - 2.0f * B * y_sum + y_sum2);
		aC = Q2 + 16.0f * (C2 - 2.0f * C * z_sum + z_sum2);
		aA = (aA == 0.0f) ? 1.0f : aA;
		aB = (aB == 0.0f) ? 1.0f : aB;
		aC = (aC == 0.0f) ? 1.0f : aC;

		//Compute next iteration
		nA = A - ((F2 + 16.0f * (B * XY + C * XZ + x_sum * (-A2 - Q0) + A * (x_sum2 + Q1 - C * z_sum - B * y_sum))) / aA);
		nB = B - ((F3 + 16.0f * (A * XY + C * YZ + y_sum * (-B2 - Q0) + B * (y_sum2 + Q1 - A * x_sum - C * z_sum))) / aB);
		nC = C - ((F4 + 16.0f * (A * XZ + B * YZ + z_sum * (-C2 - Q0) + C * (z_sum2 + Q1 - A * x_sum - B * y_sum))) / aC);

		//Check for stop condition
		dA = (nA - A);
		dB = (nB - B);
		dC = (nC - C);

		if ((dA * dA + dB * dB + dC * dC) <= delta) { break; }

		//Compute next iteration's values
		A = nA;
		B = nB;
		C = nC;
		A2 = A * A;
		B2 = B * B;
		C2 = C * C;
		QS = A2 + B2 + C2;
		QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
		Rsq = F0 + QB + QS;
		Q0 = 0.5f * (QS - Rsq);
		Q1 = F1 + Q0;
		Q2 = 8.0f * (QS - Rsq + QB + F0);
	}

	*sphere_x = A;
	*sphere_y = B;
	*sphere_z = C;
	*sphere_radius = sqrt(Rsq);
}

/**********************************************************************************************************
*�� �� ��: mag_calibration
*����˵��: ������У׼
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void mag_calibration(void)
{
	static Testime Time_Delta;
	float dt;
	float mag_a, mag_b, mag_c, mag_r;
	uint8_t i, j;
	UBaseType_t this_task_priority;
    portTickType xLastWakeTime;
	
	//��մ�����У׼ȫ����־
	mag_calibration_flag = 0;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 36; j++) {
			mag_360_flag[i][j] = 0;
		}
	}
	paramer_save_data.mag_x_offset = 0;
	paramer_save_data.mag_y_offset = 0;
	paramer_save_data.mag_z_offset = 0;
	//��ʼ��imu
	imu_init();
	//�ȴ�ң����������Ӧ
	while (rc_scan() != 0x07);
	//��߱��������ȼ�
	this_task_priority = uxTaskPriorityGet(NULL);
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
	//��ǰ�Ƕȹ�0
	mag_correct_yaw = 0;
	//�õ���ʼʱ��
	xLastWakeTime = xTaskGetTickCount();
	while(mag_calibration_one_is_ok(mag_360_flag[0]) == 0) {
		//���¼���ʱ���
		Get_Time_Period(&Time_Delta);
		dt = Time_Delta.Time_Delta / 1000000.0;
		//��ȡimu����
		get_imu_data();
		//���½Ƕ�
		mag_correct_yaw += gyroDataFilter.z * GYRO_CALIBRATION_COFF * dt;
		
		if(mag_correct_yaw < 0)
			mag_correct_yaw += 360;
		if(mag_correct_yaw > 360)
			mag_correct_yaw -= 360;
		for (i = 0; i < 36; i++) {
			if(mag_360_flag[0][i] == 0 && fabsf(mag_correct_yaw - mag_360_define[i]) <= 5.0f && acceCorrectFilter.z >= ACCEL_MAX_1G / 2) {
				mag_360_flag[0][i] = 1;
				LS_Accumulate(&Mag_LS, MagDataFilter.x, MagDataFilter.y, MagDataFilter.z);
				LS_Calculate(&Mag_LS, 36*3, 0.0f, &mag_a, &mag_b, &mag_c,&mag_r);
			}
		}
		//5ms���ڶ�ʱ
		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
	}
	//�ָ����������ȼ�
	vTaskPrioritySet(NULL, this_task_priority);
	
	//�ȴ�ң����������Ӧ
	while (rc_scan() != 0x07);
	//��߱��������ȼ�
	this_task_priority = uxTaskPriorityGet(NULL);
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
	//��ǰ�Ƕȹ�0
	mag_correct_yaw = 0;
	//�õ���ʼʱ��
	xLastWakeTime = xTaskGetTickCount();
	while(mag_calibration_one_is_ok(mag_360_flag[1]) == 0) {
		//���¼���ʱ���
		Get_Time_Period(&Time_Delta);
		dt = Time_Delta.Time_Delta / 1000000.0;
		//��ȡimu����
		get_imu_data();
		//���½Ƕ�
		mag_correct_yaw += gyroDataFilter.y * GYRO_CALIBRATION_COFF * dt;
		
		if(mag_correct_yaw < 0)
			mag_correct_yaw += 360;
		if(mag_correct_yaw > 360)
			mag_correct_yaw -= 360;
		for (i = 0; i < 36; i++) {
			if(mag_360_flag[1][i] == 0 && fabsf(mag_correct_yaw - mag_360_define[i]) <= 5.0f && acceCorrectFilter.y >= ACCEL_MAX_1G / 2) {
				mag_360_flag[1][i] = 1;
				LS_Accumulate(&Mag_LS, MagDataFilter.x, MagDataFilter.y, MagDataFilter.z);
				LS_Calculate(&Mag_LS, 36*3, 0.0f, &mag_a, &mag_b, &mag_c,&mag_r);
			}
		}
		//5ms���ڶ�ʱ
		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
	}
	//�ָ����������ȼ�
	vTaskPrioritySet(NULL, this_task_priority);
	
	//�ȴ�ң����������Ӧ
	while (rc_scan() != 0x07);
	//��߱��������ȼ�
	this_task_priority = uxTaskPriorityGet(NULL);
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
	//��ǰ�Ƕȹ�0
	mag_correct_yaw = 0;
	//�õ���ʼʱ��
	xLastWakeTime = xTaskGetTickCount();
	while(mag_calibration_one_is_ok(mag_360_flag[2]) == 0) {
		//���¼���ʱ���
		Get_Time_Period(&Time_Delta);
		dt = Time_Delta.Time_Delta / 1000000.0;
		//��ȡimu����
		get_imu_data();
		//���½Ƕ�
		mag_correct_yaw += gyroDataFilter.x * GYRO_CALIBRATION_COFF * dt;
		
		if(mag_correct_yaw < 0)
			mag_correct_yaw += 360;
		if(mag_correct_yaw > 360)
			mag_correct_yaw -= 360;
		for (i = 0; i < 36; i++) {
			if(mag_360_flag[2][i] == 0 && fabsf(mag_correct_yaw - mag_360_define[i]) <= 5.0f && acceCorrectFilter.x >= ACCEL_MAX_1G / 2) {
				mag_360_flag[2][i] = 1;
				LS_Accumulate(&Mag_LS, MagDataFilter.x, MagDataFilter.y, MagDataFilter.z);
				LS_Calculate(&Mag_LS, 36 * 3, 0.0f, &mag_a, &mag_b, &mag_c,&mag_r);
			}
		}
		//5ms���ڶ�ʱ
		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
	}
	//�ָ����������ȼ�
	vTaskPrioritySet(NULL, this_task_priority);
	//�������
	paramer_save_data.mag_x_offset = mag_a;
	paramer_save_data.mag_y_offset = mag_b;
	paramer_save_data.mag_z_offset = mag_c;
	write_save_paramer();
}

/**********************************************************************************************************
*�� �� ��: gyro_calibration
*����˵��: ������У׼У׼
*��    ��: ��
*�� �� ֵ: ��
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
	//IMU��ʼ��
	imu_init();
	//��߱��������ȼ�
	this_task_priority = uxTaskPriorityGet(NULL);
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
	//�õ���ʼʱ��
	xLastWakeTime = xTaskGetTickCount();
	for (gyro_calibration_flag = 0; gyro_calibration_flag < 400; gyro_calibration_flag++) {
		//��ȡ�����Ǵ�����
		MPU6050_ReadGyro(&gyroRawData);
		//�������������ۼ�
		gyroSumData.x += gyroRawData.x;
		gyroSumData.y += gyroRawData.y;
		gyroSumData.z += gyroRawData.z;
		//5ms���ڶ�ʱ
		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
	}
	//ȡƽ��
	gyroSumData.x = gyroSumData.x / 400;
	gyroSumData.y = gyroSumData.y / 400;
	gyroSumData.z = gyroSumData.z / 400;
	//�ָ����������ȼ�
	vTaskPrioritySet(NULL, this_task_priority);
	//�������
	paramer_save_data.gyro_x_offset = gyroSumData.x;
	paramer_save_data.gyro_y_offset = gyroSumData.y;
	paramer_save_data.gyro_z_offset = gyroSumData.z;
	//write_save_paramer();
}
