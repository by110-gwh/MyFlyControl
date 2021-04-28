/**********************************************************************************************************
                                ���ɿ� ���� �����ڴ����й���õĶ�����Դ�ɿ�
                                Github: github.com/loveuav/BlueSkyFlightControl
                                �������ۣ�bbs.loveuav.com/forum-68-1.html
 * @�ļ�     vector3.c
 * @˵��     3ά�����ṹ��Ķ�����������㺯��
 * @�汾  	 V1.0
 * @����     BlueSky
 * @��վ     bbs.loveuav.com
 * @����     2018.05
**********************************************************************************************************/
#include "vector3.h"

/**********************************************************************************************************
*�� �� ��: Vector3f_Normalize
*����˵��: ������һ��
*��    ��: Vector3f��
*�� �� ֵ: ��
**********************************************************************************************************/
void Vector3f_Normalize(Vector3f_t* vector)
{
    float mag = Pythagorous3(vector->x, vector->y, vector->z);

    vector->x /= mag;
    vector->y /= mag;
    vector->z /= mag;
}

/**********************************************************************************************************
*�� �� ��: Vector3iTo3f
*����˵��: ��������ת��
*��    ��: Vector3i��
*�� �� ֵ: Vector3f��
**********************************************************************************************************/
Vector3f_t Vector3iTo3f(Vector3i_t vector)
{
    Vector3f_t v_tmp;
    v_tmp.x = (float)vector.x;
    v_tmp.y = (float)vector.y;
    v_tmp.z = (float)vector.z;
    return v_tmp;
}

/**********************************************************************************************************
*�� �� ��: Vector3fTo3i
*����˵��: ��������ת��
*��    ��: Vector3f��
*�� �� ֵ: Vector3i��
**********************************************************************************************************/
Vector3i_t Vector3fTo3i(Vector3f_t vector)
{
    Vector3i_t v_tmp;
    v_tmp.x = (int16_t)vector.x;
    v_tmp.y = (int16_t)vector.y;
    v_tmp.z = (int16_t)vector.z;
    return v_tmp;
}

/**********************************************************************************************************
*�� �� ��: Vector3f_Add
*����˵��: �����������ӷ�
*��    ��: ����v1 ����v2
*�� �� ֵ: ������
**********************************************************************************************************/
Vector3f_t Vector3f_Add(Vector3f_t v1, Vector3f_t v2)
{
    Vector3f_t v_tmp;

    v_tmp.x = v1.x + v2.x;
    v_tmp.y = v1.y + v2.y;
    v_tmp.z = v1.z + v2.z;

    return v_tmp;
}

/**********************************************************************************************************
*�� �� ��: Vector3f_Sub
*����˵��: ��������������
*��    ��: ����v1 ����v2
*�� �� ֵ: ������
**********************************************************************************************************/
Vector3f_t Vector3f_Sub(Vector3f_t v1, Vector3f_t v2)
{
    Vector3f_t v_tmp;

    v_tmp.x = v1.x - v2.x;
    v_tmp.y = v1.y - v2.y;
    v_tmp.z = v1.z - v2.z;

    return v_tmp;
}

/**********************************************************************************************************
*�� �� ��: VectorCrossProduct
*����˵��: ��ά�����Ĳ������
*��    ��: ����a ����b
*�� �� ֵ: ���������
**********************************************************************************************************/
Vector3f_t VectorCrossProduct(Vector3f_t a, Vector3f_t b)
{
    Vector3f_t c;

    c.x = a.y * b.z - b.y * a.z;
    c.y = a.z * b.x - b.z * a.x;
    c.z = a.x * b.y - b.x * a.y;

    return c;
}

/**********************************************************************************************************
*�� �� ��: Matrix3MulVector3
*����˵��: ��ά��������ά�������
*��    ��: ��ά���� ��ά����
*�� �� ֵ: �˻�
**********************************************************************************************************/
Vector3f_t Matrix3MulVector3(float* m, Vector3f_t vector)
{
    Vector3f_t v_tmp;

    v_tmp.x = m[0] * vector.x + m[1] * vector.y + m[2] * vector.z;
    v_tmp.y = m[3] * vector.x + m[4] * vector.y + m[5] * vector.z;
    v_tmp.z = m[6] * vector.x + m[7] * vector.y + m[8] * vector.z;

    return v_tmp;
}

/**********************************************************************************************************
*�� �� ��: EulerAngleToDCM
*����˵��: ŷ����ת�������Ҿ��󣨲ο�ϵ������ϵ��
*��    ��: ŷ���� ����ָ��
*�� �� ֵ: ��
**********************************************************************************************************/
void EulerAngleToDCM(Vector3f_t angle, float* dcM)
{
    Vector3f_t cos, sin;

    cos.x = cosf(angle.x);
    cos.y = cosf(angle.y);
    cos.z = cosf(angle.z);
    sin.x = sinf(angle.x);
    sin.y = sinf(angle.y);
    sin.z = sinf(angle.z);

    dcM[0] = cos.y * cos.z;
    dcM[1] = sin.z * cos.x + sin.x * sin.y * cos.z;
    dcM[2] = -sin.x * sin.z + sin.y * cos.x * cos.z;
    dcM[3] = -sin.z * cos.y;
    dcM[4] = cos.x * cos.z - sin.x * sin.y * sin.z;
    dcM[5] = -sin.x * cos.z - sin.y * sin.z * cos.x;
    dcM[6] = -sin.y;
    dcM[7] = sin.x * cos.y;
    dcM[8] = cos.x * cos.y;
}

/**********************************************************************************************************
*�� �� ��: EulerAngleToDCM_T
*����˵��: ŷ����ת�������Ҿ��󣨻���ϵ���ο�ϵ��
*��    ��: ŷ���� ����ָ��
*�� �� ֵ: ��
**********************************************************************************************************/
void EulerAngleToDCM_T(Vector3f_t angle, float* dcM)
{
    Vector3f_t cos, sin;

    cos.x = cosf(angle.x);
    cos.y = cosf(angle.y);
    cos.z = cosf(angle.z);
    sin.x = sinf(angle.x);
    sin.y = sinf(angle.y);
    sin.z = sinf(angle.z);

    dcM[0] = cos.y * cos.z;
    dcM[1] = -sin.z * cos.y;
    dcM[2] = -sin.y;
    dcM[3] = sin.z * cos.x + sin.x * sin.y * cos.z;
    dcM[4] = cos.x * cos.z - sin.x * sin.y * sin.z;
    dcM[5] = sin.x * cos.y;
    dcM[6] = -sin.x * sin.z + sin.y * cos.x * cos.z;
    dcM[7] = -sin.x * cos.z - sin.y * sin.z * cos.x;
    dcM[8] = cos.x * cos.y;
}

/**********************************************************************************************************
*�� �� ��: VectorRotateToBodyFrame
*����˵��: ��ά������ת������ϵ
*��    ��: ��ά���� �Ƕȱ仯�������ȣ�
*�� �� ֵ: ��ת�����ά����
**********************************************************************************************************/
Vector3f_t VectorRotateToBodyFrame(Vector3f_t vector, Vector3f_t deltaAngle)
{
    float dcMat[9];

    //ŷ����תΪ�������Ҿ���
    EulerAngleToDCM(deltaAngle, dcMat);

    //�������Ҿ�������������õ���ת���������
    return Matrix3MulVector3(dcMat, vector);
}

/**********************************************************************************************************
*�� �� ��: VectorRotateToEarthFrame
*����˵��: ��ά������ת���ο�ϵ
*��    ��: ��ά���� �Ƕȱ仯�������ȣ�
*�� �� ֵ: ��ת�����ά����
**********************************************************************************************************/
Vector3f_t VectorRotateToEarthFrame(Vector3f_t vector, Vector3f_t deltaAngle)
{
    float dcMat[9];

    //ŷ����תΪ�������Ҿ���
    EulerAngleToDCM_T(deltaAngle, dcMat);

    //�������Ҿ�������������õ���ת���������
    return Matrix3MulVector3(dcMat, vector);
}

/**********************************************************************************************************
*�� �� ��: AccVectorToEulerAngle
*����˵��: �����������ٶ������ڻ���ϵ�ϵ�ͶӰ���㸩���ͺ����
*��    ��: ��̬��ָ�� ���ٶ�����
*�� �� ֵ: ��
**********************************************************************************************************/
void AccVectorToRollPitchAngle(Vector3f_t* angle, Vector3f_t vector)
{
    //���ٶ�������һ��
    Vector3f_Normalize(&vector);

    angle->x = -SafeArcsin(vector.y);       //�����
    angle->y = atan2f(vector.x, vector.z);  //������
}

/**********************************************************************************************************
*�� �� ��: MagVectorToEulerAngle
*����˵��: ���ݵشų������ڻ���ϵ�ϵ�ͶӰ����ƫ����
*��    ��: ��̬��ָ�� �شų�����
*�� �� ֵ: ��
**********************************************************************************************************/
void MagVectorToYawAngle(Vector3f_t* angle, Vector3f_t vector)
{
    angle->z = -atan2f(vector.y, vector.x);     //ƫ����
}



