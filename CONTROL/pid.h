#ifndef _PID_H
#define _PID_H

#include <stdint.h>
#include "time_cnt.h"

#define NULL 0

typedef struct pid_controler{
    //����
    float last_expect;
    //����
    float expect;
    //����ֵ
    float feedback;
    //ƫ��
    float err;
    //�ϴ�ƫ��
    float last_err;
    //���ϴ�ƫ��
    float pre_last_err;
    //ƫ���޷�ֵ
    float err_max;
    //���ַ���ƫ��ֵ
    float integrate_separation_err;
    //����ֵ
    float integrate;
    //�����޷�ֵ
    float integrate_max;
    //ƫ��΢��
    float dis_err;
    //���Ʋ���kp
    float kp;
    //���Ʋ���ki
    float ki;
    //���Ʋ���kd
    float kd;
    //ǰ�����Ʋ���kdkp
    float feedforward_kp;
    //ǰ�����Ʋ���kd
    float feedforward_kd;
    //�����������
    float control_output;
    //����޷�
    float control_output_limit;
    //��·��־
    uint8_t short_circuit_flag;
    //���ʱ�����
    Testime pid_controller_dt;
    //˽������
    void *pri_data;

    //�Զ������ƫ�ƫ����ֻص�
    void (*err_callback)(struct pid_controler *);
} pid_controler_t;

typedef void (*pid_callback_t)(pid_controler_t);

float pid_control(pid_controler_t *controler);

#endif
