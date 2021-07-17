#ifndef _PID_H
#define _PID_H

#include <stdint.h>
#include "time_cnt.h"

#define NULL 0

typedef struct pid_paramer {
    //ƫ���޷�ֵ
    float err_max;
    //���ַ���ƫ��ֵ
    float integrate_separation_err;
    //�����޷�ֵ
    float integrate_max;
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
    //����޷�
    float control_output_limit;
} pid_paramer_t;

typedef struct pid_data{
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
    //����ֵ
    float integrate;
    //ƫ��΢��
    float dis_err;
    //�����������
    float control_output;
    //���ʱ�����
    Testime pid_controller_dt;
    //˽������
    void *pri_data;
    //�Զ������ƫ�ƫ����ֻص�
    void (*err_callback)(struct pid_data *, struct pid_paramer *);
    //��·��־
    uint8_t short_circuit_flag;
} pid_data_t;

typedef void (*pid_callback_t)(struct pid_data *, struct pid_paramer *);

float pid_control(pid_data_t *data, pid_paramer_t *para);

#endif
