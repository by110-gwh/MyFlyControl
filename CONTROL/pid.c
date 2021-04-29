#include "pid.h"

#define ABS(X)  (((X) > 0)? (X) : -(X))

/**********************************************************************************************************
*�� �� ��: pid_control
*����˵��: pid����������
*��    ��: pid�������ṹ��
*�� �� ֵ: �����
**********************************************************************************************************/
float pid_control(pid_controler_t *controler)
{
	float controller_dt;
	float expect_delta;
	//��·ֱ������ڴ�ֵ
	if (controler->short_circuit_flag) {
		controler->control_output = controler->expect;
		return controler->control_output;
	}
	//��ȡdt
	Get_Time_Period(&controler->pid_controller_dt);
	controller_dt = controler->pid_controller_dt.Time_Delta / 1000000.0;
	//��һ�μ�����ʱ�佫���ּ��ʱ��ܴ�����
	if (controller_dt < 0.001)
		return 0;
	//�����ϴ�ƫ��
	controler->pre_last_err = controler->last_err;
	controler->last_err = controler->err;
	//������ȥ�����õ�ƫ��			  
	controler->err = controler->expect - controler->feedback;
	//����ƫ��΢��
	controler->dis_err = controler->err - controler->last_err;
	//�Զ���ƫ��΢�ִ���
	if (controler->err_callback)
		controler->err_callback(controler);
	//ƫ���޷��ȱ�־λ
	if (controler->err_max) {
		if (controler->err >= controler->err_max)
			controler->err = controler->err_max;
		if (controler->err <= -controler->err_max)
			controler->err = -controler->err_max;
	}
	//���ַ���
	if (controler->integrate_separation_err) {
		if (ABS(controler->err) <= controler->integrate_separation_err)
			controler->integrate += controler->ki * controler->err * controller_dt;
	} else {
		controler->integrate += controler->ki * controler->err * controller_dt;
	}
	//�����޷�
	if (controler->integrate_max) {
		if (controler->integrate >= controler->integrate_max)
			controler->integrate = controler->integrate_max;
		if (controler->integrate <= -controler->integrate_max)
			controler->integrate = -controler->integrate_max;
	}
	//���������
	controler->control_output = controler->kp * controler->err
		+ controler->integrate
		+ controler->kd * controler->dis_err;
	//ǰ������
	expect_delta = (controler->expect - controler->last_expect) / controller_dt;
	controler->last_expect = controler->expect;
	controler->control_output += controler->feedforward_kd * expect_delta
		+ controler->feedforward_kp * controler->expect;
	//������޷�
	if (controler->control_output_limit) {
		if (controler->control_output >= controler->control_output_limit)
			controler->control_output = controler->control_output_limit;
		if (controler->control_output <= -controler->control_output_limit)
			controler->control_output = -controler->control_output_limit;
	}
	//���������
	return controler->control_output;
}
