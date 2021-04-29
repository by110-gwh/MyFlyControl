#include "pid.h"

#define ABS(X)  (((X) > 0)? (X) : -(X))

/**********************************************************************************************************
*函 数 名: pid_control
*功能说明: pid控制器计算
*形    参: pid控制器结构体
*返 回 值: 输出量
**********************************************************************************************************/
float pid_control(pid_controler_t *controler)
{
	float controller_dt;
	float expect_delta;
	//短路直接输出期待值
	if (controler->short_circuit_flag) {
		controler->control_output = controler->expect;
		return controler->control_output;
	}
	//获取dt
	Get_Time_Period(&controler->pid_controller_dt);
	controller_dt = controler->pid_controller_dt.Time_Delta / 1000000.0;
	//第一次计算间隔时间将出现间隔时间很大的情况
	if (controller_dt < 0.001)
		return 0;
	//保存上次偏差
	controler->pre_last_err = controler->last_err;
	controler->last_err = controler->err;
	//期望减去反馈得到偏差			  
	controler->err = controler->expect - controler->feedback;
	//计算偏差微分
	controler->dis_err = controler->err - controler->last_err;
	//自定义偏差微分处理
	if (controler->err_callback)
		controler->err_callback(controler);
	//偏差限幅度标志位
	if (controler->err_max) {
		if (controler->err >= controler->err_max)
			controler->err = controler->err_max;
		if (controler->err <= -controler->err_max)
			controler->err = -controler->err_max;
	}
	//积分分离
	if (controler->integrate_separation_err) {
		if (ABS(controler->err) <= controler->integrate_separation_err)
			controler->integrate += controler->ki * controler->err * controller_dt;
	} else {
		controler->integrate += controler->ki * controler->err * controller_dt;
	}
	//积分限幅
	if (controler->integrate_max) {
		if (controler->integrate >= controler->integrate_max)
			controler->integrate = controler->integrate_max;
		if (controler->integrate <= -controler->integrate_max)
			controler->integrate = -controler->integrate_max;
	}
	//总输出计算
	controler->control_output = controler->kp * controler->err
		+ controler->integrate
		+ controler->kd * controler->dis_err;
	//前馈计算
	expect_delta = (controler->expect - controler->last_expect) / controller_dt;
	controler->last_expect = controler->expect;
	controler->control_output += controler->feedforward_kd * expect_delta
		+ controler->feedforward_kp * controler->expect;
	//总输出限幅
	if (controler->control_output_limit) {
		if (controler->control_output >= controler->control_output_limit)
			controler->control_output = controler->control_output_limit;
		if (controler->control_output <= -controler->control_output_limit)
			controler->control_output = -controler->control_output_limit;
	}
	//返回总输出
	return controler->control_output;
}
