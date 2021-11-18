#include "pid.h"

#define ABS(X)  (((X) > 0)? (X) : -(X))

/**********************************************************************************************************
*函 数 名: pid_control
*功能说明: pid控制器计算
*形    参: pid控制器数据结构体 pid控制器参数
*返 回 值: 输出量
**********************************************************************************************************/
float pid_control(pid_data_t *data, pid_paramer_t *para)
{
	float controller_dt;
    //短路直接输出期待值
	if (data->short_circuit_flag) {
		data->control_output = data->expect;
		return data->control_output;
	}
	//获取dt
	Get_Time_Period(&data->pid_controller_dt);
	controller_dt = data->pid_controller_dt.Time_Delta / 1000000.0;
	//第一次计算间隔时间将出现间隔时间很大的情况
	if (controller_dt < 0.001f)
		return 0;
	//保存上次偏差
	data->pre_last_err = data->last_err;
	data->last_err = data->err;
	//期望减去反馈得到偏差			  
	data->err = data->expect - data->feedback;
	//计算偏差微分
	data->dis_err = data->err - data->last_err;
	//自定义偏差微分处理
	if (data->err_callback)
		data->err_callback(data, para);
	//偏差限幅度标志位
	if (para->err_max) {
		if (data->err >= para->err_max)
			data->err = para->err_max;
		if (data->err <= -para->err_max)
			data->err = -para->err_max;
	}
	//积分分离
	if (para->integrate_separation_err) {
		if (ABS(data->err) <= para->integrate_separation_err)
			data->integrate += para->ki * data->err * controller_dt;
	} else {
		data->integrate += para->ki * data->err * controller_dt;
	}
	//积分限幅
	if (para->integrate_max) {
		if (data->integrate >= para->integrate_max)
			data->integrate = para->integrate_max;
		if (data->integrate <= -para->integrate_max)
			data->integrate = -para->integrate_max;
	}
	//总输出计算
	data->control_output = para->kp * data->err
		+ data->integrate
		+ para->kd * data->dis_err;
	//总输出限幅
	if (para->control_output_limit) {
		if (data->control_output >= para->control_output_limit)
			data->control_output = para->control_output_limit;
		if (data->control_output <= -para->control_output_limit)
			data->control_output = -para->control_output_limit;
	}
	//返回总输出
	return data->control_output;
}
