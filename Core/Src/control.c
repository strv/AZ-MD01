/*
 * control.c
 *
 *  Created on: 2018/11/28
 *      Author: STRV
 */

#include "control.h"
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "gpio.h"
#include "math.h"

static const float delta_t = 0.0001;
static const float Td = 0.01;
static float beta;
static const float eta = 0.1;
static const float cur_fo = 0.05;
static const float cur_dt_fo = 0.1;
static const float peak_r = 0.999;

void ctrl_init(void){
	beta = Td / (delta_t + eta * Td);
	LL_TIM_EnableCounter(CTRL_CUR_TIM);
	LL_TIM_EnableIT_UPDATE(CTRL_CUR_TIM);
}

inline void ctrl_cur_irq(void){
	static float cur;
	static float cur_dt;
	static float cur_dt_raw;
	static float cur_prev = 0.;
	static float cur_dt_prev = 0.;
	static float cur_dt_lpfed;
	static float cur_dt_lpfed_prev = 0.;
	static float amp;
	static float peak = 0.;

	cur = adc_get_cur_ave() * cur_fo + cur_prev * (1. - cur_fo);
	cur_dt_raw = cur_dt_prev + (eta * beta - 1.) * cur_dt_prev + beta * (cur - cur_prev);
	amp = fabs(cur_dt_raw);
	if(amp > peak){
		peak = amp;
	}
	cur_dt = cur_dt_raw / peak;
	cur_dt_lpfed = cur_dt_lpfed_prev * (1. - cur_dt_fo) + cur_dt * cur_dt_fo;
	dac_set_mv(1, cur_dt * 1500 + 1500);
	dac_set_mv(2, cur_dt_lpfed * 1500 + 1500);
	cur_prev = cur;
	cur_dt_prev = cur_dt_raw;
	cur_dt_lpfed_prev = cur_dt_lpfed;
	peak *= peak_r;
	if(cur_dt - cur_dt_lpfed > 0.1){
		gpio_ext_set();
	}else if(cur_dt - cur_dt_lpfed < -0.1){
		gpio_ext_reset();
	}
}
