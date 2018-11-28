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

static float cur_prev;
static float cur_dt;
static float cur_dt_prev;
static const float delta_t = 0.0001;
static const float Td = 0.001;
static float beta;
static float eta = 0.1;

void ctrl_init(void){
	cur_prev = 0.;
	cur_dt = 0.;
	cur_dt_prev = 0.;
	beta = Td / (delta_t + eta * Td);
	LL_TIM_EnableCounter(CTRL_CUR_TIM);
	LL_TIM_EnableIT_UPDATE(CTRL_CUR_TIM);
}

inline void ctrl_cur_irq(void){
	float cur = adc_get_cur_ave();
	dac_set_mv(1, cur * 1000 + 3300/2);
	cur_dt = cur_dt_prev + (eta * beta - 1.) * cur_dt_prev + beta * (cur - cur_prev);
	dac_set_mv(2, cur_dt * 2. * 1000 + 3300/2);
	cur_prev = cur;
	cur_dt_prev = cur_dt;
}
