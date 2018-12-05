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
#include "pwm.h"
#include <math.h>

static const float delta_t = 0.0001;
static const float Td = 0.01;
static float beta;
static const float eta = 0.1;
static const float cur_fo = 1.;
static const float cur_dt_fo = 0.1;
static const float peak_r = 0.999;
static float cur_target = 0.;
static float volt_target = 0.;
static float rotor_r;
static float rotor_l;
static float cur_gain_kp = 12.;
static float cur_gain_ti = 0.00345;
static float cur_gain_td = 0.;
static int32_t control_mode = CTRL_DUTY;

void rotor_pos_estimater(float* pcur, float* pcur_prev){
	static float cur_dt;
	static float cur_dt_raw;
	static float cur_dt_prev = 0.;
	static float cur_dt_lpfed;
	static float cur_dt_lpfed_prev = 0.;
	static float amp;
	static float peak = 0.;

	cur_dt_raw = cur_dt_prev + (eta * beta - 1.) * cur_dt_prev + beta * (*pcur - *pcur_prev);
	amp = fabs(cur_dt_raw);
	if(amp > peak){
		peak = amp;
	}
	cur_dt = cur_dt_raw / peak;
	cur_dt_lpfed = cur_dt_lpfed_prev * (1. - cur_dt_fo) + cur_dt * cur_dt_fo;
	cur_dt_prev = cur_dt_raw;
	cur_dt_lpfed_prev = cur_dt_lpfed;
	peak *= peak_r;
/*
	if(cur_dt - cur_dt_lpfed > 0.1){
		gpio_ext_set();
	}else if(cur_dt - cur_dt_lpfed < -0.1){
		gpio_ext_reset();
	}
	dac_set_mv(1, cur_dt * 1500 + 1500);
	dac_set_mv(2, cur_dt_lpfed * 1500 + 1500);
*/
}

void ctrl_init(void){
	beta = Td / (delta_t + eta * Td);
	LL_TIM_EnableCounter(CTRL_CUR_TIM);
	LL_TIM_EnableIT_UPDATE(CTRL_CUR_TIM);
}

inline void ctrl_cur_irq(void){
	static float cur = 0.;
	static float cur_prev = 0.;
	static float vbatt = 0.;
	static float vbatt_prev = 0.;
	static float volt = 0.;
	static float volt_prev = 0.;
	static float cur_delta[3];
	static float i_switch = 1.;
	static float cur_delta_i;

	vbatt = adc_get_vbatt() * cur_fo + vbatt_prev * (1. - cur_fo);
	cur = adc_get_cur_ave() * cur_fo + cur_prev * (1. - cur_fo);
	rotor_pos_estimater(&cur, &cur_prev);

	if(control_mode >= CTRL_CUR){
		cur_delta[0] = (cur_target - cur);
		cur_delta_i = delta_t / cur_gain_ti * cur_delta[0];
		volt = volt_prev
				+ cur_gain_kp * (
						(cur_delta[0] - cur_delta[1])
						+ cur_delta_i * i_switch
						);

		if(volt > vbatt && cur_delta_i > 0.){
			i_switch = 0.;
			volt = vbatt;
		}else if(volt < -vbatt && cur_delta_i < 0.){
			i_switch = 0.;
			volt = -vbatt;
		}else{
			i_switch = 1.;
		}

		pwm_set_duty_nmrzd(volt / vbatt);

		volt_prev = volt;
		cur_delta[2] = cur_delta[1];
		cur_delta[1] = cur_delta[0];
	}
	cur_prev = cur;
	vbatt_prev = vbatt;

//	dac_set_mv(1, cur_target * 500 + 1500);
	dac_set_mv(2, cur * 500 + 1500);
}

void ctrl_set_mode(int32_t mode){
	control_mode = mode;
}

void ctrl_set_cur(float current){
	cur_target = current;
}

void ctrl_set_volt(float volt){
	volt_target = volt;
}

void ctrl_set_cur_gain(float kp, float ti, float td){
	cur_gain_kp = kp;
	cur_gain_ti = ti;
	cur_gain_td = td;
}
