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
#include "motor_profiles.h"
#include <math.h>

static const float delta_t = (float)(Control_Period + 1) / 64000000ULL;
static const float delta_t_inv = (float)64000000ULL / (Control_Period + 1);
#define  LMS_N (4)
static float lms_t[LMS_N];
static float lms_sum_x;
static float lms_co;
static const float Td = 0.01;
static float beta;
static const float eta = 0.1;
static float cur_fo = 1.;
static float vel_fo = 0.2;
static float pos_fo = 0.1;
static const float cur_dt_fo = 0.1;
static const float peak_r = 0.999;
static float volt_target = 0.;
static float cur_target = 0.;
static float vel_target = 0.;
static float pos_target = 0.;
//static float rotor_kv = 4.965; // volt / (m/s)
/* for 750 motor
static float cur_gain_kp = 12.;
static float cur_gain_ti = 0.00345;
static float cur_gain_td = 0.;
*/
/*
 * for polol
 * L = 160uH @ 100kHz
 * L = 1340uH @ 10kHz
 * C = 17nF @ 100kHz
 * C = 190nF @ 10kHz
 * DCR = 4 to 6
 * Latency = 125 usec
 * Time constant = 600 usec
 * K = 0.87 [A] / 16.5*0.2 [V]
 * Kp = 0.7 T / KL = 12.75
 * Ti = T = 600u
 */
static float rotor_kv = 1. / 20.3; // volt / rps
static float rotor_r = 6.;
static float rotor_l = 0.5e-3;
static const float blush_v_drop = 0.015;
static const float min_move_cur = 0.06;
static float cur_limit = 0.8;
static float vel_limit = 1.;
static float pos_limit_upper = 1.;
static float pos_limit_lower = 0.;
static float cur_gain_kp = 12.75;
static float cur_gain_ti = 0.0006;
static float cur_gain_td = 0.;
static float vel_gain_kp = 0;
static float vel_gain_ti = 0;
static float vel_gain_td = 0;
static float pos_gain_kp = 0;
static float pos_gain_ti = 0;
static float pos_gain_td = 0;
MT_VEL_SENSOR mt_vel_sensor = MT_VEL_BEMF;
MT_POS_SENSOR mt_pos_sensor = MT_POS_INT_VEL;
/* for alps slider */
/*
static float rotor_kv = 2.5; // volt / (m/s)
static float rotor_r = 19.;
static float rotor_l = 4.7e-3;
static const float blush_v_drop = 0.04;
static const float min_move_cur = 0.02;
static float cur_gain_kp = 45.;
static float cur_gain_ti = 0.001;
static float cur_gain_td = 0.;
//static float vel_gain_kp = 0.291;
//static float vel_gain_ti = 0.0085;
//static float vel_gain_td = 0.;
static float vel_gain_kp = 0.30;
static float vel_gain_ti = 0.005;
static float vel_gain_td = 0.0021;
static float pos_gain_kp = 4.;
static float pos_gain_ti = 0.00075;
static float pos_gain_td = 0.;
static float cur_limit = 0.8;
static float vel_limit = 1.5;
static float pos_limit_upper = 0.095;
static float pos_limit_lower = 0.005;
MT_VEL_SENSOR mt_vel_sensor = MT_VEL_DIV_POS;
MT_POS_SENSOR mt_pos_sensor = MT_POS_ADC;
*/
static int32_t control_mode = CTRL_DUTY;
static const int32_t c_v_ratio = 10;
static const int32_t v_p_ratio = 10;
static float vel = 0.;
static float pos[LMS_N];
static const float vel_eta = 0.01;
static float vel_beta;
static float vel_d_fo = 0.5;
float bemf_est = 0.;

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
	float lms_sum_x2;
	volatile int32_t i;
	beta = Td / (delta_t + eta * Td);
	LL_TIM_EnableCounter(CTRL_CUR_TIM);
	LL_TIM_EnableIT_UPDATE(CTRL_CUR_TIM);

	lms_sum_x = 0;
	lms_t[0] = 0.;
	lms_sum_x = lms_t[0];
	lms_sum_x2 = lms_t[0];
	for(i = 1;i < LMS_N; i++){
		lms_t[i] = lms_t[i - 1] - delta_t;
		lms_sum_x += lms_t[i];
		lms_sum_x2 += lms_t[i] * lms_t[i];
	}
	lms_co = 1. / (lms_sum_x2 * (float)LMS_N - lms_sum_x * lms_sum_x);

	vel_beta = vel_gain_td / (delta_t * c_v_ratio + vel_eta * vel_gain_td);
}

inline void ctrl_cur_irq(void){
	static int32_t v_phase = 0;
	static int32_t p_phase = 0;
	static float vbatt = 0.;
	static float vbatt_prev = 0.;
	static float volt = 0.;
	static float volt_prev = 0.;
	static float volt_ff = 0.;
	static float cur_meas = 0.;
	static float cur_tmp = 0.;
	static float cur_meas_prev = 0.;

	static float cur_delta[3] = {0., 0., 0.};
	static float cur_i_switch = 1.;
	static float cur_delta_i;
	static float cur_target_prev = 0.;

	static float vel_delta[3] = {0., 0., 0.};
	static float vel_prev = 0.;
	static float vel_i_switch = 1.;
	static float vel_delta_i;
	static float vel_delta_d;
	static float vel_d = 0.;
	static float vel_target_prev = 0.;

	static float pos_delta[3] = {0., 0., 0.};
	static float pos_i_switch = 1.;
	static float pos_delta_i;
	static float pos_i_adapt = 0.;
	static float lms_sum_xy, lms_sum_y;

	gpio_ext_set();
	vbatt = adc_get_vbatt();
	vbatt = vbatt * cur_fo + vbatt_prev * (1. - cur_fo);
	cur_meas = adc_get_cur() * cur_fo + cur_meas_prev * (1. - cur_fo);
	pos[0] = (adc_get_ext_rate() * 0.1) * pos_fo + pos[1] * (1. - pos_fo);
	if(mt_vel_sensor & MT_VEL_BEMF){
		vel = bemf / rotor_kv;
	}else if(mt_vel_sensor & MT_VEL_DIV_POS){
		vel = ((pos[0] - pos[1]) * delta_t_inv) * vel_fo + vel_prev * (1. - vel_fo);
	}else if(mt_vel_sensor & MT_VEL_RIPPLE_CNT){
		rotor_pos_estimater(&cur_meas, &cur_meas_prev);
	}

	if(p_phase == 0 && control_mode >= CTRL_POS){
		pos_delta[0] = (pos_target - pos[0]);
		pos_delta_i = delta_t * c_v_ratio * v_p_ratio / pos_gain_ti * pos_delta[0];

		vel_target = //vel_target_prev +
				pos_gain_kp * (
						(pos_delta[0] - pos_delta[1])
						+ pos_delta_i
				);
		pos_i_adapt = vel_target;
		if(vel_target > vel_limit){
			vel_target = vel_limit;
		}else if(vel_target < -vel_limit){
			vel_target = -vel_limit;
		}
		pos_i_adapt -= vel_target;
		pos_i_adapt /= pos_gain_kp;
		vel_target_prev = vel_target;
		pos_delta[2] = pos_delta[1];
		pos_delta[1] = pos_delta[0];
	}

	if(v_phase == 0 && control_mode >= CTRL_VEL){
		if(pos[0] > pos_limit_upper && vel_target > 0.){
			vel_target = 0.;
		}else if(pos[0] < pos_limit_lower && vel_target < 0.){
			vel_target = 0.;
		}
		vel_delta[0] = (vel_target - vel);
		vel_delta_i = delta_t * c_v_ratio / vel_gain_ti * vel_delta[0];
		vel_delta_d = (vel_eta * vel_beta - 1.) * vel_d
				+ vel_beta * (vel_prev - vel);
		vel_d += vel_delta_d;
		cur_tmp = cur_tmp_prev +
				vel_gain_kp * (
						(vel_delta[0] - vel_delta[1])
						+ vel_delta_i * vel_i_switch
						+ vel_delta_d
				);
		cur_tmp_prev = cur_tmp;

		if(cur_tmp > 0.){
			cur_tmp += min_move_cur;
		}else if(cur_tmp < 0.){
			cur_tmp -= min_move_cur;
		}
		if(cur_tmp > cur_limit){
			vel_i_switch = 0.;
			cur_tmp = cur_limit;
		}else if(cur_tmp < -cur_limit){
			vel_i_switch = 0.;
			cur_tmp = -cur_limit;
		}else{
			vel_i_switch = 1.;
		}

		cur_target = cur_tmp;

		vel_delta[2] = vel_delta[1];
		vel_delta[1] = vel_delta[0];

		p_phase++;
		if(p_phase >= v_p_ratio){
			p_phase = 0;
		}
	}

	if(control_mode >= CTRL_CUR){
		cur_delta[0] = (cur_target - cur_meas);
		cur_delta_i = delta_t / cur_gain_ti * cur_delta[0];
		volt = volt_prev
				+ cur_gain_kp * (
						(cur_delta[0] - cur_delta[1])
						+ cur_delta_i * cur_i_switch
						);

		volt_prev = volt;
		if(mt_vel_sensor & MT_VEL_BEMF){
			bemf_est = volt;
			volt += cur_target * rotor_r;// - (cur_target - cur_target_prev) * rotor_l / delta_t;
		}else{
			volt += cur_target * rotor_r + vel * rotor_kv;
		}
		if(volt > 0.){
			volt += blush_v_drop;
		}else if(volt < 0.){
			volt -= blush_v_drop;
		}
		if(volt > vbatt && cur_delta_i > 0.){
			cur_i_switch = 0.;
			volt = vbatt;
		}else if(volt < -vbatt && cur_delta_i < 0.){
			cur_i_switch = 0.;
			volt = -vbatt;
		}else{
			cur_i_switch = 1.;
		}

		volt_target = volt;

		cur_delta[2] = cur_delta[1];
		cur_delta[1] = cur_delta[0];

		v_phase++;
		if(v_phase >= c_v_ratio){
			v_phase = 0;
		}
	}

	if(control_mode >= CTRL_VOLT){
		pwm_set_duty_nmrzd(volt_target / vbatt);
	}

	cur_target_prev = cur_target;
	cur_meas_prev = cur_meas;
	vel_prev = vel;
	vbatt_prev = vbatt;
	for(int i = 1; i < LMS_N; i++){
		pos[i] = pos[i - 1];
	}

	//dac_set_mv(DAC_CH1, cur_target * 500 + 1500);
	dac_set_mv(DAC_CH2, cur_meas * 500 + 1500);
	//dac_set_mv(DAC_CH1, vel_target * 50000 + 1500);
	//dac_set_mv(DAC_CH2, vel * 2000 + 1500);
	//dac_set_mv(DAC_CH1, vel_delta[0] * 2000 + 1500);
	//dac_set_mv(DAC_CH2, vel_delta_d * 2000 + 1500);
	//dac_set_mv(DAC_CH2, vel_d * 2000 + 1500);
	//dac_set_mv(DAC_CH1, pos_target * 30000);
	//dac_set_mv(DAC_CH2, pos[0] * 30000);
	gpio_ext_reset();
}

void ctrl_set_mode(int32_t mode){
	control_mode = mode;
	switch (mode){
		case CTRL_POS:
			ctrl_set_pos(0.);
		case CTRL_VEL:
			ctrl_set_vel(0.);
		case CTRL_CUR:
			ctrl_set_cur(0.);
		case CTRL_VOLT:
			ctrl_set_volt(0.);
		case CTRL_DUTY:
			pwm_set_duty(0.);
	}
}

void ctrl_set_volt(float voltage){
	volt_target = voltage;
}

void ctrl_set_cur(float current){
	cur_target = current;
}

void ctrl_set_vel(float vel){
	vel_target = vel;
}

void ctrl_set_pos(float pos){
	pos_target = pos;
}

void ctrl_set_cur_gain(float kp, float ti, float td){
	cur_gain_kp = kp;
	cur_gain_ti = ti;
	cur_gain_td = td;
}

void ctrl_set_vel_gain(float kp, float ti, float td){
	vel_gain_kp = kp;
	vel_gain_ti = ti;
	vel_gain_td = td;
	vel_beta = vel_gain_td / (delta_t * c_v_ratio + vel_eta * vel_gain_td);
}

void ctrl_set_pos_gain(float kp, float ti, float td){
	pos_gain_kp = kp;
	pos_gain_ti = ti;
	pos_gain_td = td;
}

void ctrl_set_cur_lpf(float fo){
	cur_fo = fo;
}

void ctrl_set_vel_lpf(float fo){
	vel_fo = fo;
}

void ctrl_set_pos_lpf(float fo){
	pos_fo = fo;
}

float ctrl_get_vel(){
	return vel;
}

float ctrl_get_pos(){
	return pos[0];
}
