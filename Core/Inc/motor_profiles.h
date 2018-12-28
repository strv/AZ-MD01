/*
 * motor_profiles.h
 *
 *  Created on: 2018/12/28
 *      Author: STRV
 */

#ifndef INC_MOTOR_PROFILES_H_
#define INC_MOTOR_PROFILES_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum{
	MT_VEL_BEMF			= (1 << 0),
	MT_VEL_RIPPLE_CNT	= (1 << 1),
	MT_VEL_ENC			= (1 << 2),
	MT_VEL_ADC			= (1 << 3),
	MT_VEL_DIV_POS		= (1 << 4)
}MT_VEL_SENSOR;

typedef enum{
	MT_POS_INT_VEL		= (1 << 0),
	MT_POS_INCENC		= (1 << 1),
	MT_POS_ABSENC		= (1 << 2),
	MT_POS_ADC			= (1 << 3)
}MT_POS_SENSOR;

typedef struct{
	float rotor_kv = 1. / 20.3; // volt / rps
	float rotor_r = 6.;
	float rotor_l = 0.5e-3;
	float blush_v_drop = 0.015;
	float min_move_cur = 0.06;
	float cur_limit = 0.8;
	float vel_limit = 1.;
	float pos_limit_upper = 1.;
	float pos_limit_lower = 0.;
	float cur_gain_kp = 12.75;
	float cur_gain_ti = 0.0006;
	float cur_gain_td = 0.;
	float vel_gain_kp = 0;
	float vel_gain_ti = 0;
	float vel_gain_td = 0;
	float pos_gain_kp = 0;
	float pos_gain_ti = 0;
	float pos_gain_td = 0;
}MT_Prof;

#endif /* INC_MOTOR_PROFILES_H_ */
