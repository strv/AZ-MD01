/*
 * control.h
 *
 *  Created on: 2018/11/28
 *      Author: STRV
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#define CTRL_CUR_TIM TIM6

#include <stdint.h>
#include <stdbool.h>

typedef enum{
	CTRL_DUTY,
	CTRL_CUR,
	CTRL_VEL,
	CTRL_POS
}CTRL_MODES;

void ctrl_init(void);
void ctrl_cur_irq(void);
void ctrl_set_mode(int32_t mode);
void ctrl_set_cur(float current);
void ctrl_set_vel(float vel);
void ctrl_set_pos(float pos);
void ctrl_set_cur_gain(float kp, float ti, float td);
void ctrl_set_vel_gain(float kp, float ti, float td);
void ctrl_set_pos_gain(float kp, float ti, float td);
void ctrl_set_cur_lpf(float fo);
void ctrl_set_vel_lpf(float fo);
void ctrl_set_pos_lpf(float fo);
float ctrl_get_vel();
float ctrl_get_pos();

#endif /* INC_CONTROL_H_ */
