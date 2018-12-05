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
	CTRL_SPD_BEMF,
	CTRL_SPD_ENC,
	CTRL_POS
};
void ctrl_init(void);
void ctrl_cur_irq(void);
void ctrl_set_mode(int32_t mode);
void ctrl_set_cur(float current);
void ctrl_set_volt(float volt);
void ctrl_set_cur_gain(float kp, float ti, float td);

#endif /* INC_CONTROL_H_ */
