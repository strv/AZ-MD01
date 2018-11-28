/*
 * control.h
 *
 *  Created on: 2018/11/28
 *      Author: STRV
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#define CTRL_CUR_TIM TIM6

void ctrl_init(void);
void ctrl_cur_irq(void);

#endif /* INC_CONTROL_H_ */
