/*
 * pwm.h
 *
 *  Created on: 2018/08/09
 *      Author: STRV
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#define PWM_TIM TIM1

#define pwm_md_enable() LL_GPIO_SetOutputPin(MD_EN_GPIO_Port, MD_EN_Pin)
#define pwm_md_disable() LL_GPIO_ResetOutputPin(MD_EN_GPIO_Port, MD_EN_Pin)

void pwm_init(void);
void pwm_set_duty(float percent);


#endif /* INC_PWM_H_ */
