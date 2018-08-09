#include "pwm.h"
#include "main.h"

#define pwm_set_a(val) LL_TIM_OC_SetCompareCH2(PWM_TIM, val)
#define pwm_set_b(val) LL_TIM_OC_SetCompareCH3(PWM_TIM, val)

void pwm_init(void){
	LL_TIM_CC_EnableChannel(PWM_TIM, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(PWM_TIM, LL_TIM_CHANNEL_CH3);
	LL_TIM_EnableCounter(PWM_TIM);
	LL_TIM_EnableAllOutputs(PWM_TIM);
}

void pwm_set_duty(float percent){
	if(percent > 100.) percent = 100.;
	if(percent < -100.) percent = -100.;

	if(percent > 0.){
		pwm_set_a(PWM_Period * percent / 100.);
		pwm_set_b(0);
	}else if(percent < 0){
		pwm_set_a(0.);
		pwm_set_b(PWM_Period * -percent / 100.);
	}else{
		pwm_set_a(0);
		pwm_set_b(0);
	}
}
