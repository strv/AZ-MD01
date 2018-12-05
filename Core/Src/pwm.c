#include "pwm.h"
#include "main.h"
#include "dac.h"

#define PWM_DUTY_MAX (99)
#define PWM_Period_MAX (PWM_DUTY_MAX * PWM_Period / 100)
#define PWM_ADC_OFFSET (0)

#define pwm_set_a(val) LL_TIM_OC_SetCompareCH2(PWM_TIM, val)
#define pwm_set_b(val) LL_TIM_OC_SetCompareCH3(PWM_TIM, val)

void pwm_init(void){
	// for PWM output
	LL_TIM_CC_EnableChannel(PWM_TIM, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(PWM_TIM, LL_TIM_CHANNEL_CH3);

	// for ADC trigger
	LL_TIM_CC_EnableChannel(PWM_TIM, LL_TIM_CHANNEL_CH5);
	LL_TIM_CC_EnableChannel(PWM_TIM, LL_TIM_CHANNEL_CH6);

	pwm_set_duty(0.);

	LL_TIM_EnableCounter(PWM_TIM);
	LL_TIM_EnableAllOutputs(PWM_TIM);
	//LL_TIM_EnableIT_UPDATE(PWM_TIM);
}

void pwm_set_duty(float percent){
	if(percent > 100.) percent = 100.;
	if(percent < -100.) percent = -100.;
	dac_set_mv(1, percent * 15 + 1500);

	int ton = PWM_Period_MAX * percent / 200. + PWM_Period / 2;

	pwm_set_a(ton);
	pwm_set_b(ton);

	LL_TIM_OC_SetCompareCH5(PWM_TIM, ton / 2 - PWM_ADC_OFFSET);
	LL_TIM_OC_SetCompareCH6(PWM_TIM, (PWM_Period + ton) / 2 - (PWM_ADC_OFFSET * 2));
}

inline void pwm_set_duty_nmrzd(float val){
	int ton = PWM_Period_MAX * val / 2. + PWM_Period / 2;

	pwm_set_a(ton);
	pwm_set_b(ton);

	LL_TIM_OC_SetCompareCH5(PWM_TIM, ton / 2 - PWM_ADC_OFFSET);
	LL_TIM_OC_SetCompareCH6(PWM_TIM, (PWM_Period + ton) / 2 - (PWM_ADC_OFFSET * 2));
}
