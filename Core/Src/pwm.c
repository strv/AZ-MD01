#include "pwm.h"
#include "main.h"
#include "dac.h"

#define PWM_DUTY_MAX (99)
#define PWM_Period_MAX (PWM_DUTY_MAX * PWM_Period / 100)
static const int32_t PWM_Period_Half = PWM_Period / 2;
#define PWM_ADC_OFFSET (0)

#define pwm_set_a(val) LL_TIM_OC_SetCompareCH2(PWM_TIM, val)
#define pwm_set_b(val) LL_TIM_OC_SetCompareCH3(PWM_TIM, val)

static float pwm_nmrz_gain;
int32_t ton;
int32_t ton_half;
float pwm_rate;

void pwm_init(void){
	// for PWM output
	LL_TIM_CC_EnableChannel(PWM_TIM, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(PWM_TIM, LL_TIM_CHANNEL_CH3);

	// for ADC trigger
	LL_TIM_CC_EnableChannel(PWM_TIM, LL_TIM_CHANNEL_CH5);
	LL_TIM_CC_EnableChannel(PWM_TIM, LL_TIM_CHANNEL_CH6);

	pwm_nmrz_gain = PWM_Period_MAX / 2.;
	pwm_set_duty(0.);

	LL_TIM_EnableCounter(PWM_TIM);
	LL_TIM_EnableAllOutputs(PWM_TIM);
	//LL_TIM_EnableIT_UPDATE(PWM_TIM);
}

void pwm_set_duty(float percent){
	if(percent > 100.) percent = 100.;
	if(percent < -100.) percent = -100.;
	pwm_set_duty_nmrzd(percent / 100.);
}

inline void pwm_set_duty_nmrzd(float val){
	pwm_rate = val;
	ton = val * pwm_nmrz_gain + PWM_Period_Half;
	ton_half = ton / 2;

	pwm_set_a(ton);
	pwm_set_b(ton);

	LL_TIM_OC_SetCompareCH5(PWM_TIM, ton_half - PWM_ADC_OFFSET);
	LL_TIM_OC_SetCompareCH6(PWM_TIM, PWM_Period_Half + ton_half - (PWM_ADC_OFFSET * 2));
}

inline float pwm_get_dir(){
	return pwm_rate;
}
