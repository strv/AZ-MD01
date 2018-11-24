/*
 * encoder.c
 *
 *  Created on: 2018/08/10
 *      Author: STRV
 */

#include "encoder.h"

void enc_init(void){
	LL_TIM_CC_EnableChannel(ENC_TIM, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(ENC_TIM, LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(ENC_TIM);
}

uint32_t enc_get(void){
	return LL_TIM_GetCounter(ENC_TIM);
}
