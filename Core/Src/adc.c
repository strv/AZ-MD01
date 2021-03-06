/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "pwm.h"
static int32_t phase = 0;
int32_t adc1_results[ADC_BUFF_LEN];
static float cur_zero_a;
static float cur_zero_b;
const static float vref_real = 1.23;	// vref voltage in volt
static float vref_raw = 1526.3181818181818181818181818182;	// vref voltage in ADC raw value
static int32_t cur_zero_a_i;
static int32_t cur_zero_b_i;
static const float cur_zero_lpf_fo = 0.01;
static float vbatt_gain;
static float cur_gain;
static float ext_gain;
const static float ext_rate_gain = 0.00012210012210012210012210012210012;	// 1.0 / (4095.0 * 2.0)
static float ext_gain;
const static float r_sense = 0.1;
const static float cur_amp_gain = 3.3;
const static float vbatt_amp_gain = 1. / 11.;
/* USER CODE END 0 */

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**ADC1 GPIO Configuration  
  PA0   ------> ADC1_IN1
  PA1   ------> ADC1_IN2
  PB0   ------> ADC1_IN11
  PB1   ------> ADC1_IN12 
  */
  GPIO_InitStruct.Pin = ADC_CUR_A_Pin|ADC_CUR_B_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ADC_VB_Pin|ADC_EXT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ADC1 DMA Init */
  
  /* ADC1 Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);

  /* ADC1 interrupt Init */
  NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(ADC1_2_IRQn);

  /**Common config 
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM1_TRGO2;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV1;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  /**Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);
  /**Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_2);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SINGLE_ENDED);
  /**Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_11);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SINGLE_ENDED);
  /**Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_12);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_12, LL_ADC_SINGLE_ENDED);
  /**Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_5, LL_ADC_CHANNEL_VREFINT);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_181CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);

}

/* USER CODE BEGIN 1 */
void ADC1_2_IRQHandler(void)
{
	if(LL_ADC_IsActiveFlag_OVR(ADC1)){
		phase = 0;
	}
	phase++;
	phase &= ADC_SAM_NUM - 1;
	LL_ADC_ClearFlag_EOS(ADC1);
}

void adc_init(void){
	LL_ADC_EnableInternalRegulator(ADC1);
	LL_mDelay(1);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_BUFF_LEN);
	LL_DMA_ConfigAddresses(	DMA1,
							LL_DMA_CHANNEL_1,
							LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
							(uint32_t)adc1_results,
							LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
	while(LL_ADC_IsCalibrationOnGoing(ADC1));
	LL_mDelay(1);
	LL_ADC_Enable(ADC1);
	LL_mDelay(1);
	//LL_ADC_EnableIT_EOC(ADC1);
	LL_ADC_EnableIT_EOS(ADC1);
	LL_ADC_REG_StartConversion(ADC1);
}

inline int32_t adc_get(uint32_t ch){
	//if(ch >= ADC_BUFF_LEN)return -1;
	return adc1_results[ch];
}

inline float adc_get_vbatt(){
	return (float)(adc1_results[2 + phase * 10] + adc1_results[7 + phase * 10]) * vbatt_gain;
}

inline float adc_get_vbatt_ave(){
	int32_t v = 0.;
	for(int i = 2; i < ADC_BUFF_LEN; i+=5){
		v += adc1_results[i];
	}
	return v * vbatt_gain / ADC_SAM_NUM;
}

void adc_cur_cal_proc(){
	cur_zero_a = cur_zero_a * (1. - cur_zero_lpf_fo) + ((float)adc1_results[5 + phase / 2 * 10]) * cur_zero_lpf_fo;
	cur_zero_b = cur_zero_b * (1. - cur_zero_lpf_fo) + ((float)adc1_results[1 + phase / 2 * 10]) * cur_zero_lpf_fo;
	vref_raw = vref_raw * (1. - cur_zero_lpf_fo) + (float)adc1_results[4 + phase / 2 * 10] * cur_zero_lpf_fo;
	cur_zero_a_i = cur_zero_a;
	cur_zero_b_i = cur_zero_b;
	vbatt_gain = vref_real / vref_raw / vbatt_amp_gain / 2.;	//Vbatt は、2つのADCの結果を毎回使えるのでゲインを半分にしておく
	cur_gain = vref_real / vref_raw / r_sense / cur_amp_gain;
}

inline float adc_get_cur(){
	static float gain, gain_inv;
	static float a, b;
	gain = pwm_get_dir() * 5. + 0.5;
	if(gain > 1.){
		gain = 1.;
	}else if(gain < 0.){
		gain = 0.;
	}
	gain_inv = 1. - gain;
	a = adc1_results[5 + phase * 10] - cur_zero_a_i;
	b = cur_zero_b_i - adc1_results[1 + phase * 10];
	return (a * gain_inv + b * gain) * cur_gain;
}

inline float adc_get_cur_ave(){
	int32_t a = 0;
	int32_t b = 0;
	float gain, gain_inv;
	gain = pwm_get_dir() * 5. + 0.5;
	if(gain > 1.){
		gain = 1.;
	}else if(gain < 0.){
		gain = 0.;
	}
	gain_inv = 1. - gain;
	for(int i = 0; i < ADC_SAM_NUM; i++){
		a += adc1_results[5 + i*10] - cur_zero_a_i;
		b += cur_zero_b_i - adc1_results[1 + i*10];
	}
	return (a * gain_inv + b * gain) * cur_gain / ADC_SAM_NUM;
}

inline int16_t adc_get_cur_zero(uint32_t side){
	if(side == 0){
		return cur_zero_a;
	}
	return cur_zero_b;
}

inline float adc_get_ext(){
	float v = 0.;
	for(int i = 3; i < ADC_BUFF_LEN; i+=5){
		v += (float)adc1_results[i];
	}
	return v * 1.23 / vref_raw / (ADC_SAM_NUM * 2);
}

/**
 * \fn
 * 外部ADC入力端子の値を割合で取得する関数。最新の値だけを使う。
 * \return 0.0 to 1.0
 **/
inline float adc_get_ext_rate(){
	//return (float)(adc1_results[3] + adc1_results[8]) / 4095. / 2.;
	return (float)(adc1_results[3 + phase * 10] + adc1_results[8 + phase * 10]) * ext_rate_gain;
}

/**
 * \fn
 * 外部ADC入力端子の値を割合で取得する関数。DMAバッファ中の全部の値をつかって平均を取る。
 * \return 0.0 to 1.0
 */
inline float adc_get_ext_rate_ave(){
	int32_t v = 0.;
	for(int i = 3; i < ADC_BUFF_LEN; i+=5){
		v += adc1_results[i];
	}
	return (float)v * ext_rate_gain / ADC_SAM_NUM;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
