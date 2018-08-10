/*
 * encoder.h
 *
 *  Created on: 2018/08/10
 *      Author: STRV
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#define ENC_TIM TIM3

#include "main.h"

void enc_init(void);
uint32_t enc_get(void);

#endif /* INC_ENCODER_H_ */
