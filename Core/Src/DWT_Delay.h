/*
 * DWT_Delay.h
 *
 *  Created on: Feb 25, 2021
 *      Author: yehen
 */

#ifndef SRC_DWT_DELAY_H_
#define SRC_DWT_DELAY_H_

#include "main.h"

uint32_t DWT_Delay_Init(void);
void DWT_Delay_us(volatile uint32_t au32_microseconds);
void DWT_Delay_ms(volatile uint32_t au32_milliseconds);



#endif /* SRC_DWT_DELAY_H_ */
