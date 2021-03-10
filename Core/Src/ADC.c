/*
 * ADC.c
 *
 *  Created on: Mar 10, 2021
 *      Author: yehen
 */
#include "main.h"


int ADC1ReadVal8(uint8_t channel){

	//ADC1->CR2 &= ~(1);
	ADC1->SQR3 &= ~(0x1f);
	ADC1->SQR3 |= channel & 0x1f;
	//ADC1->CR2 |= 1;
	//ADC1->CR2 |= 1;
	ADC1->CR2 |= (1<22);
	while((ADC1->SR & (1<<1)) == 0); //wait for EOC
	return ((ADC1->DR & 0x0fff)>>4);



}
