/*
 * LEDMatrix.h
 *
 *  Created on: Feb 25, 2021
 *      Author: yehen
 */

#ifndef SRC_LEDMATRIX_H_
#define SRC_LEDMATRIX_H_

#include "main.h"

#define LEDMatrix_Address 0b01001000

void LEDMatrixInit(uint8_t addr);
void LEDMatrixStart(uint8_t addr);
void LEDMatrixNextRow(uint8_t addr);

extern uint8_t LEDMatrix[4];
extern uint8_t LEDMatrixBuffer[12];
extern volatile uint8_t currentLEDRow;




#endif /* SRC_LEDMATRIX_H_ */
