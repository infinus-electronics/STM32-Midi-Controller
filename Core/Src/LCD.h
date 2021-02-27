/*
 * LCD.h
 *
 *  Created on: Feb 25, 2021
 *      Author: yehen
 */

#ifndef SRC_LCD_H_
#define SRC_LCD_H_

#include "main.h"



#define LCD_Address 0b01001110

typedef enum bank {A, B} bank;

void LCDCommand(char data, uint8_t addr);
void LCDData(char data, uint8_t addr);
void LCDInit(uint8_t addr);
void LCDWriteChar(char c, uint8_t addr);
void LCDWriteString(char *str, uint8_t addr);
void LCDClear(uint8_t addr);
void LCDCycleEN(uint8_t addr);
void LCDSetCursor(uint8_t row, uint8_t col, uint8_t addr);
void LCDShiftRight(uint8_t addr);
void LCDShiftLeft(uint8_t addr);

void LCDPrepareInt();
void LCDWriteStringInt(uint8_t section);
void LCDPrintString(char* str, uint8_t line);
extern volatile uint8_t updateLCD;
extern volatile uint8_t currentLCDSection;
extern uint8_t LCDBuffer[36];
extern volatile uint8_t cycleEN;
extern volatile uint8_t currentLCDByte;

void MCP23017SetPin(uint8_t pin, bank b, uint8_t address);
void MCP23017ClearPin(uint8_t pin, bank b, uint8_t address);



extern uint8_t currentIOState[2]; //current state of the LCD MCP23017



#endif /* SRC_LCD_H_ */
