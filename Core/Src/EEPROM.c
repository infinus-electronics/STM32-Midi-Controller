/*
 * EEPROM.c
 *
 *  Created on: 25 Apr 2021
 *      Author: yehen
 *
 */

#include "EEPROM.h"


//implement a FIFO
uint8_t eepromDataQueue[64];
volatile int8_t inputEEPROMBufferPosition = 0;
volatile int8_t outputEEPROMBufferPosition = 0; //this will be incremented in an interrupt


void EEPROMWriteParameter(uint16_t parameterAddress, uint8_t value){

	eepromDataQueue[inputEEPROMBufferPosition] = (uint8_t)(parameterAddress >> 8); //load the address high byte into the queue
	inputEEPROMBufferPosition = ((inputEEPROMBufferPosition + 1) % (sizeof(eepromDataQueue)/sizeof(eepromDataQueue[0]))); //sizeof is compile time
	eepromDataQueue[inputEEPROMBufferPosition] = (uint8_t)(parameterAddress & 0xff); //load the address low byte into the queue
	inputEEPROMBufferPosition = ((inputEEPROMBufferPosition + 1) % (sizeof(eepromDataQueue)/sizeof(eepromDataQueue[0]))); //sizeof is compile time
	eepromDataQueue[inputEEPROMBufferPosition] = value; //load the data into the queue
	inputEEPROMBufferPosition = ((inputEEPROMBufferPosition + 1) % (sizeof(eepromDataQueue)/sizeof(eepromDataQueue[0]))); //sizeof is compile time

}
