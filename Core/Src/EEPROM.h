/*
 * EEPROM.h
 *
 *  Created on: 22 Apr 2021
 *      Author: yehen
 */

#ifndef SRC_EEPROM_H_
#define SRC_EEPROM_H_


#include <stdint.h>

/*
 * Serialisation Format
 *
 * Bytes: (all values int8_t unless specified)
 * 0 Midi Note Offset
 * 1 Midi Note Velo
 * 2 Midi Channel
 * 3-5 Midi CC Fader LUT
 * 6-9 Midi CC Encoder LUT
 * 10-13 Encoder Speed
 * 14-17 Encoder Note
 * 18 Filter Beta
 */

extern uint8_t EEPROMQueued;
extern uint8_t EEPROMData;
extern uint16_t EEPROMAddress;


//implement a FIFO
extern uint8_t eepromDataQueue[64];
extern volatile int8_t inputEEPROMBufferPosition;
extern volatile int8_t outputEEPROMBufferPosition; //this will be incremented in an interrupt




void EEPROMWriteParameter(uint16_t parameterAddress, uint8_t value);





#endif /* SRC_EEPROM_H_ */
