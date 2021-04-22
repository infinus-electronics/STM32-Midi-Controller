/*
 * EEPROM.h
 *
 *  Created on: 22 Apr 2021
 *      Author: yehen
 */

#ifndef SRC_EEPROM_H_
#define SRC_EEPROM_H_


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

uint8_t EEPROMQueued = 0;
uint8_t EEPROMData = 0;
uint16_t EEPROMAddress = 0;





#endif /* SRC_EEPROM_H_ */
