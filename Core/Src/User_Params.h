/*
 * User_Params.h
 *
 *  Created on: 21 Apr 2021
 *      Author: yehen
 */

#ifndef SRC_USER_PARAMS_H_
#define SRC_USER_PARAMS_H_


uint8_t MidiNoteLUT[20]; //which key maps to which note
uint8_t MidiNoteOffset = 60; //which key does the bottom left button map to?
uint8_t MidiNoteVelo = 127;
uint8_t MidiChannel = 0;
uint8_t MidiCCFaderLUT[3] = {1, 7, 10}; //which fader maps to which CC
uint8_t MidiCCEncoderLUT[4] = {1, 7, 10, 11};
uint8_t EncoderSpeed[4] = {1, 1, 1, 1};
uint8_t EncoderNote[4] = {60, 60, 60, 60};


#endif /* SRC_USER_PARAMS_H_ */
