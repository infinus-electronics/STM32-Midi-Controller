/*
 * User_Params.h
 *
 *  Created on: 21 Apr 2021
 *      Author: yehen
 */

#ifndef SRC_USER_PARAMS_H_
#define SRC_USER_PARAMS_H_

#define MAX_ENCODER_SPEED 6
int8_t MidiNoteLUT[20]; //which key maps to which note
int8_t MidiNoteOffset = 60; //which key does the bottom left button map to?
int8_t MidiNoteVelo = 127;
int8_t MidiChannel = 0;
int8_t MidiCCFaderLUT[3] = {1, 7, 10}; //which fader maps to which CC
int8_t MidiCCEncoderLUT[4] = {1, 7, 10, 11};
int8_t EncoderSpeed[4] = {1, 1, 1, 1};
int8_t EncoderNote[4] = {60, 60, 60, 60};

#define MAX_FILTER_BETA 16
int8_t filterBeta = 7;

int8_t NAverages = 16;


#endif /* SRC_USER_PARAMS_H_ */
