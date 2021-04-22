/*
 * Menu.h
 *
 *  Created on: 20 Apr 2021
 *      Author: yehen
 */

#ifndef SRC_MENU_H_
#define SRC_MENU_H_

#include "User_Params.h"

/* menu hierarchy is:
 *
 * main menu
 * sub menu
 * parameter
 *
 */
#define MENU_SIZE ((sizeof(menuItems)/sizeof(menuItems[0])-1))
const char menuItems[6][15] = {"Faders", "Knobs", "Knob Buttons", "Key Matrix", "Misc", "Back"};

const char subMenu0[4][15] = {"Fader 1 CC", "Fader 2 CC", "Fader 3 CC", "Back"};
const char subMenu1[9][15] = {"Knob 1 CC", "Knob 1 Speed", "Knob 2 CC", "Knob 2 Speed", "Knob 3 CC", "Knob 3 Speed", "Knob 4 CC", "Knob 4 Speed", "Back"};
const char subMenu2[5][15] = {"Knob 1 Note", "Knob 2 Note", "Knob 3 Note", "Knob 4 Note", "Back"};
const char subMenu3[3][15] = {"First Note", "Master Velo", "Back"}; //trap for young players: we need to remember to leave one more byte for the null terminator
const char subMenu4[2][15] = {"Midi Channel", "Back"};
const char (*subMenus[5])[15] = {subMenu0, subMenu1, subMenu2, subMenu3, subMenu4};
const int8_t subMenuSizes[5] = {(sizeof(subMenu0)/sizeof(subMenu0[0]))-1, (sizeof(subMenu1)/sizeof(subMenu1[0]))-1, (sizeof(subMenu2)/sizeof(subMenu2[0]))-1, (sizeof(subMenu3)/sizeof(subMenu3[0]))-1, (sizeof(subMenu4)/sizeof(subMenu4[0]))-1}; //beware of zero indexing

int8_t *parameters0[8]= {&MidiCCFaderLUT[0], &MidiCCFaderLUT[1], &MidiCCFaderLUT[2]};
int8_t *parameters1[8] = {&MidiCCEncoderLUT[0], &EncoderSpeed[0], &MidiCCEncoderLUT[1], &EncoderSpeed[1], &MidiCCEncoderLUT[2], &EncoderSpeed[2], &MidiCCEncoderLUT[3], &EncoderSpeed[3]};
int8_t *parameters2[8] = {&EncoderNote[0], &EncoderNote[1], &EncoderNote[2], &EncoderNote[3]};
int8_t *parameters3[8] = {&MidiNoteOffset, &MidiNoteVelo};
int8_t *parameters4[8] = {&MidiChannel};
int8_t **parameters[8] = {parameters0, parameters1, parameters2, parameters3, parameters4}; //pointers to all arrays to make life easier


//lower bounds for all parameters
const int8_t parameterLB0[8] = {0, 0, 0};
const int8_t parameterLB1[8] = {0, 1, 0, 1, 0, 1, 0, 1};
const int8_t parameterLB2[8] = {0, 0, 0, 0};
const int8_t parameterLB3[8] = {0, 0};
const int8_t parameterLB4[8] = {0};
const int8_t *parameterLBs[5] = {parameterLB0, parameterLB1, parameterLB2, parameterLB3, parameterLB4};

//lower bounds for all parameters
const int8_t parameterUB0[8] = {127, 127, 127};
const int8_t parameterUB1[8] = {127, 4, 127, 4, 127, 4, 127, 4};
const int8_t parameterUB2[8] = {127, 127, 127, 127};
const int8_t parameterUB3[8] = {93, 127};
const int8_t parameterUB4[8] = {15};
const int8_t *parameterUBs[5] = {parameterUB0, parameterUB1, parameterUB2, parameterUB3, parameterUB4};

const char noteNames[12][3] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};


int8_t menuItemSelected;
int8_t subMenuSelected;
int8_t parameterSelected;


enum currentStatus {Status, Menu, SubMenu, ParaSet} status = Status; //show status page by default



#endif /* SRC_MENU_H_ */
