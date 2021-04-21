/*
 * Menu.h
 *
 *  Created on: 20 Apr 2021
 *      Author: yehen
 */

#ifndef SRC_MENU_H_
#define SRC_MENU_H_


/* menu hierarchy is:
 *
 * main menu
 * sub menu
 * parameter
 *
 */

const char menuItems[5][15] = {"Faders", "Knobs", "Knob Buttons", "Key Matrix", "Back"};
const char subMenu0[4][15] = {"Fader 1 CC", "Fader 2 CC", "Fader 3 CC", "Back"};
const char subMenu1[9][15] = {"Knob 1 Speed", "Knob 1 CC", "Knob 2 Speed", "Knob 2 CC", "Knob 3 Speed", "Knob 3 CC", "Knob 4 Speed", "Knob 4 CC", "Back"};
const char subMenu2[5][15] = {"Knob 1 Note", "Knob 2 Note", "Knob 3 Note", "Knob 4 Note", "Back"};
const char subMenu3[3][15] = {"First Note", "Master Velocity", "Back"};
const char (*subMenus[4])[15] = {subMenu0, subMenu1, subMenu2, subMenu3};
const uint8_t subMenuSizes[4] = {sizeof(subMenu0)/15, sizeof(subMenu1)/15, sizeof(subMenu2)/15, sizeof(subMenu3)/15};
const void* parameters[][] = {MidiCCFaderLut


int8_t menuItemSelected;
int8_t subMenuSelected;
int8_t parameterSelected;


enum currentStatus {Status, Menu, SubMenu, ParaSet} status = Status; //show status page by default



#endif /* SRC_MENU_H_ */
