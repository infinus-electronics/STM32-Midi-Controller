/*
 * Midi.c
 *
 *  Created on: Mar 9, 2021
 *      Author: yehen
 */

#include "usb_device.h"
#include "usbd_cdc_if.h"


void MidiCC(uint8_t channel, uint8_t cc, uint8_t val){

	uint8_t buffer[3];
	buffer[0] = 0b10110000 | channel;
	buffer[1] = cc;
	buffer[2] = val;
	while(CDC_Transmit_FS(buffer, 3) != USBD_OK); //TODO: observe if it causes hangups

}



void MidiNoteOn(uint8_t channel, uint8_t note, uint8_t velo){

	uint8_t buffer[3];
	buffer[0] = 0b10010000 | channel;
	buffer[1] = note;
	buffer[2] = velo;
	while(CDC_Transmit_FS(buffer, 3) != USBD_OK);

}


void MidiNoteOff(uint8_t channel, uint8_t note, uint8_t velo){

	uint8_t buffer[3];
	buffer[0] = 0b10000000 | channel;
	buffer[1] = note;
	buffer[2] = velo;
	while(CDC_Transmit_FS(buffer, 3) != USBD_OK);

}
