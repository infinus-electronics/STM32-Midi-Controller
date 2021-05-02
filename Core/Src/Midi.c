/*
 * Midi.c
 *
 *  Created on: Mar 9, 2021
 *      Author: yehen
 */

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "Midi.h"

uint8_t MidiDataQueue[128];
int inputMidiBufferPosition = 0;
int outputMidiBufferPosition = 0;

extern USBD_HandleTypeDef hUsbDeviceFS;


void MidiTransmit(uint8_t data[], int numBytes){ //function to chuck the data into the FIFO

	for(int i = 0; i < numBytes; i++){

		MidiDataQueue[inputMidiBufferPosition] = data[i];
		inputMidiBufferPosition = ((inputMidiBufferPosition+1) % (sizeof(MidiDataQueue)/sizeof(MidiDataQueue[0]))); //increment the input pointer as we go

	}

}


void MidiCC(uint8_t channel, uint8_t cc, uint8_t val){

	uint8_t buffer[3];
	buffer[0] = 0b10110000 | channel;
	buffer[1] = cc;
	buffer[2] = val;
	//while(CDC_Transmit_FS(buffer, 3) == USBD_BUSY); //TODO: observe if it causes hangups
	//CDC_Transmit_FS(buffer, 3);
	MidiTransmit(buffer, 3);

}



void MidiNoteOn(uint8_t channel, uint8_t note, uint8_t velo){

	uint8_t buffer[3];
	buffer[0] = 0b10010000 | channel;
	buffer[1] = note;
	buffer[2] = velo;
	//while(CDC_Transmit_FS(buffer, 3) == USBD_BUSY);
	MidiTransmit(buffer, 3);

}


void MidiNoteOff(uint8_t channel, uint8_t note, uint8_t velo){

	uint8_t buffer[3];
	buffer[0] = 0b10000000 | channel;
	buffer[1] = note;
	buffer[2] = velo;
	MidiTransmit(buffer, 3);

}

void flushMidi(){

	uint8_t buf[128];
	int j = 0;

	if(((((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState) == 0) && (outputMidiBufferPosition != inputMidiBufferPosition)){ //USB is not busy AND we have data to transmit

		/*if(inputMidiBufferPosition > outputMidiBufferPosition){
			int numBytes = inputMidiBufferPosition - outMidiBufferPosition;
		}
		else{
			int numBytes = (sizeof(MidiDataQueue)/sizeof(MidiDataQueue[0]))-(outputMidiBufferPosition-inputMidiBufferPosition);
		}*/

		for(; outputMidiBufferPosition != inputMidiBufferPosition; outputMidiBufferPosition = ((outputMidiBufferPosition+1) % (sizeof(MidiDataQueue)/sizeof(MidiDataQueue[0])))){
			buf[j] = MidiDataQueue[outputMidiBufferPosition];
			j++;
		}

		CDC_Transmit_FS(buf, j);

	}
}
