/*
 * Midi.h
 *
 *  Created on: Mar 9, 2021
 *      Author: yehen
 */

#ifndef SRC_MIDI_H_
#define SRC_MIDI_H_

void MidiCC(uint8_t channel, uint8_t cc, uint8_t val);
void MidiNoteOn(uint8_t channel, uint8_t note, uint8_t velo);
void MidiNoteOff(uint8_t channel, uint8_t note, uint8_t velo);



#endif /* SRC_MIDI_H_ */
