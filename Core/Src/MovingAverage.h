/*
 * Moving Average.h
 *
 *  Created on: Apr 25, 2021
 *      Author: yehen
 */

#ifndef SRC_MOVINGAVERAGE_H_
#define SRC_MOVINGAVERAGE_H_

#include <stdint.h>
#include <string.h>

//reference: https://gist.github.com/bmccormack/d12f4bf0c96423d03f82

#define NUM_CHANNELS 4
#define MAX_N_AVERAGES 256

extern uint8_t NAverages;

int inputBuffer[NUM_CHANNELS][MAX_N_AVERAGES];
int oldestPos[NUM_CHANNELS];
long sum[NUM_CHANNELS];

void initMovingAverage(int *arr, long *sum, int *oldestPos);
int movingAverage(int *arr, long *sum, int *oldestPos, int len, int nextNum);


#endif /* SRC_MOVINGAVERAGE_H_ */
