/*
 * MovingAverage.c
 *
 *  Created on: Apr 25, 2021
 *      Author: yehen
 */


#include "MovingAverage.h"



void initMovingAverage(int *arr, long *sum, int *oldestPos){

	memset(arr, 0, MAX_N_AVERAGES*sizeof(int)); //clear input buffer
	*sum = 0;
	*oldestPos = 0;
}




int movingAverage(int *arr, long *sum, int *oldestPos, int len, int nextNum){

	*sum = *sum - arr[*oldestPos] + nextNum; //subtract the oldest data point, and add in the newest

	arr[*oldestPos] = nextNum; //put next num in its place

	*oldestPos = (((*oldestPos) + 1) % len); //increment the pointer to the oldest position

	return (*sum/len);
}
