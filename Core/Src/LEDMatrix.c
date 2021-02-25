/*
 * LEDMatrix.c
 *
 *  Created on: Feb 25, 2021
 *      Author: yehen
 */

#include "LEDMatrix.h"

uint8_t LEDMatrix[4] = {0b10101010, 0b01010101, 0b11110000, 0b00001111}; //current state of the LED Matrix per row
uint8_t LEDMatrixBuffer[12];
volatile uint8_t currentLEDRow = 0;





void LEDMatrixInit(uint8_t addr){


	//note: BTF clearing and stop generation are handled by the Event Interrupt
	__disable_irq();



	I2C1->CR1 |= (1<<8); //send start condition
	while ((I2C1->SR1 & 1) == 0); //clear SB
	I2C1->DR = addr; //address the MCP23017
	while ((I2C1->SR1 & (1<<1)) == 0); //wait for ADDR flag
	while ((I2C1->SR2 & (1<<2)) == 0); //read I2C SR2
	while ((I2C1->SR1 & (1<<7)) == 0); //make sure TxE is 1
	I2C1->DR = 0x00; //write to IODIR_A
	while ((I2C1->SR1 & (1<<7)) == 0); //make sure TxE is 1
	I2C1->DR = 0x00; //all outputs
	while ((I2C1->SR1 & (1<<7)) == 0); //make sure TxE is 1
	I2C1->DR = 0x00; //all outputs for next address which is IODIR_B
	while ((I2C1->SR1 & (1<<7)) == 0); //make sure TxE is 1
	//while ((I2C1->SR1 & (1<<2)) == 0); //make sure BTF is 1
	I2C1->CR1 |= (1<<9); //send stop condition
	__enable_irq();

}

void LEDMatrixStart(uint8_t addr){

	while(blocked); //just so nothing stupid happens


	DMA1_Channel6->CMAR = (uint32_t)LEDMatrixBuffer;
	DMA1_Channel6->CPAR = (uint32_t)&(I2C1->DR);
	DMA1_Channel6->CNDTR = 3;
	DMA1_Channel6->CCR |= (0b11<<12); //High Priority
	DMA1_Channel6->CCR |= (1<<4 | 1<<7); //set MINC and Read from Memory
	//DMA1_Channel6->CCR |= (1<<1); //enable transfer complete interrupt

	DMA1_Channel6->CCR |= 1; //activate DMA

	__disable_irq();
	I2C1->CR2 |= (1<<9); //enable event interrupts
	I2C1->CR1 |= (1<<8); //send start condition
	while ((I2C1->SR1 & 1) == 0); //clear SB
	I2C1->DR = addr; //address the MCP23017
	I2C1->CR2 |= (1<<11); //enable DMA Requests
	__enable_irq();



}
void LEDMatrixNextRow(uint8_t addr){

	blocked = 1; //avoid issues

	DMA1_Channel6->CCR &= ~1; //disable DMA1 Channel 6 for reconfiguring
	if(currentLEDRow == 3) currentLEDRow = 0;
	else currentLEDRow++;
	DMA1_Channel6->CNDTR = 3; //reload 3 bytes to transfer
	DMA1_Channel6->CMAR = (uint32_t)&(LEDMatrixBuffer[currentLEDRow*3]); //set next target
	DMA1_Channel6->CCR |= 1; //enable DMA1 Channel 6

	__disable_irq();
	TIM2->CR1 &= ~1; //disable BAM Driver
	TIM3->CR1 &= ~1;
	I2C1->CR1 |= (1<<8); //send restart condition
	while ((I2C1->SR1 & 1) == 0); //clear SB
	I2C1->DR = LEDMatrix_Address; //address the MCP23017
	I2C1->CR2 |= (1<<11); //enable DMA Requests
	TIM2->CR1 |= 1; //enable BAM Driver
	TIM3->CR1 |= 1;
	__enable_irq();

/*	__disable_irq();
	TIM2->CR1 &= ~1; //disable BAM Driver
	TIM3->CR1 &= ~1;
	I2C1->CR1 |= (1<<8); //send start condition
	while ((I2C1->SR1 & 1) == 0); //clear SB
	I2C1->DR = LEDMatrix_Address; //address the MCP23017

	I2C1->CR2 |= (1<<11); //enable DMA Requests
	if(DMA1_Channel6->CCR&1) GPIOA->BSRR = 1<<6;
	else GPIOA->BRR = 1<<6;
	TIM2->CR1 |= 1; //enable BAM Driver
	TIM3->CR1 |= 1;
	__enable_irq();*/

}
