/*
 * LCD.c
 *
 *  Created on: Feb 25, 2021
 *      Author: yehen
 */

#include "LCD.h"
#include "DWT_Delay.h"


uint8_t currentIOState[2] = {0, 0}; //current state of the LCD MCP23017

volatile uint8_t updateLCD = 0; //flag that is to be set to 1 if the LCD has pending data to be updated
volatile uint8_t currentLCDSection = 0; //the LCD is going to be updated in 4 separate quadrants
uint8_t LCDBuffer[36]; //32 chars, plus 4 cursor set commands
volatile uint8_t cycleEN = 0; //does the enable pin need to be cycled in I2C2 BTF?
volatile uint8_t currentLCDByte = 0; //which byte are we at (out of 9)


/* MCP23017 Defines */

void MCP23017SetPin(uint8_t pin, bank b, uint8_t addr){

	while(blocked); //wait for clearance
	//GPIOA->BSRR = (1<<7);

	currentIOState[b] |= (1<<pin);

	//Note that all the I2C pointers are already volatile
	//I think I know the problem here, the start condition is sent, then the interrupt fires, then I2C DR has yet to be written, so the entire thing crashes in a pile of flames. It looks like the interrupt routine is plenty fast when compared to a full byte transfer, but just too long to squeeze into a start condition. YUP, CONFIRMED THAT IT GETS STUCK WAITING FOR THE ADDRESS FLAG, IE the address flag is not set!
	//write out the new state
	//UPDATE: This messes up the BAM Driver because it causes the BAM to skip entire steps... its better just to pause TIM2
	//__disable_irq(); //the entire routine will be super duper unhappy unless this is in place


	TIM2->CR1 &= ~1; //disable BAM Driver
	TIM3->CR1 &= ~1;
	//__disable_irq();

	I2C2->CR1 |= (1<<8); //send start condition
	while ((I2C2->SR1 & 1) == 0); //clear SB
	I2C2->DR = addr; //address the MCP23017
	//__enable_irq(); didn't work here
	while ((I2C2->SR1 & (1<<1)) == 0); //wait for ADDR flag
	while ((I2C2->SR2 & (1<<2)) == 0); //read I2C SR2

	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	if(b==A){
		I2C2->DR = 0x14;
	}
	else{
		I2C2->DR = 0x15;
	}
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	I2C2->DR = currentIOState[b]; //just pull everything low
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	//while ((I2C2->SR1 & (1<<2)) == 0); //make sure BTF is 1
	I2C2->CR1 |= (1<<9); //send stop condition

	while ((I2C2->SR2 & (1<<1)) == 1); //make damn sure the I2C bus is free

	//__enable_irq();
	TIM2->CR1 |= 1; //enable BAM Driver
	TIM3->CR1 |= 1;

	//GPIOA->BRR = (1<<7);

}

void MCP23017ClearPin(uint8_t pin, bank b, uint8_t addr){

	while(blocked); //wait for clearance
	//GPIOA->BSRR = (1<<7);

	currentIOState[b] &= ~(1<<pin);
	//Note that all the I2C pointers are already volatile
	//I think I know the problem here, the start condition is sent, then the interrupt fires, then I2C DR has yet to be written, so the entire thing crashes in a pile of flames. It looks like the interrupt routine is plenty fast when compared to a full byte transfer, but just too long to squeeze into a start condition. YUP, CONFIRMED THAT IT GETS STUCK WAITING FOR THE ADDRESS FLAG, IE the address flag is not set!
	//write out the new state
	//UPDATE: This messses up the BAM Driver... I think it'll be better just to stop TIM2
	//__disable_irq(); //the entire routine will be super duper unhappy unless this is in place

	//potential issue: the other interrupts may cause this crap to fail again...

	TIM2->CR1 &= ~1; //disable BAM Driver
	TIM3->CR1 &= ~1;
	//__disable_irq();

	I2C2->CR1 |= (1<<8); //send start condition
	while ((I2C2->SR1 & 1) == 0); //clear SB
	I2C2->DR = addr; //address the MCP23017
	//__enable_irq(); didn't work here
	while ((I2C2->SR1 & (1<<1)) == 0); //wait for ADDR flag
	while ((I2C2->SR2 & (1<<2)) == 0); //read I2C SR2
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	if(b==A){
		I2C2->DR = 0x14;
	}
	else{
		I2C2->DR = 0x15;
	}
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	I2C2->DR = currentIOState[b]; //just pull everything low
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	//while ((I2C2->SR1 & (1<<2)) == 0); //make sure BTF is 1
	I2C2->CR1 |= (1<<9); //send stop condition
	while ((I2C2->SR2 & (1<<1)) == 1); //make damn sure the I2C bus is free

	//__enable_irq();
	TIM2->CR1 |= 1; //enable BAM Driver
	TIM3->CR1 |= 1;
	//__enable_irq();
	//GPIOA->BRR = (1<<7);

}

/* LCD Defines */

/**
 * LCD Pinout:
 * A0-A7 : D0-D7
 * B2: RS
 * B1: RW
 * B0: E
 */

#define RS_Pin 2
#define RW_Pin 1
#define EN_Pin 0

/**
 * \fn LCDInit
 * @brief Initialises both the LCD and the MCP23017
 *
 * @param addr Address of the MCP23017
 */
void LCDInit(uint8_t addr){ //interrupts should be disabled here

	//while(blocked); //wait for clearance anyways just for good measure

	//Initialise the MCP23017 first
	__disable_irq(); //let's allow the init to go down peacefully
	I2C2->CR1 |= (1<<8); //send start condition
	while ((I2C2->SR1 & 1) == 0); //clear SB
	I2C2->DR = addr; //address the MCP23017
	while ((I2C2->SR1 & (1<<1)) == 0); //wait for ADDR flag
	while ((I2C2->SR2 & (1<<2)) == 0); //read I2C SR2
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	I2C2->DR = 0x00; //write to IODIR_A
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	I2C2->DR = 0x00; //all outputs
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	I2C2->DR = 0x00; //all outputs for next address which is IODIR_B
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	//while ((I2C2->SR1 & (1<<2)) == 0); //make sure BTF is 1
	I2C2->CR1 |= (1<<9); //send stop condition


	//Pull RS, RW and E pins LOW
	MCP23017ClearPin(RS_Pin, B, LCD_Address);
	MCP23017ClearPin(RW_Pin, B, LCD_Address);
	GPIOA->BRR = 1<<8;



	LCDData(0x00, addr); //clear the data pins as well
	DWT_Delay_ms(30);

	LCDCommand(0x30, addr); //function set
	DWT_Delay_ms(5);

	LCDCommand(0x30, addr); //function set
	DWT_Delay_ms(5);

	LCDCommand(0x30, addr); //function set
	DWT_Delay_us(1000);

	LCDCommand(0x38, addr); //8-bit mode, 2 lines, smaller font

	LCDCommand(0x0C, addr); //display ON

	LCDCommand(0x01, addr); //display clear
	DWT_Delay_us(2000); //clear requires a substantial delay

	LCDCommand(0x06, addr); //set entry mode

	__enable_irq();


}

/**
 * \fn LCDData
 * @brief Presents the data to D0 to D7 (located on Bank A)
 *
 * @param data Data to send
 * @param addr I2C Address of the MCP23017
 */
void LCDData(char data, uint8_t addr){

	while(blocked); //wait for clearance

	TIM2->CR1 &= ~1; //disable BAM Driver
	TIM3->CR1 &= ~1;
	//__disable_irq();

	I2C2->CR1 |= (1<<8); //send start condition
	while ((I2C2->SR1 & 1) == 0); //clear SB
	I2C2->DR = addr; //address the MCP23017
	while ((I2C2->SR1 & (1<<1)) == 0); //wait for ADDR flag
	while ((I2C2->SR2 & (1<<2)) == 0); //read I2C SR2
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	I2C2->DR = 0x14; //write to GPIO_A
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	I2C2->DR = data; //present data at output bank A
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	while ((I2C2->SR1 & (1<<2)) == 0); //make sure BTF is 1
	I2C2->CR1 |= (1<<9); //send stop condition

	//__enable_irq();
	TIM2->CR1 |= 1; //enable BAM Driver
	TIM3->CR1 |= 1;

}

void LCDCommand(char data, uint8_t addr){


	MCP23017ClearPin(RS_Pin, B, addr);

	LCDData(data, addr);

	LCDCycleEN(addr);

	MCP23017SetPin(RS_Pin, B, addr);

}

void LCDCycleEN(uint8_t addr){

/*
 * MCP23017ClearPin(EN_Pin, B, addr);
	DWT_Delay_us(1);
	MCP23017SetPin(EN_Pin, B, addr);
	DWT_Delay_us(1);
	MCP23017ClearPin(EN_Pin, B, addr);
	DWT_Delay_us(100);

*/
	GPIOA->BRR = 1<<8;
	GPIOA->BSRR = 1<<8; //this pulse is 100ns, aka too short, datasheet specifies min of 230 ns
	GPIOA->BSRR = 1<<8;
	GPIOA->BSRR = 1<<8;
	GPIOA->BRR = 1<<8;

}

void LCDWriteChar(char data, uint8_t addr){

	//MCP23017SetPin(RS_Pin, B, addr);
	//I2C2->CR2 |= (1<<9); //enable event interrupts
	LCDData(data, addr);
	LCDCycleEN(addr);

}

void LCDWriteString(char *str, uint8_t addr){

	for(int i = 0; (volatile char)str[i] != '\x00' ; i++){ //Nice touch: take advantage of null byte terminated strings
		LCDWriteChar(str[i], addr);
	}

}

void LCDClear(uint8_t addr){

	LCDCommand(1, addr);
	DWT_Delay_us(2000);

}

void LCDSetCursor(uint8_t row, uint8_t col, uint8_t addr){

	char outbyte;

	if(row == 1){
		outbyte = 0x80 + col - 1;
		LCDCommand(outbyte, addr);
	}
	else if(row == 2){
		outbyte = 0xC0 + col - 1;
		LCDCommand(outbyte, addr);
	}

}

void LCDShiftLeft(uint8_t addr){

	LCDCommand(0x18, addr);


}

void LCDShiftRight(uint8_t addr){

	LCDCommand(0x1C, addr);


}

/*
 * \fn LCDPrepareInt
 *
 * @brief this function sets up the MCP23017 so that it can take on the interrupt based auto LCD updating routine
 */
//TODO: might want to convert this to DMA driven code
void LCDPrepareInt(){

	//initialise the buffer for the DMA by inserting all the requisite stuff
	LCDBuffer[0] = 0x80; //line 1 col 1
	LCDBuffer[9] = 0x88; //line 1 col 9
	LCDBuffer[18] = 0xC0; //line 2 col 1
	LCDBuffer[27] = 0xC8; //line 2 col 2

	while(blocked); //wait for clearance

	TIM2->CR1 &= ~1; //disable BAM Driver
	TIM3->CR1 &= ~1;
	__disable_irq();

	I2C2->CR1 |= (1<<8); //send start condition
	while ((I2C2->SR1 & 1) == 0); //clear SB
	I2C2->DR = LCD_Address; //address the MCP23017
	while ((I2C2->SR1 & (1<<1)) == 0); //wait for ADDR flag
	while ((I2C2->SR2 & (1<<2)) == 0); //read I2C SR2
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	I2C2->DR = 0x0A; //write to IOCON
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	I2C2->DR = ((1<<5)|(1<<7)); //disable address incrementation and set bank mode to 1 to prevent the sneaky alternate bank switching shenanigans
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	while ((I2C2->SR1 & (1<<2)) == 0); //make sure BTF is 1
	I2C2->CR1 |= (1<<9); //send stop condition

	__enable_irq();
	TIM2->CR1 |= 1; //enable BAM Driver
	TIM3->CR1 |= 1;

	I2C1->CR2 |= 1<<9; //enable I2C2 event Interrupts

	//prepare DMA1 Channel 4

	DMA1_Channel4->CMAR = (uint32_t)LCDBuffer;
	DMA1_Channel4->CPAR = (uint32_t)&(I2C2->DR);
	DMA1_Channel4->CNDTR = 1; //this is just for the sake of having to wait for all the I2C data to be shifted out
	DMA1_Channel4->CCR |= (0b10<<12); //High Priority
	DMA1_Channel4->CCR |= (1<<4 | 1<<7); //set MINC and Read from Memory

	DMA1_Channel4->CCR |= 1; //activate DMA

}

/*
 * \fn LCDWriteStringInt
 * call this in the TIM 2 ISR
 * update the variable currentLCDSection before calling this function
 * @brief This function sets up an interrupt based transfer routine, which is to be driven by TIM 2 every BAM cycle to update a quarter of the LCD
 */
//this code must be DMA driven since its called inside the TIM 2 interrupt, which is enough of a mess already
void LCDWriteStringInt(uint8_t section){

	//TODO: code now worry later - implement the line setting code, maybe just set it to run nevertheless for more straightforward timing control
	//TODO: Set dma to transfer only 1 byte; we are using the DMA to avoid tying up the cpu only, coz we need the BTF signal...

	currentLCDByte = 0; //reset the byte counter

	__disable_irq(); //this part is set to run after each LED Matrix next row, so I think we will not face any lockups

	DMA1_Channel4->CCR &= ~1; //disable DMA1 Channel 4 for reconfiguring

	DMA1_Channel4->CNDTR = 1; //reload
	DMA1_Channel4->CMAR = (uint32_t)&(LCDBuffer[section*9]); //set target

	DMA1_Channel4->CCR |= 1; //enable DMA1 Channel 4


	//clear RS
	cycleEN = 1; //indicate that the subsequent I2C byte transfers should be followed by cycling EN

	__disable_irq();

	I2C2->CR1 |= (1<<8); //send start condition
	while ((I2C2->SR1 & 1) == 0); //clear SB
	I2C2->DR = LCD_Address; //address the LCD MCP23017
	I2C2->CR2 |= (1<<11); //enable DMA Requests
	__enable_irq();


}

/*
 * \fn LCDPrintString
 *
 * Interrupt driven auto printer thing
 *
 * IMPORTANT: always pass a string 16 characters long into this function, any extra will get truncated at best, potential buffer ovf (CTF brain engaged)
 */
void LCDPrintString(char* str, uint8_t line){

	//this function basically copies all the chars into the
	if(line == 1){

		for(int i = 0; i < 8; i++){
			LCDBuffer[i+1] = str[i];
		}
		for(int i = 0; i < 8; i++){
			LCDBuffer[i+10] = str[i];
		}

	}

	else if(line == 2){
		for(int i = 0; i < 8; i++){
			LCDBuffer[i+19] = str[i];
		}
		for(int i = 0; i < 8; i++){
			LCDBuffer[i+28] = str[i];
		}
	}

	updateLCD = 1; //signal the interrupt routine that an LCD update is pending
}

