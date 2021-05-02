/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include "DWT_Delay.h"
#include "LCD.h"
#include "LEDMatrix.h"
#include "Midi.h"
#include "ADC.h"
#include "User_Params.h"
#include "Menu.h"
#include "EEPROM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_tx;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile uint8_t brightness[4] = {1,1,1,1}; //initialize array holding brightness values for the 4 indicator LED's
volatile uint8_t BAMIndex = 0;
volatile uint8_t blocked = 0; //time critical BAM code running, do not disable interrupts


volatile uint8_t currentEncoder = 0; //which encoder are we polling right now?
volatile uint8_t lastEncoder[5] = {0,0,0,0,0};//initialize array containing past encoder readoff
volatile int encoderValues[5] = {0,0,0,0,0};//initialize array containing encoder values
volatile int8_t encoderLUT[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
int lastEncoderValues[5] = {0, 0, 0, 0, 0};

uint8_t lastButtonState = 0; //menu encoder's button




uint32_t lastKeyMatrix = 0; //last state of the key matrix
uint32_t currentKeyMatrix = 0; //current state of the key matrix

int8_t MidiCCValues[128];

int adcSmooth[3]; //adc data after its passed thru the IIR filter
int lastFaderValues[3] = {0, 0, 0};



char LCDQueueTop[17];
char LCDQueueBottom[17];
uint8_t LCDTopQueued; //does the LCD have to be updated this round?
uint8_t LCDBottomQueued;



#define MAXIMUM  15
int8_t dIntegrator = 0;
int8_t dOutput = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void debug(){
	GPIOA->BSRR = (1<<7);
	GPIOA->BRR = (1<<7);
}

void clampedIncrement(int8_t* n, int8_t inc, int8_t lo, int8_t hi){

	int a = *n;
	int temp = a + inc;

	if(temp < lo){
		*n = lo;
	}
	else if(temp > hi){
		*n = hi;
	}
	else{
		*n = (int8_t)temp;
	}

}

/*
//screw unsigned types....sigh...this crap wasted hours of my life
void uclampedIncrement(uint8_t* n, int8_t inc, uint8_t lo, uint8_t hi){

	int temp = (int)*n + (int)inc;

	if(temp < 0){
		brightness[0] += 5;
		*n = 0;
	}
	else if(temp > hi){
		*n = hi;
	}
	else{
		*n = (uint8_t)temp;
	}

}
*/


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  NVIC_SetPriorityGrouping(0U); //use standard interrupt grouping

  //init stuff
  DWT_Delay_Init();

/*
  //write in all user parameters to the EEPROM
  I2C2->CR1 |= (1<<8); //send start condition
  while ((I2C2->SR1 & 1) == 0); //clear SB
  I2C2->DR = 0xA0; //address the EEPROM
  while ((I2C2->SR1 & (1<<1)) == 0); //wait for ADDR flag
  while ((I2C2->SR2 & (1<<2)) == 0); //read I2C SR2
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = 0x0; //write to GPIO_A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = 0x0; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = MidiNoteOffset; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = MidiNoteVelo; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = MidiChannel; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = MidiCCFaderLUT[0]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = MidiCCFaderLUT[1]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = MidiCCFaderLUT[2]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = MidiCCEncoderLUT[0]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = MidiCCEncoderLUT[1]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = MidiCCEncoderLUT[2]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = MidiCCEncoderLUT[3]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = EncoderSpeed[0]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = EncoderSpeed[1]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = EncoderSpeed[2]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = EncoderSpeed[3]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = EncoderNote[0]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = EncoderNote[1]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = EncoderNote[2]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = EncoderNote[3]; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = filterBeta; //present data at output bank A
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  while ((I2C2->SR1 & (1<<2)) == 0); //make sure BTF is 1
  I2C2->CR1 |= (1<<9); //send stop condition

  DWT_Delay_ms(500);
  */

  //read in all user parameters from the EEPROM
  I2C2->CR1 |= (1<<8); //send start condition

  while ((I2C2->SR1 & 1) == 0); //clear SB
  I2C2->DR = 0xA0; //address the EEPROM in write mode
  while ((I2C2->SR1 & (1<<1)) == 0); //wait for ADDR flag
  while ((I2C2->SR2 & (1<<2)) == 0); //read I2C SR2
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = 0x0; //address
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  I2C2->DR = 0x0; //address
  while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  while ((I2C2->SR1 & (1<<2)) == 0); //make sure BTF is 1
  I2C2->CR1 |= 1<<10;
  I2C2->CR1 |= (1<<8); //send start condition
  while ((I2C2->SR1 & 1) == 0); //clear SB
  I2C2->DR = 0xA1; //address the EEPROM in read mode
  while ((I2C2->SR1 & (1<<1)) == 0); //wait for ADDR flag
  while ((I2C2->SR2) & 0); //read I2C SR2
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  MidiNoteOffset = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  MidiNoteVelo = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  MidiChannel = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  MidiCCFaderLUT[0] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  MidiCCFaderLUT[1] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  MidiCCFaderLUT[2] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  MidiCCEncoderLUT[0] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  MidiCCEncoderLUT[1] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  MidiCCEncoderLUT[2] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  MidiCCEncoderLUT[3] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  EncoderSpeed[0] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  EncoderSpeed[1] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  EncoderSpeed[2] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  EncoderSpeed[3] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  EncoderNote[0] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  EncoderNote[1] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  EncoderNote[2] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  EncoderNote[3] = (I2C2->DR) & 0xff;
  while ((I2C2->SR1 & 1<<6) == 0); //wait for RXNE
  I2C2->CR1 &= ~(1<<10); //NACK
  I2C2->CR1 |= 1<<9; //STOP
  filterBeta = I2C2->DR;




  DWT_Delay_ms(50); //let stuff settle down properly

  IWDG->KR = 0xAAAA; //reset the watchdog timer
  blocked = 0;
  I2C2->CR1 |= 1; //enable i2c 2 peripheral for LCD and EEPROM
  I2C1->CR1 |= 1; //enable i2c 1 peripheral for LED Matrix

  LCDInit(LCD_Address);
  LEDMatrixInit(LEDMatrix_Address);

  LCDClear(LCD_Address);

  LCDSetCursor(1, 1, LCD_Address);

  LCDPrepareInt();


  TIM2->CR1 |= 1; //enable BAM Driver
  TIM3->CR1 |= 1; //enable encoder scan driver


  //indicate where are all the note C's
  for(int8_t i = 0; i < 4; i++){
	  LEDMatrix[i] &= 0xf; //clear all the red channels
  }

  for(int8_t i = 0; i < 16; i++){ //we have 16 keys

	  int8_t row = 3-(i>>2);
	  int8_t col = (i%4);
	  if((MidiNoteOffset+i)%12 == 0){ //this key is a C
		  LEDMatrix[row] |= (1<<(4+col));
	  }

  }



  for(int i = 0; i < 4; i++){ //function to drive the LED's
	  LEDMatrixBuffer[i*4] = 0b1111; //clear all pins first to prevent ghosting
	  LEDMatrixBuffer[i*4+1] = 0x00;
	  LEDMatrixBuffer[i*4+2] = ~(1<<i);
	  LEDMatrixBuffer[i*4+3] = LEDMatrix[i];
  }

  LEDMatrixStart(LEDMatrix_Address);




  for(int i = 0; i < 4; i++){ //function to fill in the MidiNoteLut

	  for(int j = 0; j < 4; j++){

		  MidiNoteLUT[5*(3-i)+j+1] = MidiNoteOffset + (4*i+j); //math...

	  }

  }

  for(int i = 1; i < 4; i++){

	  int currentADC = ADC1ReadVal(i);
	  currentADC += ADC1ReadVal(i);
	  currentADC += ADC1ReadVal(i);
	  currentADC = currentADC/3;

	  lastFaderValues[i-1] = currentADC >> 5;


  }

  for(int i = 0; i < 5; i++){

	  lastEncoderValues[i] = encoderValues[i];

  }

  //memset(MidiCCValues, 0, sizeof(MidiCCValues));


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  IWDG->KR = 0xAAAA; //reset the watchdog timer
	  flushMidi();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  brightness[0] = MidiCCValues[MidiCCEncoderLUT[3]] << 1;
	  brightness[1] = MidiCCValues[MidiCCEncoderLUT[2]] << 1;
	  brightness[2] = MidiCCValues[MidiCCEncoderLUT[1]] << 1;
	  brightness[3] = MidiCCValues[MidiCCEncoderLUT[0]] << 1;


	 uint8_t currentButtonState = ((GPIOB->IDR)&1);


	 /*
	  *
	  * debounce.c
		written by Kenneth A. Kuhn
		version 1.00
	  */

	 /* Step 1: Update the integrator based on the input signal.  Note that the
	 integrator follows the input, decreasing or increasing towards the limits as
	 determined by the input state (0 or 1). */

	 if (currentButtonState == 0){ //button is currently depressed
		 if(dIntegrator < MAXIMUM){
			 dIntegrator++;
		 }
	 }
	 else if(dIntegrator > 0){ //button is not depressed
		 dIntegrator--;
	 }

	 /* Step 2: Update the output state based on the integrator.  Note that the
	 output will only change states if the integrator has reached a limit, either

	 0 or MAXIMUM. */

	 if(dIntegrator == 0){
		 dOutput = 0;
	 }

	 else if(dIntegrator >= MAXIMUM){
		 dOutput = 1;
		 dIntegrator = MAXIMUM; /* defensive code if integrator got corrupted */
	 }


	 /*begin button handler*/
	 if (dOutput == 1 && lastButtonState == 0){ //button has been pressed


		 switch (status){

		 case Status:
			 //we are now entering menu mode
			 status = Menu;
			 menuItemSelected = 0; //start from the first item
			 snprintf(LCDQueueTop, 17, "\x7E%s", menuItems[menuItemSelected]);
			 snprintf(LCDQueueBottom, 17, " %s", menuItems[menuItemSelected+1]); //normally you'd have to check if there is a next item available, but since this is only the menu init, we don't have to. (We will have to in the encoder-rotated handler)
			 LCDTopQueued = 1; //signal that we need to update the LCD
			 LCDBottomQueued = 1;
			 break;

		 case Menu:
			 //we are already in our menu, time to enter whatever submenu is selected (or exit)
			 if(menuItemSelected != MENU_SIZE){ //we have not selected "back", go into the selected SubMenu;
				 status = SubMenu;
				 subMenuSelected = menuItemSelected;
				 parameterSelected = 0; //start afresh
				 snprintf(LCDQueueTop, 17, "\x7E%s", (*(subMenus[subMenuSelected]+parameterSelected)));
				 snprintf(LCDQueueBottom, 17, " %s", (*(subMenus[subMenuSelected]+parameterSelected+1)));
				 LCDTopQueued = 1; //signal that we need to update the LCD
				 LCDBottomQueued = 1;

			 }

			 else{ //exit menu back into status
				 status = Status;
				 snprintf(LCDQueueTop, 17, "                "); //clear LCD
				 snprintf(LCDQueueBottom, 17, "                ");
				 LCDTopQueued = 1; //signal that we need to update the LCD
				 LCDBottomQueued = 1;
			 }
			 break;

		 case SubMenu:

			 if(parameterSelected != subMenuSizes[subMenuSelected]){//we have not selected "back", go into selected parameter page
				 status = ParaSet;
				 snprintf(LCDQueueTop, 17, "%s", (*(subMenus[subMenuSelected]+parameterSelected))); //print the current parameter on the top line
				 if((subMenuSelected == 2) | (subMenuSelected == 3 && parameterSelected == 0)){ //if we are changing notenames
					 snprintf(LCDQueueBottom, 17, "%s%d", noteNames[(*(*(parameters[subMenuSelected]+parameterSelected))) % 12], (int8_t)(((*(*(parameters[subMenuSelected]+parameterSelected))) / 12) - 1)); //print the NOTENAME of the parameter under question
				 }
				 else{
					 snprintf(LCDQueueBottom, 17, "%d", (*(*(parameters[subMenuSelected]+parameterSelected)))); //print the current value of the parameter under question
				 }
				 LCDTopQueued = 1; //signal that we need to update the LCD
				 LCDBottomQueued = 1;
			 }

			 else{ //exit sub menu back into main menu
				 status = Menu;
				 snprintf(LCDQueueTop, 17, "\x7E%s", menuItems[menuItemSelected]);
				 snprintf(LCDQueueBottom, 17, " %s", menuItems[menuItemSelected+1]); //normally you'd have to check if there is a next item available, but since this is only the menu init, we don't have to. (We will have to in the encoder-rotated handler)
				 LCDTopQueued = 1; //signal that we need to update the LCD
				 LCDBottomQueued = 1;
			 }
			 break;

		 case ParaSet: //save parameter and return to the SubMenu

			 status = SubMenu;

			 EEPROMWriteParameter(*(parameterEAddrs[subMenuSelected]+parameterSelected), *(*(parameters[subMenuSelected]+parameterSelected)));


			 snprintf(LCDQueueTop, 17, "\x7E%s", (*(subMenus[subMenuSelected]+parameterSelected)));
			 snprintf(LCDQueueBottom, 17, " %s", (*(subMenus[subMenuSelected]+parameterSelected+1)));
			 LCDTopQueued = 1; //signal that we need to update the LCD
			 LCDBottomQueued = 1;


			 break;

		 default:
			 break;



		 }


	 }
	 lastButtonState = dOutput;
	 /* end button handler */



	 /* begin control encoder rotated handler */
	 if(((encoderValues[4] - lastEncoderValues[4]) >= 2) | ((lastEncoderValues[4] - encoderValues[4]) >= 2)){ //control encoder has been rotated

		 int8_t increment = encoderValues[4]>lastEncoderValues[4] ? 1 : -1; //this control encoder is 2 counts per indent

		 switch (status){

		 case Menu:

			 clampedIncrement(&menuItemSelected, increment, 0, MENU_SIZE);
			 if(increment > 0 && menuItemSelected != 0){ //we advance in the menu, pointer should be in second row
				 snprintf(LCDQueueTop, 17, " %s", menuItems[menuItemSelected-1]);
				 snprintf(LCDQueueBottom, 17, "\x7E%s", menuItems[menuItemSelected]);
			 }
			 else if(menuItemSelected != MENU_SIZE){
				 snprintf(LCDQueueTop, 17, "\x7E%s", menuItems[menuItemSelected]);
				 snprintf(LCDQueueBottom, 17, " %s", menuItems[menuItemSelected+1]);
			 }
			 LCDTopQueued = 1; //signal that we need to update the LCD
			 LCDBottomQueued = 1;
			 break;

		 case SubMenu:

			 clampedIncrement(&parameterSelected, increment, 0, subMenuSizes[subMenuSelected]);
			 if(increment > 0 && parameterSelected != 0){ //we advance in the menu, pointer should be in second row
				 snprintf(LCDQueueTop, 17, " %s", (*(subMenus[subMenuSelected]+parameterSelected-1)));
				 snprintf(LCDQueueBottom, 17, "\x7E%s", (*(subMenus[subMenuSelected]+parameterSelected)));
			 }
			 else if(parameterSelected != subMenuSizes[subMenuSelected]){
				 snprintf(LCDQueueTop, 17, "\x7E%s", (*(subMenus[subMenuSelected]+parameterSelected)));
				 snprintf(LCDQueueBottom, 17, " %s", (*(subMenus[subMenuSelected]+parameterSelected+1)));
			 }
			 LCDTopQueued = 1; //signal that we need to update the LCD
			 LCDBottomQueued = 1;
			 break;

		 case ParaSet:

			 clampedIncrement((*(parameters[subMenuSelected]+parameterSelected)), increment, (*(parameterLBs[subMenuSelected]+parameterSelected)), (*(parameterUBs[subMenuSelected]+parameterSelected)));

			 //if we are changing notenames
			 if((subMenuSelected == 2) | (subMenuSelected == 3 && parameterSelected == 0)){
				 snprintf(LCDQueueBottom, 17, "%s%d", noteNames[(*(*(parameters[subMenuSelected]+parameterSelected))) % 12], (int8_t)(((*(*(parameters[subMenuSelected]+parameterSelected))) / 12) - 1)); //print the NOTENAME of the parameter under question
			 }

			 //update red LED's to indicate positions of C
			 if(subMenuSelected == 3 && parameterSelected == 0){

				 for(int8_t i = 0; i < 4; i++){
					 LEDMatrix[i] &= 0xf; //clear all the red channels
				 }

				 for(int8_t i = 0; i < 16; i++){ //we have 16 keys

					 int8_t row = 3-(i>>2);
					 int8_t col = (i%4);
					 if((MidiNoteOffset+i)%12 == 0){ //this key is a C
						 LEDMatrix[row] |= (1<<(4+col));
					 }

				 }
				 for(int i = 0; i < 4; i++){ //function to drive the LED's

					 LEDMatrixBuffer[i*4] = 0b1111; //clear all pins first to prevent ghosting
					 LEDMatrixBuffer[i*4+1] = 0x00;
					 LEDMatrixBuffer[i*4+2] = ~(1<<i);
					 LEDMatrixBuffer[i*4+3] = LEDMatrix[i];

				 }
			 }

			 //nothing special
			 else{
				 //snprintf(LCDQueueBottom, 17, "%d", *(parameterLBs[subMenuSelected]+parameterSelected));
				 snprintf(LCDQueueBottom, 17, "%d", (*(*(parameters[subMenuSelected]+parameterSelected))));//print the current value of the parameter under question
			 }
			 LCDBottomQueued = 1;
			 break;

		 default:
			 break;




		 }


		 lastEncoderValues[4] = encoderValues[4];
	 }
	 /* end control encoder rotated handler */



	 /*begin CC encoder handler*/
	  for(int i = 0; i < 4; i++){ //send encoder CC Values

		  if(encoderValues[i] != lastEncoderValues[i]){ //encoder was rotated

			  int8_t increment = encoderValues[i] > lastEncoderValues[i] ? 1 : -1;
			  clampedIncrement(&MidiCCValues[MidiCCEncoderLUT[i]], increment*EncoderSpeed[i], 0, 127);
			  MidiCC(MidiChannel, MidiCCEncoderLUT[i], MidiCCValues[MidiCCEncoderLUT[i]]);

			  if(status == Status){
				  snprintf(LCDQueueTop, 17, "CC %d", MidiCCEncoderLUT[i]);
				  LCDTopQueued = 1;
				  snprintf(LCDQueueBottom, 17, "%-16d", MidiCCValues[MidiCCEncoderLUT[i]]);
				  LCDBottomQueued = 1;
			  }

			  lastEncoderValues[i] = encoderValues[i];

		  }

	  }
	  /*end CC encoder handler*/

	  /*begin ADC handler*/
	  for(int j = 1; j < 4; j++){

		  int i = j-1;
		  int currentADC = ADC1ReadVal(j);
		  //IIR filter https://kiritchatterjee.wordpress.com/2014/11/10/a-simple-digital-low-pass-filter-in-c/
		  //but we are working in 12 bit fixed point anyways
		  adcSmooth[i] = (adcSmooth[i] << filterBeta) - adcSmooth[i];
		  adcSmooth[i] += currentADC;
		  adcSmooth[i] = adcSmooth[i] >> filterBeta;

		  currentADC = adcSmooth[i] >> 5; //convert the filter output to 7 bit, and store it currentADC
		  //Note: lastFaderValues[i] is 7 bit
		  //Note: this is NOT division for signed ints, due to the sign bit in front


		  if(lastFaderValues[i] != currentADC){ // this particular ADC Channel has been updated

			  MidiCCValues[MidiCCFaderLUT[i]] = currentADC & 0x7f; //mask off only last 7 bits

			  MidiCC(MidiChannel, MidiCCFaderLUT[i], MidiCCValues[MidiCCFaderLUT[i]]);

			  if(status == Status){
				  snprintf(LCDQueueTop, 17, "CC %d", MidiCCFaderLUT[i]);
				  LCDTopQueued = 1;
				  snprintf(LCDQueueBottom, 17, "%-16d", MidiCCValues[MidiCCFaderLUT[i]]);
				  LCDBottomQueued = 1;
			  }


			  lastFaderValues[i] = currentADC;

		  }

	  }
	  /*end ADC handler*/




	  //scan key matrix
	  for(int i = 0; i < 4; i++){

		  GPIOA->BRR = (0b1111 << 4);  //clear all of PA 4,5,6,7
		  GPIOA->BSRR = (1 << (4+i));  //energize the ith row
		  currentKeyMatrix |= ((((GPIOB->IDR) >> 3) & 0b11111) << (5*i)); //hmmmmmmmmm

	  }

	  //a key was pressed
	  if(currentKeyMatrix != lastKeyMatrix){

		  //handle keys here
		  for(int i = 0; i < 4; i++){
			  LEDMatrix[3-i] &= 0xf0; //clear the greens
			  LEDMatrix[3-i] |= (currentKeyMatrix >> ((5*i)+1)) & 0b1111;
			  //LEDMatrix[3-i] = (1<<i); //FRAK ZERO INDEXING alkfjngkjkfla (originally the idiot me had 4-i)
			  //hmmm, but on a more serious note tho, why is this array out of bounds not detected... that's definitely something to keep in mind
		  }

		  for(int i = 0; i < 4; i++){ //function to drive the LED's

			  LEDMatrixBuffer[i*4] = 0b1111; //clear all pins first to prevent ghosting
			  LEDMatrixBuffer[i*4+1] = 0x00;
			  LEDMatrixBuffer[i*4+2] = ~(1<<i);
			  LEDMatrixBuffer[i*4+3] = LEDMatrix[i];

		   }

		  for(int i = 0; i < 20; i++){ //iterate through all 20 bits and send out Midi Note messages as necessary

			  if((currentKeyMatrix & (1<<i)) && ((lastKeyMatrix & (1<<i)) == 0)){ //this key was pressed

				  MidiNoteOn(MidiChannel, MidiNoteLUT[i], 127);

			  }


			  else if((lastKeyMatrix & (1<<i)) && ((currentKeyMatrix & (1<<i)) == 0)){

				  MidiNoteOff(MidiChannel, MidiNoteLUT[i], 0);

			  }

		  }

		  lastKeyMatrix = currentKeyMatrix;
	  }

	  currentKeyMatrix = 0; //start afresh


	  if(!isLCDPrinting){ //update LCD here
		  //assumption: at most only one parameter will be changed with each loop, therefore, only allow for 1 queued write

		  if(LCDTopQueued){
			  LCDPrintStringTop(LCDQueueTop);
			  LCDTopQueued = 0;
		  }
		  else if(LCDBottomQueued){
			  LCDPrintStringBottom(LCDQueueBottom);
			  LCDBottomQueued = 0;
		  }

	  }



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* I2C2_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  ADC1->CR2 |= 1; //turn on ADC1
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  //dude, the code generation is evil... doesn't help you all the way!!! Came a gutsa so many times....


  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */
  //I2C2->CR2 |= (1<<9); //enable event interrupts
  //TODO: this is just temporarily here, might cause issues
  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 511;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  TIM2->CR1 &= ~(1<<1); //Clear the UDIS bit to ensure the BAM Interrupt is triggered
  TIM2->DIER |= 1; //Update interrupt enable

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 16383;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  TIM3->CR1 &= ~(1<<1); //Clear the UDIS bit to ensure the Encoder Scan Interrupt is triggered
  TIM3->DIER |= 1; //Update interrupt enable
  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7
                           PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB12 PB13 PB14
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
