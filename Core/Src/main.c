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
uint8_t lastEncoderValues[5] = {0, 0, 0, 0, 0};

uint8_t lastButtonState = 1; //menu encoder's button

uint8_t lastFaderValues[4] = {0, 0, 0, 0};


uint32_t lastKeyMatrix = 0; //last state of the key matrix
uint32_t currentKeyMatrix = 0; //current state of the key matrix


uint8_t MidiNoteLUT[20]; //which key maps to which note
uint8_t MidiNoteOffset = 60; //which key does the bottom left button map to?
uint8_t MidiChannel = 0;
uint8_t MidiCCFaderLUT[4] = {1, 7, 10, 11}; //which fader maps to which CC
uint8_t MidiCCEncoderLUT[4] = {1, 7, 10, 11};

uint8_t LCDQueueTop[17];
uint8_t LCDQueueBottom[17];
uint8_t LCDTopQueued; //does the LCD have to be updated this round?
uint8_t LCDBottomQueued;


//menu stuff
typedef enum mainMenu {EncoderSettings, FaderSettings, KeypadSettings, Exit} _mainMenu;
typedef enum menuState {StatusDisplay, MainMenu, SubMenu, SettingsActive} _menuState;

_menuState menuState = StatusDisplay;
_mainMenu mainMenuPage = EncoderSettings;
int subMenuPage = 0;

char mainMenuItems[16][16] = {"Encoder Settings", "Fader Settings", "KeypadSettings", "Exit"};
char subMenuItems[16][16][16] = {{"Encoder 1 CC", "Encoder 1 Vel", "Exit"}, {"Encoder 2 CC", "Encoder 2 Vel", "Exit"}, {"Encoder 3 CC", "Encoder 3 Vel", "Exit"}, {"Encoder 4 CC", "Encoder 4 Vel", "Exit"},
								 {"Fader 1 CC", "Exit"}, {"Fader 2 CC", "Exit"}, {"Fader 3 CC", "Exit"}, {"Fader 4 CC", "Exit"},
								 {"Keypad Offset", "Exit"}
};



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

  for(int i = 0; i < 4; i++){

	  int currentADC = ADC1ReadVal8(i);
	  currentADC += ADC1ReadVal8(i);
	  currentADC += ADC1ReadVal8(i);
	  currentADC = currentADC/3;

	  lastFaderValues[i] = currentADC;


  }

  for(int i = 0; i < 5; i++){

	  lastEncoderValues[i] = encoderValues[i];

  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  IWDG->KR = 0xAAAA; //reset the watchdog timer
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  brightness[0] = encoderValues[3];
	  brightness[1] = encoderValues[2];
	  brightness[2] = encoderValues[1];
	  brightness[3] = encoderValues[0];


	 uint8_t currentButtonState = ((GPIOB->IDR)&1);
	 if (currentButtonState == 0 && lastButtonState == 1){ //button has been pressed
		 switch (menuState){

		 case StatusDisplay:
			 menuState = MainMenu; //enter main menu

		 case MainMenu:

			 if(mainMenuPage == Exit){

				 menuState = StatusDisplay;

			 }
			 else menuState = SubMenu;

		 case SubMenu:
			 if(subMenuItems[mainMenuPage][subMenuPage] == "Exit"){
				 menuState = MainMenu;
			 }
			 else{

				 menuState = SettingsActive;

			 }

		 }
	 }




	  for(int i = 0; i < 4; i++){ //send encoder CC Values

		  if(encoderValues[i] != lastEncoderValues[i]){

			  MidiCC(MidiChannel, MidiCCEncoderLUT[i], (encoderValues[i]>>1));
			  snprintf(LCDQueueTop, 17, "Encoder %-8d", i);
			  LCDTopQueued = 1;
			  snprintf(LCDQueueBottom, 17, "%-16d", encoderValues[i]);
			  LCDBottomQueued = 1;

			  lastEncoderValues[i] = encoderValues[i];

		  }

	  }

	  for(int i = 1; i < 4; i++){

		  int currentADC = ADC1ReadVal8(i);
		  currentADC += ADC1ReadVal8(i);
		  currentADC += ADC1ReadVal8(i);
		  currentADC = currentADC/3;

		  if(abs(currentADC - lastFaderValues[i]) > 3){ // this particular ADC Channel has been updated

			  MidiCC(MidiChannel, MidiCCFaderLUT[i], (currentADC>>1));
			  lastFaderValues[i] = currentADC;

		  }

	  }





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

			  LEDMatrix[3-i] = (currentKeyMatrix >> ((5*i)+1)) & 0b1111;
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
		  if(LCDTopQueued){
			  LCDPrintStringTop(LCDQueueTop);
			  LCDTopQueued = 0;
		  }
		  else if(LCDBottomQueued){
			  LCDPrintStringBottom(LCDQueueBottom);
			  LCDBottomQueued = 0;
		  }

	  }







	  DWT_Delay_ms(10);

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

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
