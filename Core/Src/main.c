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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum bank {A, B} bank;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_Address 0b01001110
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile uint8_t brightness[4] = {1,1,1,1}; //initialize array holding brightness values for the 4 indicator LED's
volatile uint8_t BAMIndex = 0;
volatile uint8_t blocked = 0; //time critical BAM code running, do not disable interrupts

volatile uint8_t currentEncoder = 0; //which encoder are we polling right now?
volatile uint8_t lastEncoder[5] = {0,0,0,0,0};//initialize array containing past encoder readoff
volatile uint8_t encoderValues[5] = {0,0,0,0,0};//initialize array containing encoder values
volatile int8_t encoderLUT[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

uint8_t currentIOState[2] = {0, 0}; //current state of the LCD MCP23017




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void LCDCommand(char data, uint8_t addr);
void LCDData(char data, uint8_t addr);
void LCDInit(uint8_t addr);
void LCDWriteChar(char c, uint8_t addr);
void LCDWriteString(char *str, uint8_t addr);
void LCDClear(uint8_t addr);
void LCDCycleEN(uint8_t addr);
void LCDSetCursor(uint8_t row, uint8_t col, uint8_t addr);
void LCDShiftRight(uint8_t addr);
void LCDShiftLeft(uint8_t addr);
void MCP23017SetPin(uint8_t pin, bank b, uint8_t address);
void MCP23017ClearPin(uint8_t pin, bank b, uint8_t address);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void debug(){
	GPIOA->BSRR = (1<<7);
	GPIOA->BRR = (1<<7);
}

/* DWT based delay */
uint32_t DWT_Delay_Init(void)
{
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;
    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;
    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    /* Check if clock cycle counter has started */
    if(DWT->CYCCNT)
    {
       return 0; /*clock cycle counter started*/
    }
    else
    {
      return 1; /*clock cycle counter not started*/
    }
}

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t au32_microseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
  au32_microseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_microseconds-au32_ticks);
}

__STATIC_INLINE void DWT_Delay_ms(volatile uint32_t au32_milliseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000);
  au32_milliseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_milliseconds);
}

//https://deepbluembedded.com/stm32-delay-microsecond-millisecond-utility-dwt-delay-timer-delay/

/* MCP23017 Defines */

void MCP23017SetPin(uint8_t pin, bank b, uint8_t addr){

	GPIOA->BSRR = (1<<7);

	currentIOState[b] |= (1<<pin);

	//Note that all the I2C pointers are already volatile
	//I think I know the problem here, the start condition is sent, then the interrupt fires, then I2C DR has yet to be written, so the entire thing crashes in a pile of flames. It looks like the interrupt routine is plenty fast when compared to a full byte transfer, but just too long to squeeze into a start condition. YUP, CONFIRMED THAT IT GETS STUCK WAITING FOR THE ADDRESS FLAG, IE the address flag is not set!
	//write out the new state
	//UPDATE: This messses up the BAM Driver... I think it'll be better just to stop TIM2
	//__disable_irq(); //the entire routine will be super duper unhappy unless this is in place
	//TODO: investigate what happens if we disable irq, since the blocking code is already in place
	TIM2->CR1 &= ~1; //disable BAM Driver
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
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure BTF is 1
	I2C2->CR1 |= (1<<9); //send stop condition

	while ((I2C2->SR2 & (1<<1)) == 1); //make damn sure the I2C bus is free
	TIM2->CR1 |= 1; //enable BAM Driver
	//__enable_irq();
	GPIOA->BRR = (1<<7);

}

void MCP23017ClearPin(uint8_t pin, bank b, uint8_t addr){

	GPIOA->BSRR = (1<<7);
	//first, read current state of Bank B so we can safely toggle pins
	uint8_t current = 0; //current state of bank b
	//select register
	I2C2->CR1 |= (1<<8); //send start condition
	while ((I2C2->SR1 & 1) == 0); //clear SB
	I2C2->DR = addr; //address the MCP23017
	while ((I2C2->SR1 & (1<<1)) == 0); //wait for ADDR flag
	while ((I2C2->SR2 & (1<<2)) == 0); //read I2C SR2
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	if(b==A){
		I2C2->DR = 0x12; //read from bank A
	}
	else{
		I2C2->DR = 0x13;
	}
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure BTF is 1
	I2C2->CR1 |= (1<<9); //send stop condition

	while ((I2C2->SR2 & (1<<1)) == 1); //make damn sure the I2C bus is free

	//read in register contents

	I2C2->CR1 |= (1<<8); //send start condition
	while ((I2C2->SR1 & 1) == 0); //clear SB
	I2C2->DR = addr | 1; //address the MCP23017 and READ from it
	while ((I2C2->SR1 & (1<<1)) == 0); //wait for ADDR flag
	if(I2C2->SR2); //read I2C SR2 Note: Can't check for busyness, results in spurious extra byte read
	I2C2->CR1 &= ~(1 << 10); //Disable ACK and
	I2C2->CR1 |= (1 << 9); //queue stop condition here (Just after EV6)
	while ((I2C2->SR1 & (1<<6)) == 0); //while RxNE is low, ie data is still being read
	current = I2C2->DR; //read current state of Bank B
	I2C2->CR1 |= (1<<10); //Re-enable ACK

	while ((I2C2->SR2 & (1<<1)) == 1); //make damn sure the I2C bus is free

	current &= ~(1<<pin);

	//write out the new state
	I2C2->CR1 |= (1<<8); //send start condition
	while ((I2C2->SR1 & 1) == 0); //clear SB
	I2C2->DR = addr; //address the MCP23017
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
	I2C2->DR = current; //just pull everything low
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure BTF is 1
	I2C2->CR1 |= (1<<9); //send stop condition

	while ((I2C2->SR2 & (1<<1)) == 1); //make damn sure the I2C bus is free
	GPIOA->BRR = (1<<7);



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
void LCDInit(uint8_t addr){

	//Initialise the MCP23017 first
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
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure BTF is 1
	I2C2->CR1 |= (1<<9); //send stop condition

	//Pull RS, RW and E pins LOW
	I2C2->CR1 |= (1<<8); //send start condition
	while ((I2C2->SR1 & 1) == 0); //clear SB
	I2C2->DR = addr; //address the MCP23017
	while ((I2C2->SR1 & (1<<1)) == 0); //wait for ADDR flag
	while ((I2C2->SR2 & (1<<2)) == 0); //read I2C SR2
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	I2C2->DR = 0x15; //write to OLAT_B
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	I2C2->DR = 0x00; //just pull everything low
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure BTF is 1
	I2C2->CR1 |= (1<<9); //send stop condition


	LCDData(0x00, addr); //clear the data pins as well
	DWT_Delay_ms(30);

	LCDCommand(0x30, addr); //function set
	DWT_Delay_ms(5);

	LCDCommand(0x30, addr); //function set
	DWT_Delay_ms(5);

	LCDCommand(0x30, addr); //function set
	DWT_Delay_us(1000);

	LCDCommand(0x38, addr); //8-bit mode, 2 lines, smaller font

	LCDCommand(0x08, addr); //display off

	LCDCommand(0x01, addr); //display clear
	DWT_Delay_us(2000); //clear requires a substantial delay

	LCDCommand(0x06, addr); //set entry mode




}

/**
 * \fn LCDData
 * @brief Presents the data to D0 to D7 (located on Bank A)
 *
 * @param data Data to send
 * @param addr I2C Address of the MCP23017
 */
void LCDData(char data, uint8_t addr){

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
	while ((I2C2->SR1 & (1<<7)) == 0); //make sure BTF is 1
	I2C2->CR1 |= (1<<9); //send stop condition


}

void LCDCommand(char data, uint8_t addr){


	MCP23017ClearPin(RS_Pin, B, addr);

	LCDData(data, addr);

	LCDCycleEN(addr);

}

void LCDCycleEN(uint8_t addr){

	MCP23017ClearPin(EN_Pin, B, addr);
	DWT_Delay_us(1);
	MCP23017SetPin(EN_Pin, B, addr);
	DWT_Delay_us(1);
	MCP23017ClearPin(EN_Pin, B, addr);
	DWT_Delay_us(100);


}

void LCDWriteChar(char data, uint8_t addr){

	MCP23017SetPin(RS_Pin, B, addr);
	LCDData(data, addr);
	LCDCycleEN(addr);

}

void LCDWriteString(char *str, uint8_t addr){

	for(int i = 0; str[i]!= '\x00'; i++){ //Nice touch: take advantage of null byte terminated strings
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
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();

  I2C2->CR1 |= 1; //enable i2c 2 peripheral for LCD and EEPROM

  //Initialise the MCP23017 first
  	I2C2->CR1 |= (1<<8); //send start condition
  	while ((I2C2->SR1 & 1) == 0); //clear SB
  	I2C2->DR = LCD_Address; //address the MCP23017
  	while ((I2C2->SR1 & (1<<1)) == 0); //wait for ADDR flag
  	while ((I2C2->SR2 & (1<<2)) == 0); //read I2C SR2
  	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  	I2C2->DR = 0x00; //write to IODIR_A
  	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  	I2C2->DR = 0x00; //all outputs
  	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  	I2C2->DR = 0x00; //all outputs for next address which is IODIR_B
  	while ((I2C2->SR1 & (1<<7)) == 0); //make sure TxE is 1
  	while ((I2C2->SR1 & (1<<7)) == 0); //make sure BTF is 1
  	I2C2->CR1 |= (1<<9); //send stop condition


  //LCDInit(LCD_Address);

  //LCDClear(LCD_Address);

  //LCDSetCursor(1, 1, LCD_Address);

  //LCDWriteString("AAAA", LCD_Address);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  brightness[0] = encoderValues[3];
	  brightness[1] = encoderValues[2];
	  brightness[2] = encoderValues[1];
	  brightness[3] = encoderValues[0];

	  //LCDShiftLeft(LCD_Address);
	  DWT_Delay_us(500);
	  if (!blocked) MCP23017SetPin(0, B, LCD_Address); //spam if not blocked
	  //MCP23017ClearPin(0, B, LCD_Address);


	  //LCDShiftRight(LCD_Address);
	  //DWT_Delay_ms(500);
	  //LCDCycleEN(LCD_Address);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

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
  hi2c2.Init.ClockSpeed = 400000;
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

  /* USER CODE END I2C2_Init 2 */

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
  htim2.Init.Period = 2047;
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
  TIM2->CR1 |= 1; //enable BAM Driver
  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
