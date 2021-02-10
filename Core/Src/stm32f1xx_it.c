/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern volatile uint8_t brightness[4];
extern volatile uint8_t BAMIndex;
extern volatile uint8_t blocked;

extern volatile uint8_t currentEncoder;
extern volatile uint8_t lastEncoder[5];
extern volatile int8_t encoderLUT[16];
extern volatile int encoderValues[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

	I2C1->CR1 |= (1<<9); //generate stop condition

	/* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	//GPIOA->BSRR = 1<<6;
	if(BAMIndex == 0){
		blocked = 1; //block to protect the time sensitive LSB's, otherwise it gets pretty flicker-ry

	}

	if(brightness[0] & (1 << BAMIndex))	GPIOB->BSRR = (1<<12);
	else GPIOB->BRR = (1<<12);
	if(brightness[1] & (1 << BAMIndex))	GPIOB->BSRR = (1<<13);
	else GPIOB->BRR = (1<<13);
	if(brightness[2] & (1 << BAMIndex))	GPIOB->BSRR = (1<<14);
	else GPIOB->BRR = (1<<14);
	if(brightness[3] & (1 << BAMIndex))	GPIOB->BSRR = (1<<15);
	else GPIOB->BRR = (1<<15);


	/*
	//this new routine 2.16us :/
	GPIOB->BRR = (0b1111 << 12);
	GPIOB->BSRR = ((brightness[0] & (1 << BAMIndex)) << (12 - BAMIndex) | (brightness[1] & (1 << BAMIndex)) << (13 - BAMIndex) | (brightness[2] & (1 << BAMIndex)) << (14 - BAMIndex) | (brightness[3] & (1 << BAMIndex)) << (15 - BAMIndex));
	*/

	/* no improvement...
	GPIOB->BRR = (0b1111 << 12);
	if(brightness[0] & (1 << BAMIndex))	GPIOB->BSRR = (1<<12);
	if(brightness[1] & (1 << BAMIndex))	GPIOB->BSRR = (1<<13);
	if(brightness[2] & (1 << BAMIndex))	GPIOB->BSRR = (1<<14);
	if(brightness[3] & (1 << BAMIndex))	GPIOB->BSRR = (1<<15);
	*/

	//GPIOA->BRR = 1<<6;
	/*
	for(int i = 0; i < 4; i++){ //BAM all 4 LED's

		if(brightness[i] & (1 << BAMIndex)){
			GPIOB->BSRR = (1<<(i+12));
		}
		else{
			GPIOB->BSRR = (1<<(i+12))
		}

	}


*/

	//FIXME this might potentially cause issues, as it blocks for half of the time
	if(BAMIndex == 3){
		blocked = 0; //Time sensitive LSB's are done, unblock, value of 3 or less gives visible flicker

	}

	if(BAMIndex == 7){ //We've passed one BAM cycle


		BAMIndex = 0;
		TIM2->PSC = 1;



	}
	else{
		BAMIndex++;
		TIM2->PSC = (volatile)(TIM2->PSC << 1); //set next write to occupy twice the time of this current write.
	}

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

	GPIOA->BSRR = 1<<6;
	uint8_t currentReadoff = ((((GPIOA->IDR)>>9) & 1) << 1) | (((GPIOA->IDR)>>10) & 1); //read current encoder state
	uint8_t index = (lastEncoder[currentEncoder]<<2) | currentReadoff;
	encoderValues[currentEncoder] += encoderLUT[index];

	//constrain encoderValues
	if(encoderValues[currentEncoder] > 255) encoderValues[currentEncoder] = 255;
	if(encoderValues[currentEncoder] < 0) encoderValues[currentEncoder] = 0;

	lastEncoder[currentEncoder] = currentReadoff;

	//if(currentEncoder == 0){
	//uint8_t buffer[256];
	//sprintf(buffer, "currentReadoff %d index %d encoderValue %d\r\n", currentReadoff, index, encoderValues[0]);
	//CDC_Transmit_FS(buffer, sizeof(buffer));
	//}

	if(currentEncoder == 4) currentEncoder = 0;
	else currentEncoder++;

	//select the nth encoder here to allow the mux time to settle
	GPIOC->BRR = (3<<13); //clear GPIO Pins
	GPIOC->BSRR = ((currentEncoder&3)<<13);
	GPIOA->BRR = (1<<15);
	if(currentEncoder&4) GPIOA->BSRR = (1<<15); //BLOODY SOLDER DAG!!! Shorted out the pins giving the result in DS14

	GPIOA->BRR = 1<<6;
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
