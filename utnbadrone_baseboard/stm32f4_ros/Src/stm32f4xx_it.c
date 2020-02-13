/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
#include "string.h"
#include "mainpp.h"

extern TIM_HandleTypeDef htim1;
extern osSemaphoreId GPSIntSemHandle;
extern osMessageQId IMUQueueHandle;
extern TIM_HandleTypeDef htim13;


extern volatile uint8_t buffer[PINGPONG_SIZE], databuf[PINGPONG_SIZE];
extern volatile uint32_t movement_counter;

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;

extern TIM_HandleTypeDef htim14;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  uint32_t count;
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
  count = __HAL_TIM_GetCounter(&htim13);    //read TIM13 counter value
  __HAL_TIM_SetCounter(&htim13,0);			//reset counter value
  osMessagePut(IMUQueueHandle,count,0);
//  HAL_GPIO_TogglePin(LD4_GPIO_Port,LD4_Pin);
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream1 global interrupt.
*/
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream2 global interrupt.
*/
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */
		uint32_t len = 0, tocopy = 0;
		uint32_t Write = 0;

		/* FIXME: Para que estaba este regs? */
	//	DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma_uart4_rx.StreamBaseAddress;

		if(__HAL_DMA_GET_IT_SOURCE(&hdma_uart4_rx, DMA_IT_TC) != RESET)   // if the source is TC
		{
			/* Get the length of the data */
			len = PINGPONG_SIZE - hdma_uart4_rx.Instance->NDTR;

			/* Get number of bytes we can copy to the end of buffer */
			tocopy = PINGPONG_SIZE - Write;

			/* Check how many bytes to copy */
			if (tocopy > len)
				tocopy = len;
			/* Write received data for UART main buffer for manipulation later */
		   memcpy((void*)&databuf[Write],(const void*) buffer, tocopy);   /* Copy first part */

		   /* EN REEMPLAZO */
	//	   gps_int_state = 1;
		   osSemaphoreRelease(GPSIntSemHandle);

	//	   regs->IFCR = 0x3FU << hdma_uart4_rx.StreamIndex; // clear all interrupts
	//	   hdma_uart4_rx.Instance->FCR = 0x3FU << hdma_uart4_rx.StreamIndex;
		   hdma_uart4_rx.Instance->M0AR = (uint32_t)buffer;   /* Set memory address for DMA again */
		   hdma_uart4_rx.Instance->NDTR = PINGPONG_SIZE;    /* Set number of bytes to receive */
		   hdma_uart4_rx.Instance->CR |= DMA_SxCR_EN;            /* Start DMA transfer */
		}
  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream3 global interrupt.
*/
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
* @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
*/
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
* @brief This function handles UART4 global interrupt.
*/
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */
  if (huart4.Instance->SR & UART_FLAG_IDLE)           /* if Idle flag is set */
  {
	  /* FIXME: Para que estaba este tmp? */
//	  volatile uint32_t tmp;                  /* Must be volatile to prevent optimizations */
//	  tmp = huart4.Instance->SR;                       /* Read status register */
//	  tmp = huart4.Instance->DR;                       /* Read data register */
	  hdma_uart4_rx.Instance->CR &= ~DMA_SxCR_EN;       /* Disabling DMA will force transfer complete interrupt if enabled */
  }

  /* USER CODE END UART4_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	movement_counter ++;
	if(movement_counter >= 700)
	{
		movement_counter = 0;
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
	}
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
