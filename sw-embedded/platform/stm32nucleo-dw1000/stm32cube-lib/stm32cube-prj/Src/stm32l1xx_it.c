/**
******************************************************************************
* @file    stm32l1xx_it.c
* @author  System LAB
* @version V1.0.0
* @date    17-June-2015
* @brief   Main Interrupt Service Routines
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "stm32cube_hal_init.h"
#include "stm32l1xx_hal_gpio.h"
#include "stm32l1xx_it.h"
#include "stm32l1xx_nucleo.h"

#include "dw1000-arch.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifndef RADIO_GPIO_3_EXTI_LINE
#define RADIO_GPIO_3_EXTI_LINE                     GPIO_PIN_7
#endif

#ifndef RADIO_GPIO_3_PIN
#define RADIO_GPIO_3_PIN                           GPIO_PIN_7
#endif

#ifndef RADIO_GPIO_3_PORT
#define RADIO_GPIO_3_PORT                          GPIOC
#endif

extern UART_HandleTypeDef UartHandle;

/** @addtogroup STM32L1xx_HAL_Examples
* @{
*/

/** @addtogroup Templates
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/
extern I2C_HandleTypeDef I2cHandle;
/**
* @brief   This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
}
void WWDG_IRQHandler(void)
{
  while(1);
}
/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  PRINTF("HardFault_Handler called at %08x\n", HardFault_Handler);
  while (1)
  {
  }
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
void SVC_Handler(void)
{
}

/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
}

/**
* @brief  This function handles PendSVC exception.
* @param  None
* @retval None
*/
void PendSV_Handler(void)
{
}


/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx.s).                                               */
/******************************************************************************/
/**
* @brief  This function handles I2C event interrupt request.  
* @param  None
* @retval None
* @Note   This function is redefined in "stm32cube_hal_init.h" and related to I2C data transmission     
*/
void I2Cx_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(& I2cHandle);
}

/**
* @brief  This function handles I2C error interrupt request.
* @param  None
* @retval None
* @Note   This function is redefined in "stm32cube_hal_init.h" and related to I2C error
*/
void I2Cx_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(& I2cHandle);
}


/**
* @brief  This function handles External lines 15 to 4 interrupt request.
* @param  None
* @retval None
*/
void EXTI0_IRQHandler(void)
{
    PRINTF("EXTI0_IRQHandler");
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0))
    {

        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
        HAL_GPIO_EXTI_Callback(RADIO_GPIO_3_EXTI_LINE);

        PRINTF("Interrupt 0\n");
        dw1000_isr();
        PRINTF("After dw100interrupt\n");

    }
}

void EXTI1_IRQHandler(void)
{
  /* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET) 
  { 
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
  }  
  //while(1);
  PRINTF("EXTI1_IRQHandler\n");
}

void EXTI2_IRQHandler(void)
{
  /* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET) 
  { 
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
  }
  //while(1);
  PRINTF("EXTI2_IRQHandler\n");
}

void EXTI3_IRQHandler(void)
{
  /* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != RESET) 
  { 
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
  }
  PRINTF("EXTI3_IRQHandler\n");
  //while(1);
}


/**
* @brief  This function handles External lines 15 to 4 interrupt request.
* @param  None
* @retval None
*/

void EXTI9_5_IRQHandler(void) {
    PRINTF("EXTI9_5_IRQHandler\n");
	/* EXTI line 5 interrupt detected */
	if (__HAL_GPIO_EXTI_GET_IT(RADIO_GPIO_3_EXTI_LINE)) {
		__HAL_GPIO_EXTI_CLEAR_IT(RADIO_GPIO_3_EXTI_LINE);
		//HAL_GPIO_EXTI_Callback(RADIO_GPIO_3_EXTI_LINE);

		PRINTF("calling dw1000_isr at %08x\n", dw1000_isr);
		do {
			dw1000_isr();
		} while (HAL_GPIO_ReadPin(RADIO_GPIO_3_PORT, RADIO_GPIO_3_PIN)
				== GPIO_PIN_SET);

	}

}


/**
* @brief  This function handles EXTI15_10_IRQHandler
* @param  None
* @retval None
*/
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(USER_BUTTON_PIN);
  PRINTF("EXTI15_10_IRQHandler\n");
}


/**
* @brief  This function handles UART interrupt request.  
* @param  None
* @retval None
* @Note   This function is redefined in "stm32cube_hal_init.h" and related to DMA  
*         used for USART data transmission     
*/

void USART2_IRQHandler()
{
	UART_HandleTypeDef *huart = &UartHandle;

	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_PE)){
		__HAL_UART_CLEAR_PEFLAG(huart);
	}
	
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_FE)){
		__HAL_UART_CLEAR_FEFLAG(huart);
	}
	
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)){
		__HAL_UART_CLEAR_NEFLAG(huart);  
	}
	
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)){
		__HAL_UART_CLEAR_OREFLAG(&UartHandle);
	}
	
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)){
		slip_input_byte(UartHandle.Instance->DR);
		__HAL_UART_GET_IT_SOURCE(&UartHandle, UART_IT_RXNE);
	}
}

/**
* @brief  UART error callbacks
* @param  UartHandle: UART handle
* @note   This example shows a simple way to report transfer error, and you can
*         add your own implementation.
* @retval None
*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  // Error_Handler();
}


/**
* @brief  Tx Transfer completed callback
* @param  UartHandle: UART handle. 
* @note   This example shows a simple way to report end of IT Tx transfer, and 
*         you can add your own implementation. 
* @retval None
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  
  
}


/**
* @}
*/ 

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
