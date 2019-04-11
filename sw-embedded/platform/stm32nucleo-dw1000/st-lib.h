/*
 * Copyright (c) 2012, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * \defgroup stm32nucleo-dw1000  STM32Cube HAL APIs
 *
 * Abstraction of STM32Cube HAL APIs as per Contiki coding rules
 * @{
 *
 * \file
 * Header file for the STM32Cube HAL APIs
 */
/*---------------------------------------------------------------------------*/
#ifndef ST_LIB_H_
#define ST_LIB_H_

/*---------------------------------------------------------------------------*/
/* extern global varialbles */
#define st_lib_uart_handle                       UartHandle

#define st_lib_g_x_status                        g_xStatus

#define st_lib_p_spi_handle                      pSpiHandle
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* misc */
#define st_lib_tim2_irq_handler(...)                      TIM2_IRQHandler(__VA_ARGS__)
#define st_lib_sys_tick_handler(...)                      SysTick_Handler(__VA_ARGS__)

/*---------------------------------------------------------------------------*/
/* stm32l152xe.h */
#include "stm32l152xe.h"

#define st_lib_irq_n_type                        IRQn_Type
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* stm32l1xx.h */
#include "stm32l1xx.h"

#define st_lib_flag_status                       FlagStatus
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* stm32l1xx_hal_cortex.h */
#include "stm32l1xx_hal_cortex.h"

#define st_lib_hal_nvic_enable_irq(...)           HAL_NVIC_EnableIRQ(__VA_ARGS__)
#define st_lib_hal_nvic_set_priority(...)         HAL_NVIC_SetPriority(__VA_ARGS__)
#define st_lib_hal_systick_clk_source_config(...) HAL_SYSTICK_CLKSourceConfig(__VA_ARGS__)
#define st_lib_hal_systick_config(...)            HAL_SYSTICK_Config(__VA_ARGS__)
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* stm32l1xx_hal_rcc.h */
#include "stm32l1xx_hal_rcc.h"

#define st_lib_tim2_clk_enable(...)              __TIM2_CLK_ENABLE(__VA_ARGS__)
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* stm32l1xx_hal_spi.h */
#include "stm32l1xx_hal_spi.h"

#define st_lib_spi_handle_typedef                SPI_HandleTypeDef

#define st_lib_hal_spi_get_flag(...)             __HAL_SPI_GET_FLAG(__VA_ARGS__)
#define st_lib_hal_spi_transmit_receive(...)     HAL_SPI_TransmitReceive(__VA_ARGS__)
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* stm32l1xx_hal_tim.h */
#include "stm32l1xx_hal_tim.h"

#define st_lib_tim_handle_typedef                TIM_HandleTypeDef
#define st_lib_tim_clock_config_typedef          TIM_ClockConfigTypeDef
#define st_lib_tim_oc_init_typedef               TIM_OC_InitTypeDef

#define st_lib_hal_tim_base_init(...)            HAL_TIM_Base_Init(__VA_ARGS__)
#define st_lib_hal_tim_base_start_it(...)        HAL_TIM_Base_Start_IT(__VA_ARGS__)
#define st_lib_hal_tim_config_clock_source(...)  HAL_TIM_ConfigClockSource(__VA_ARGS__)
#define st_lib_hal_tim_clear_flag(...)           __HAL_TIM_CLEAR_FLAG(__VA_ARGS__)
#define st_lib_hal_tim_clear_it(...)             __HAL_TIM_CLEAR_IT(__VA_ARGS__)
#define st_lib_hal_tim_enable(...)               __HAL_TIM_ENABLE(__VA_ARGS__)
#define st_lib_hal_tim_enable_it(...)            __HAL_TIM_ENABLE_IT(__VA_ARGS__)
#define st_lib_hal_tim_oc_init(...)              HAL_TIM_OC_Init(__VA_ARGS__)
#define st_lib_hal_tim_oc_config_channel(...)    HAL_TIM_OC_ConfigChannel(__VA_ARGS__)
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* stm32l1xx_hal_uart.h */
#include "stm32l1xx_hal_uart.h"

#define st_lib_uart_handle_typedef               UART_HandleTypeDef

#define st_lib_hal_uart_enable_it(...)           __HAL_UART_ENABLE_IT(__VA_ARGS__)
#define st_lib_hal_uart_init(...)                HAL_UART_Init(__VA_ARGS__)
#define st_lib_hal_uart_receive(...)             HAL_UART_Receive(__VA_ARGS__)
#define st_lib_hal_uart_receive_it(...)          HAL_UART_Receive_IT(__VA_ARGS__)
#define st_lib_hal_uart_rx_cplt_callback(...)    HAL_UART_RxCpltCallback(__VA_ARGS__)
#define st_lib_hal_uart_transmit(...)            HAL_UART_Transmit(__VA_ARGS__)
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* stm32l1xx_nucleo.h */
#include "stm32l1xx_nucleo.h"

#define st_lib_gpio_typedef                      GPIO_TypeDef
#define st_lib_gpio_port                         GPIO_PORT
#define st_lib_gpio_pin                          GPIO_PIN

#define st_lib_bsp_led_init(...)                 BSP_LED_Init(__VA_ARGS__)
#define st_lib_bsp_led_off(...)                  BSP_LED_Off(__VA_ARGS__)
#define st_lib_bsp_led_on(...)                   BSP_LED_On(__VA_ARGS__)
#define st_lib_bsp_pb_init(...)                  BSP_PB_Init(__VA_ARGS__)
#define st_lib_bsp_pb_get_state(...)             BSP_PB_GetState(__VA_ARGS__)
#define st_lib_hal_gpio_read_pin(...)            HAL_GPIO_ReadPin(__VA_ARGS__)
#define st_lib_hal_gpio_write_pin(...)           HAL_GPIO_WritePin(__VA_ARGS__)
/*---------------------------------------------------------------------------*/

#ifdef X_NUCLEO_IKS01A1
/*---------------------------------------------------------------------------*/
/* x_nucleo_iks01a1.h */
#include "x_nucleo_iks01a1.h"

#define st_lib_axes_raw_typedef                  AxesRaw_TypeDef
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* x_nucleo_iks01a1_hum_temp.h */
#include "x_nucleo_iks01a1_hum_temp.h"

#define st_lib_bsp_hum_temp_is_initialized(...)  BSP_HUM_TEMP_isInitialized(__VA_ARGS__)
#define st_lib_bsp_hum_temp_init(...)            BSP_HUM_TEMP_Init(__VA_ARGS__)
#define st_lib_bsp_hum_temp_get_humidity(...)    BSP_HUM_TEMP_GetHumidity(__VA_ARGS__)
#define st_lib_bsp_hum_temp_get_temperature(...) BSP_HUM_TEMP_GetTemperature(__VA_ARGS__)
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* x_nucleo_iks01a1_imu_6axes.h */
#include "x_nucleo_iks01a1_imu_6axes.h"

#define st_lib_bsp_imu_6axes_is_initialized(...) BSP_IMU_6AXES_isInitialized(__VA_ARGS__)
#define st_lib_bsp_imu_6axes_init(...)           BSP_IMU_6AXES_Init(__VA_ARGS__)
#define st_lib_bsp_imu_6axes_g_get_axes_raw(...) BSP_IMU_6AXES_G_GetAxesRaw(__VA_ARGS__)
#define st_lib_bsp_imu_6axes_x_get_axes_raw(...) BSP_IMU_6AXES_X_GetAxesRaw(__VA_ARGS__)
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* x_nucleo_iks01a1_magneto.h */
#include "x_nucleo_iks01a1_magneto.h"

#define st_lib_bsp_magneto_m_get_axes_raw(...)   BSP_MAGNETO_M_GetAxesRaw(__VA_ARGS__)
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* x_nucleo_iks01a1_pressure.h */
#include "x_nucleo_iks01a1_pressure.h"

#define st_lib_bsp_pressure_init(...)            BSP_PRESSURE_Init(__VA_ARGS__)
#define st_lib_bsp_pressure_get_pressure(...)    BSP_PRESSURE_GetPressure(__VA_ARGS__)
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#endif /*X_NUCLEO_IKS01A1*/
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#endif /*ST_LIB_H_*/
/*---------------------------------------------------------------------------*/
/** @} */
