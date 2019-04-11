/**
 * \file
 *      Platform Dependent DW1000 Driver Source File
 *
 */

//#include "stm32l1xx_hal_spi.h"
#include "contiki.h"
#include "sys/clock.h"
#include "leds.h"
#include "dw1000-arch.h"

/*---------------------------------------------------------------------------*/
#include "deca_device_api.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


static SPI_HandleTypeDef pSpiHandle;

#define RADIO_GPIO_NUMBER 5

#define RADIO_GPIO_0_PORT                          GPIOA
#define RADIO_GPIO_0_PIN                           GPIO_PIN_9
#define RADIO_GPIO_0_CLOCK_ENABLE()                __GPIOC_CLK_ENABLE()
#define RADIO_GPIO_0_CLOCK_DISABLE()               __GPIOC_CLK_DISABLE()
#define RADIO_GPIO_0_SPEED                         GPIO_SPEED_HIGH
#define RADIO_GPIO_0_PUPD                          GPIO_NOPULL
#define RADIO_GPIO_0_EXTI_LINE                     GPIO_PIN_9
#define RADIO_GPIO_0_EXTI_MODE                     GPIO_MODE_IT_FALLING
#define RADIO_GPIO_0_EXTI_IRQN                     EXTI1_IRQn
#define RADIO_GPIO_0_EXTI_PREEMPTION_PRIORITY      2
#define RADIO_GPIO_0_EXTI_SUB_PRIORITY             2
#define RADIO_GPIO_0_EXTI_IRQ_HANDLER              EXTI1_IRQHandler

#define RADIO_GPIO_1_PORT                          GPIOA
#define RADIO_GPIO_1_PIN                           GPIO_PIN_8
#define RADIO_GPIO_1_CLOCK_ENABLE()                __GPIOB_CLK_ENABLE()
#define RADIO_GPIO_1_CLOCK_DISABLE()               __GPIOB_CLK_ENABLE()
#define RADIO_GPIO_1_SPEED                         GPIO_SPEED_HIGH
#define RADIO_GPIO_1_PUPD                          GPIO_NOPULL
#define RADIO_GPIO_1_EXTI_LINE                     GPIO_PIN_9
#define RADIO_GPIO_1_EXTI_MODE                     GPIO_MODE_IT_FALLING
#define RADIO_GPIO_1_EXTI_IRQN                     EXTI0_IRQn
#define RADIO_GPIO_1_EXTI_PREEMPTION_PRIORITY      2
#define RADIO_GPIO_1_EXTI_SUB_PRIORITY             2
#define RADIO_GPIO_1_EXTI_IRQ_HANDLER              EXTI0_IRQHandler

#define RADIO_GPIO_2_PORT                          GPIOA
#define RADIO_GPIO_2_PIN                           GPIO_PIN_4
#define RADIO_GPIO_2_CLOCK_ENABLE()                __GPIOA_CLK_ENABLE()
#define RADIO_GPIO_2_CLOCK_DISABLE()               __GPIOA_CLK_ENABLE()
#define RADIO_GPIO_2_SPEED                         GPIO_SPEED_HIGH
#define RADIO_GPIO_2_PUPD                          GPIO_NOPULL
#define RADIO_GPIO_2_EXTI_LINE                     GPIO_PIN_4
#define RADIO_GPIO_2_EXTI_MODE                     GPIO_MODE_IT_FALLING
#define RADIO_GPIO_2_EXTI_IRQN                     EXTI4_IRQn
#define RADIO_GPIO_2_EXTI_PREEMPTION_PRIORITY      2
#define RADIO_GPIO_2_EXTI_SUB_PRIORITY             2
#define RADIO_GPIO_2_EXTI_IRQ_HANDLER              EXTI4_IRQHandler

#define RADIO_GPIO_3_PORT                          GPIOC
#define RADIO_GPIO_3_PIN                           GPIO_PIN_7
#define RADIO_GPIO_3_CLOCK_ENABLE()              __GPIOC_CLK_ENABLE()
#define RADIO_GPIO_3_CLOCK_DISABLE()             __GPIOC_CLK_DISABLE()
#define RADIO_GPIO_3_SPEED                         GPIO_SPEED_HIGH
#define RADIO_GPIO_3_PUPD                          GPIO_NOPULL
#define RADIO_GPIO_3_EXTI_LINE                     GPIO_PIN_7
#define RADIO_GPIO_3_EXTI_MODE                     GPIO_MODE_IT_RISING
#define RADIO_GPIO_3_EXTI_IRQN                     EXTI9_5_IRQn
#define RADIO_GPIO_3_EXTI_PREEMPTION_PRIORITY      2
#define RADIO_GPIO_3_EXTI_SUB_PRIORITY             2
#define RADIO_GPIO_3_EXTI_IRQ_HANDLER              EXTI9_5_IRQHandler

#define RADIO_GPIO_SDN_PORT                        GPIOA
#define RADIO_GPIO_SDN_PIN                         GPIO_PIN_10
#define RADIO_GPIO_SDN_CLOCK_ENABLE()            __GPIOA_CLK_ENABLE()
#define RADIO_GPIO_SDN_CLOCK_DISABLE()           __GPIOA_CLK_DISABLE()
#define RADIO_GPIO_SDN_SPEED                       GPIO_SPEED_HIGH
#define RADIO_GPIO_SDN_PUPD                        GPIO_PULLUP

#define RADIO_GPIO_IRQ      RADIO_GPIO_3

/**
* @brief  Radio_Gpio Port array
*/
GPIO_TypeDef* aRADIO_GPIO_PORT[RADIO_GPIO_NUMBER] = {
 RADIO_GPIO_0_PORT,
 RADIO_GPIO_1_PORT,
 RADIO_GPIO_2_PORT,
 RADIO_GPIO_3_PORT,
 RADIO_GPIO_SDN_PORT
};


/**
* @brief  Radio_Gpio Pin array
*/
static const uint16_t aRADIO_GPIO_PIN[RADIO_GPIO_NUMBER] = {
 RADIO_GPIO_0_PIN,
 RADIO_GPIO_1_PIN,
 RADIO_GPIO_2_PIN,
 RADIO_GPIO_3_PIN,
 RADIO_GPIO_SDN_PIN
};


/**
* @brief  Radio_Gpio Speed array
*/
static const uint32_t aRADIO_GPIO_SPEED[RADIO_GPIO_NUMBER] = {
 RADIO_GPIO_0_SPEED,
 RADIO_GPIO_1_SPEED,
 RADIO_GPIO_2_SPEED,
 RADIO_GPIO_3_SPEED,
 RADIO_GPIO_SDN_SPEED
};


/**
* @brief  Radio_Gpio PuPd array
*/
static const uint32_t aRADIO_GPIO_PUPD[RADIO_GPIO_NUMBER] = {
 RADIO_GPIO_0_PUPD,
 RADIO_GPIO_1_PUPD,
 RADIO_GPIO_2_PUPD,
 RADIO_GPIO_3_PUPD,
 RADIO_GPIO_SDN_PUPD
};


/**
* @brief  Exti Mode array
*/
static const uint32_t aRADIO_GPIO_EXTI_MODE[RADIO_GPIO_NUMBER-1] = {
 RADIO_GPIO_0_EXTI_MODE,
 RADIO_GPIO_1_EXTI_MODE,
 RADIO_GPIO_2_EXTI_MODE,
 RADIO_GPIO_3_EXTI_MODE
};


/**
* @brief  Exti IRQn array
*/
static const uint8_t aRADIO_GPIO_IRQn[RADIO_GPIO_NUMBER-1] = {
 RADIO_GPIO_0_EXTI_IRQN,
 RADIO_GPIO_1_EXTI_IRQN,
 RADIO_GPIO_2_EXTI_IRQN,
 RADIO_GPIO_3_EXTI_IRQN
};


/**
* @brief  Configures MCU GPIO and EXTI Line for GPIOs.
* @param  xGpio Specifies the GPIO to be configured.
*         This parameter can be one of following parameters:
*         @arg GPIO_0
*         @arg GPIO_1
*         @arg GPIO_2
*         @arg GPIO_3
* @param  xGpioMode Specifies GPIO mode.
*         This parameter can be one of following parameters:
*         @arg RADIO_MODE_GPIO_IN: MCU GPIO will be used as simple input.
*         @argRADIO_MODE_GPIO_OUT: MCU GPIO will be used as simple output.
*         @arg RADIO_MODE_EXTI_IN: MCU GPIO will be connected to EXTI line with interrupt
*         generation capability.
* @retval None.
*/
void
RadioGpioInit(RadioGpioPin xGpio, RadioGpioMode xGpioMode)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Check the parameters */
    assert_param(IS_RADIO_GPIO_PIN(xGpio));
    assert_param(IS_RADIO_GPIO_MODE(xGpioMode));

    /* GPIO Ports Clock Enable */
    __GPIOA_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();

    /* Configures MCU GPIO */
    if (xGpioMode == RADIO_MODE_GPIO_OUT)
    {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    }
    else
    {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    }

    GPIO_InitStruct.Pin = aRADIO_GPIO_PIN[xGpio];
    GPIO_InitStruct.Pull = aRADIO_GPIO_PUPD[xGpio];
    GPIO_InitStruct.Speed = aRADIO_GPIO_SPEED[xGpio];
    HAL_GPIO_Init(aRADIO_GPIO_PORT[xGpio], &GPIO_InitStruct);

    if (xGpioMode == RADIO_MODE_EXTI_IN)
    {
        GPIO_InitStruct.Pin = aRADIO_GPIO_PIN[xGpio];
        GPIO_InitStruct.Pull = aRADIO_GPIO_PUPD[xGpio];
        GPIO_InitStruct.Speed = aRADIO_GPIO_SPEED[xGpio];
        GPIO_InitStruct.Mode = aRADIO_GPIO_EXTI_MODE[xGpio];
        HAL_GPIO_Init(aRADIO_GPIO_PORT[xGpio], &GPIO_InitStruct);

        /* Enable and set Button EXTI Interrupt to the lowest priority */
        /*  NVIC_SetPriority((IRQn_Type)(aRADIO_GPIO_IRQn[xGpio]), 0x02); */
        /*  HAL_NVIC_EnableIRQ((IRQn_Type)(aRADIO_GPIO_IRQn[xGpio]));     */
    }
}


/**
* @brief  Enables or disables the interrupt on GPIO .
* @param  xGpio Specifies the GPIO whose priority shall be changed.
*         This parameter can be one of following parameters:
*         @arg GPIO_0
*         @arg GPIO_1
*         @arg GPIO_2
*         @arg GPIO_3
* @param  nPreemption Specifies Preemption Priority.
* @param  nSubpriority Specifies Subgroup Priority.
* @param  xNewState Specifies the State.
*         This parameter can be one of following parameters:
*         @arg ENABLE: Interrupt is enabled
*         @arg DISABLE: Interrupt is disabled
* @retval None.
*/
void RadioGpioInterruptCmd(RadioGpioPin xGpio, uint8_t nPreemption, uint8_t nSubpriority, FunctionalState xNewState)
{
  HAL_NVIC_SetPriority((IRQn_Type) (aRADIO_GPIO_IRQn[xGpio]), nPreemption, nSubpriority);
  if (!xNewState)
  {
       HAL_NVIC_DisableIRQ((IRQn_Type)(aRADIO_GPIO_IRQn[xGpio]));
  }
  else
  {
        HAL_NVIC_EnableIRQ((IRQn_Type)(aRADIO_GPIO_IRQn[xGpio]));
  }
}

void GpioIRQInit(void)
{
    RadioGpioInit(RADIO_GPIO_3, RADIO_MODE_EXTI_IN); //Initialize interrupt PIN
    RadioGpioInit(RADIO_GPIO_0, RADIO_MODE_GPIO_OUT); //Set wakeup pin to output
    RadioGpioInit(RADIO_GPIO_1, RADIO_MODE_GPIO_OUT); //Set test runing pin to output
    /*RadioGpioInit(GPIO_PIN_1, RADIO_MODE_EXTI_IN);
    RadioGpioInit(GPIO_PIN_2, RADIO_MODE_EXTI_IN);
    RadioGpioInit(GPIO_PIN_3, RADIO_MODE_EXTI_IN);*/

    RadioGpioInterruptCmd(RADIO_GPIO_3, 0x04,0x04,ENABLE); //Initialize interrupt
    /*RadioGpioInterruptCmd(GPIO_PIN_1, 0x04,0x04,ENABLE);
    RadioGpioInterruptCmd(GPIO_PIN_2, 0x04,0x04,ENABLE);
    RadioGpioInterruptCmd(GPIO_PIN_3, 0x04,0x04,ENABLE);*/
}

void
SdkEvalSpiInit(void)
{
    pSpiHandle.State = HAL_SPI_STATE_RESET;
    PRINTF("SdkEvalSpiInit\n");

    if (HAL_SPI_GetState(&pSpiHandle) == HAL_SPI_STATE_RESET)
    {
        /* SPI Config */
        pSpiHandle.Instance = RADIO_SPI;
        pSpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        pSpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
        pSpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
        pSpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
        pSpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
        pSpiHandle.Init.CRCPolynomial = 7;
        pSpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
        pSpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
        pSpiHandle.Init.NSS = SPI_NSS_HARD_OUTPUT;
        pSpiHandle.Init.TIMode = SPI_TIMODE_DISABLED;
        pSpiHandle.Init.Mode = SPI_MODE_MASTER;

        HAL_SPI_MspInit(&pSpiHandle);
        HAL_SPI_Init(&pSpiHandle);
    }
}

void
HAL_SPI_MspInit(SPI_HandleTypeDef* pSpiHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    if (pSpiHandle->Instance == RADIO_SPI)
    {
        PRINTF("HAL_SPI_MspInit\n");
        /*** Configure the GPIOs ***/
        /* Enable GPIO clock */
        RADIO_SPI_SCK_CLOCK_ENABLE();
        RADIO_SPI_MISO_CLOCK_ENABLE();
        RADIO_SPI_MOSI_CLOCK_ENABLE();

        /**SPI1 GPIO Configuration */

        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;

        GPIO_InitStruct.Pin = RADIO_SPI_SCK_PIN;
        HAL_GPIO_Init(RADIO_SPI_SCK_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;

        GPIO_InitStruct.Pin = RADIO_SPI_MISO_PIN;
        HAL_GPIO_Init(RADIO_SPI_MISO_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = RADIO_SPI_MOSI_PIN;
        HAL_GPIO_Init(RADIO_SPI_MOSI_PORT, &GPIO_InitStruct);

        RADIO_SPI_CS_CLOCK_ENABLE();

        /* Configure SPI pin: CS */
        GPIO_InitStruct.Pin = RADIO_SPI_CS_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        HAL_GPIO_Init(RADIO_SPI_CS_PORT, &GPIO_InitStruct);

        RADIO_SPI_CLK_ENABLE();
    }
}

/*---------------------------------------------------------------------------*/
/* Declaration of static functions */
static int8_t dw1000_get_exti_int_status(uint32_t exti_line);
static void dw1000_select(void);
static void dw1000_deselect(void);
/*---------------------------------------------------------------------------*/
/* Set the DW1000 ISR to NULL by default */
dw1000_isr_t dw1000_isr = NULL;
/*---------------------------------------------------------------------------*/
/* DW1000 Interrupt pin handler */
// see RADIO_GPIO_3_EXTI_IRQ_HANDLER              EXTI9_5_IRQHandler

/*---------------------------------------------------------------------------*/
static int8_t
dw1000_get_exti_int_status(uint32_t exti_line)
{
  /* Check the parameters */
  assert_param(IS_GET_EXTI_LINE(exti_line));

  return (int8_t)(EXTI->IMR & exti_line);
}
/*---------------------------------------------------------------------------*/
int8_t
dw1000_disable_interrupt(void)
{
  int8_t irqn_status;

  irqn_status = dw1000_get_exti_int_status(RADIO_GPIO_3_EXTI_LINE);

  if(irqn_status != 0) {
    NVIC_DisableIRQ(RADIO_GPIO_3_EXTI_IRQN);
  }
  return irqn_status;
}
/*---------------------------------------------------------------------------*/
void
dw1000_enable_interrupt(int8_t irqn_status)
{
  if(irqn_status != 0) {
    NVIC_EnableIRQ(RADIO_GPIO_3_EXTI_IRQN);
  }
}
/*---------------------------------------------------------------------------*/
static void
dw1000_select(void)
{
	HAL_GPIO_WritePin(RADIO_SPI_CS_PORT, RADIO_SPI_CS_PIN, GPIO_PIN_RESET);
}
/*---------------------------------------------------------------------------*/
static void
dw1000_deselect(void)
{
	HAL_GPIO_WritePin(RADIO_SPI_CS_PORT, RADIO_SPI_CS_PIN, GPIO_PIN_SET);
}
/*---------------------------------------------------------------------------*/
void
dw1000_set_isr(dw1000_isr_t new_dw1000_isr)
{
  int8_t irqn_status;

  irqn_status = dw1000_get_exti_int_status(RADIO_GPIO_3_EXTI_LINE);

  /* Disable DW1000 EXT Interrupt */
  if(irqn_status) {
    dw1000_disable_interrupt();
  }
  /* Set ISR Handler */
  dw1000_isr = new_dw1000_isr;

  /* Re-enable DW1000 EXT Interrupt state */
  if(irqn_status) {
    dw1000_enable_interrupt(irqn_status);
  }
}
/*---------------------------------------------------------------------------*/
void
dw1000_spi_open(void)
{

}
/*---------------------------------------------------------------------------*/
void
dw1000_spi_close(void)
{

}
/*---------------------------------------------------------------------------*/
void
dw1000_spi_read(uint16_t hdrlen,
                const uint8_t *hdrbuf,
                uint32_t len,
                uint8_t *buf)
{
    int8_t irqn_status;
    HAL_StatusTypeDef spi_err = HAL_OK;

    /* Disable DW1000 EXT Interrupt */
    irqn_status = dw1000_disable_interrupt();

    /* Clear SPI1 Chip Select */
    dw1000_select();

    spi_err = HAL_SPI_Transmit(&pSpiHandle, hdrbuf, hdrlen, 1000);
    while (__HAL_SPI_GET_FLAG(&pSpiHandle, SPI_FLAG_TXE)
            == HAL_SPI_STATE_BUSY_TX)
        ;
    if (spi_err != HAL_OK)
    {
        PRINTF("Error while reading (writing) header\n");
    }

    spi_err = HAL_SPI_TransmitReceive(&pSpiHandle, buf, buf, len, 1000);

    while (__HAL_SPI_GET_FLAG(&pSpiHandle, SPI_FLAG_TXE)
            == HAL_SPI_STATE_BUSY_TX)
        ;
    while (__HAL_SPI_GET_FLAG(&pSpiHandle, SPI_FLAG_TXE)
            == HAL_SPI_STATE_BUSY_RX)
        ;

    if (spi_err != HAL_OK)
    {
        PRINTF("Error while reading buf\n");
    }
//  /* Write Header */
//  for(i = 0; i < hdrlen; i++) {
//    /* Send data over SPI, timeout = 1000 */
//
//    /* Wait for the RX Buffer to be filled */
//    //while (__HAL_SPI_GET_FLAG(&pSpiHandle, SPI_FLAG_TXE) == RESET);
//  }

//  /* Read body */
//  for(i = 0; i < len; i++) {
//    /* Send dummy data over SPI */
//      spi_err = HAL_SPI_Receive(&pSpiHandle, (uint8_t *)(buf + i), 1, 1000);
//      PRINTF("read reg. err = %d\n", spi_err);
//    /* Wait for the RX Buffer to be filled */
//      while (__HAL_SPI_GET_FLAG(&pSpiHandle, SPI_FLAG_TXE) == RESET);
//  }

    /* Set SPI1 Chip Select */
    dw1000_deselect();

    /* Re-enable DW1000 EXT Interrupt state */
    dw1000_enable_interrupt(irqn_status);
}
/*---------------------------------------------------------------------------*/
void
dw1000_spi_write(uint16_t hdrlen,
                 const uint8_t *hdrbuf,
                 uint32_t len,
                 const uint8_t *buf)
{
    int8_t irqn_status;
    HAL_StatusTypeDef spi_err = HAL_OK;

    /* Disable DW1000 EXT Interrupt */
    irqn_status = dw1000_disable_interrupt();

    /* Clear SPI1 Chip Select */
    dw1000_select();
//
//  /* Write Header */
//  for(i = 0; i < hdrlen; i++) {
//    /* Send data over SPI, timeout = 1000 */
//	  HAL_SPI_Transmit(&pSpiHandle, (uint8_t*)(hdrbuf + i), 1, 1000);
//
//    /* Wait for the TX Buffer to be filled */
//	while (__HAL_SPI_GET_FLAG(&pSpiHandle, SPI_FLAG_TXE) == RESET);
//  }
//
//  /* Write body */
//  for(i = 0; i < len; i++) {
//    /* Send data over SPI */
//	  HAL_SPI_Transmit(&pSpiHandle, (uint8_t*)(buf + i), 1, 1000);
//
//    /* Wait for the TX Buffer to be filled */
//	  while (__HAL_SPI_GET_FLAG(&pSpiHandle, SPI_FLAG_TXE) == RESET);
//  }

    spi_err = HAL_SPI_Transmit(&pSpiHandle, hdrbuf, hdrlen, 1000);
    while (__HAL_SPI_GET_FLAG(&pSpiHandle, SPI_FLAG_TXE)
            == HAL_SPI_STATE_BUSY_TX)
        ;

    if (spi_err != HAL_OK)
    {
        PRINTF("ERROR while writing header\n");
    }

    spi_err = HAL_SPI_Transmit(&pSpiHandle, buf, len, 1000);

    while (__HAL_SPI_GET_FLAG(&pSpiHandle, SPI_FLAG_TXE)
            == HAL_SPI_STATE_BUSY_TX)
        ;

    /* Check the communication status */
    if (spi_err != HAL_OK)
    {
        PRINTF("ERROR while writing buffer\n");
    }

    /* Set SPI1 Chip Select */
    dw1000_deselect();

    /* Re-enable DW1000 EXT Interrupt state */
    dw1000_enable_interrupt(irqn_status);
}
/*---------------------------------------------------------------------------*/
void
dw1000_set_spi_bit_rate(uint16_t brate)
{
    pSpiHandle.State = HAL_SPI_STATE_RESET;
    PRINTF("SPI prescaler set to: %d\n", brate);
    //     0 ... fast
    // other ... slow

    if (HAL_SPI_GetState(&pSpiHandle) == HAL_SPI_STATE_RESET)
    {
        pSpiHandle.Instance = RADIO_SPI;
        pSpiHandle.Init.BaudRatePrescaler = brate;
        pSpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
        pSpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
        pSpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
        pSpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
        pSpiHandle.Init.CRCPolynomial = 7;
        pSpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
        pSpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
        pSpiHandle.Init.NSS = SPI_NSS_HARD_OUTPUT;
        pSpiHandle.Init.TIMode = SPI_TIMODE_DISABLED;
        pSpiHandle.Init.Mode = SPI_MODE_MASTER;

        HAL_SPI_MspInit(&pSpiHandle);
        HAL_SPI_Init(&pSpiHandle);
    }
}
/*---------------------------------------------------------------------------*/
void
dw1000_arch_init()
{
  /* Reset and initialise DW1000.
   * For initialisation, DW1000 clocks must be temporarily set to crystal speed.
   * After initialisation SPI rate can be increased for optimum performance.
   */
  PRINTF("dw1000_arch_init\n");
  dw1000_set_spi_bit_rate(SPI_BAUDRATEPRESCALER_16);
  dw1000_arch_reset(); /* Target specific drive of RSTn line into DW1000 low for a period.*/
  if(dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
    PRINTF("dw1000_arch_init:: INIT FAILED\n");
    while(1) {
      /* If the init function fails, we stop here */
    }
  }

  PRINTF("dw1000_arch_init success\n");
  dw1000_set_spi_bit_rate(SPI_BAUDRATEPRESCALER_2); // 8MHz
}

/*---------------------------------------------------------------------------*/
void
dw1000_arch_reset()
{
	GPIO_InitTypeDef GPIO_InitStucture;
	GPIO_InitStucture.Pin = GPIO_PIN_0;
	GPIO_InitStucture.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStucture.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStucture);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	GPIO_InitStucture.Pin = GPIO_PIN_0;
	GPIO_InitStucture.Mode = GPIO_MODE_INPUT;
	GPIO_InitStucture.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStucture);
	HAL_Delay(500);
}
/*---------------------------------------------------------------------------*/

void
debug_gpio_init()
{
    // init GPIOs PA2 and PA3, PB4 and PB5
    GPIO_InitTypeDef gpio0init;
    gpio0init.Pin = GPIO_PIN_8;
    gpio0init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio0init.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio0init);

    GPIO_InitTypeDef gpio1init;
    gpio1init.Pin = GPIO_PIN_10;
    gpio1init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio1init.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio1init);

    GPIO_InitTypeDef gpio2init;
    gpio2init.Pin = GPIO_PIN_4;
    gpio2init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio2init.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio2init);

    GPIO_InitTypeDef gpio3init;
    gpio3init.Pin = GPIO_PIN_5;
    gpio3init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio3init.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio3init);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

//-----------------------------------------------------------------------------
void dw1000_load_transmitter_calibration(uint8_t channel, uint8_t prf)
{
    dwt_txconfig_t cfg;
    if (prf == DWT_PRF_16M) {
        switch (channel) {
            case 1:
            {
                cfg.PGdly = PGDELAY_CH1_PRF16;
                cfg.power = TXPOWER_CH1_PRF16;
                break;
            }
            case 2:
            {
                cfg.PGdly = PGDELAY_CH2_PRF16;
                cfg.power = TXPOWER_CH2_PRF16;
                break;
            }
            case 3:
            {
                cfg.PGdly = PGDELAY_CH3_PRF16;
                cfg.power = TXPOWER_CH3_PRF16;
                break;
            }
            case 4:
            {
                cfg.PGdly = PGDELAY_CH4_PRF16;
                cfg.power = TXPOWER_CH4_PRF16;
                break;
            }
            case 5:
            {
                cfg.PGdly = PGDELAY_CH5_PRF16;
                cfg.power = TXPOWER_CH5_PRF16;
                break;
            }
            case 7:
            {
                cfg.PGdly = PGDELAY_CH7_PRF16;
                cfg.power = TXPOWER_CH7_PRF16;
                break;
            }
            default:
                return;
        }
    } else { // if (prf == DWT_PRF_16M)
        switch (channel) {
            case 1:
            {
                cfg.PGdly = PGDELAY_CH1_PRF64;
                cfg.power = TXPOWER_CH1_PRF64;
                break;
            }
            case 2:
            {
                cfg.PGdly = PGDELAY_CH2_PRF64;
                cfg.power = TXPOWER_CH2_PRF64;
                break;
            }
            case 3:
            {
                cfg.PGdly = PGDELAY_CH3_PRF64;
                cfg.power = TXPOWER_CH3_PRF64;
                break;
            }
            case 4:
            {
                cfg.PGdly = PGDELAY_CH4_PRF64;
                cfg.power = TXPOWER_CH4_PRF64;
                break;
            }
            case 5:
            {
                cfg.PGdly = PGDELAY_CH5_PRF64;
                cfg.power = TXPOWER_CH5_PRF64;
                break;
            }
            case 7:
            {
                cfg.PGdly = PGDELAY_CH7_PRF64;
                cfg.power = TXPOWER_CH7_PRF64;
                break;
            }
            default:
                PRINTF("ERROR: Wrong channel number\n");
                return;
        }
    }
    dwt_setxtaltrim(CALIB_XTAL_TRIM);
    dwt_configuretxrf(&cfg);
}
