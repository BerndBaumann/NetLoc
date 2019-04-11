/**
 * \file
 *      Platform Dependent DW1000 Driver Header File
 *
 */

#ifndef DW1000_ARCH_H_
#define DW1000_ARCH_H_
/*---------------------------------------------------------------------------*/
#include "contiki.h"

#include "st-lib.h"
#include "stm32l1xx_hal.h"
/* Exported types ------------------------------------------------------------*/
/* MCU GPIO pin working mode for GPIO */
typedef enum
{
    RADIO_MODE_GPIO_IN = 0x00, /*!< Work as GPIO input */
    RADIO_MODE_EXTI_IN, /*!< Work as EXTI */
    RADIO_MODE_GPIO_OUT, /*!< Work as GPIO output */
} RadioGpioMode;

/* MCU GPIO pin enumeration for GPIO */
typedef enum
{
    RADIO_GPIO_0 = 0x00, /*!< GPIO_0 selected */
    RADIO_GPIO_1 = 0x01, /*!< GPIO_1 selected */
    RADIO_GPIO_2 = 0x02, /*!< GPIO_2 selected */
    RADIO_GPIO_3 = 0x03, /*!< GPIO_3 selected */
    RADIO_GPIO_SDN = 0x04, /*!< GPIO_SDN selected */
} RadioGpioPin;

#define RADIO_GPIO_IRQ      RADIO_GPIO_3
/* SPI1 */
#define RADIO_SPI                                 SPI1
#define RADIO_SPI_CLK_ENABLE()                  __SPI1_CLK_ENABLE()
#define RADIO_SPI_CLK_DISABLE()                 __SPI1_CLK_DISABLE()

#define RADIO_SPI_MISO_PORT                      GPIOA
#define RADIO_SPI_MISO_PIN                       GPIO_PIN_6
#define RADIO_SPI_MISO_CLOCK_ENABLE()            __GPIOA_CLK_ENABLE()
#define RADIO_SPI_MISO_CLOCK_DISABLE()           __GPIOA_CLK_DISABLE()

#define RADIO_SPI_MOSI_PORT                      GPIOA
#define RADIO_SPI_MOSI_PIN                       GPIO_PIN_7
#define RADIO_SPI_MOSI_CLOCK_ENABLE()            __GPIOA_CLK_ENABLE()
#define RADIO_SPI_MOSI_CLOCK_DISABLE()           __GPIOA_CLK_DISABLE()

#define RADIO_SPI_SCK_PORT                      GPIOA
#define RADIO_SPI_SCK_PIN                       GPIO_PIN_5
#define RADIO_SPI_SCK_CLOCK_ENABLE()            __GPIOA_CLK_ENABLE()
#define RADIO_SPI_SCK_CLOCK_DISABLE()           __GPIOA_CLK_DISABLE()

// if using SPI-CS2
//#define RADIO_SPI_CS_PORT                        GPIOA
//#define RADIO_SPI_CS_PIN                         GPIO_PIN_1
// if using SPI-CS1
#define RADIO_SPI_CS_PORT                        GPIOB
#define RADIO_SPI_CS_PIN                         GPIO_PIN_6
#define RADIO_SPI_CS_CLOCK_ENABLE()            __GPIOB_CLK_ENABLE()
#define RADIO_SPI_CS_CLOCK_DISABLE()           __GPIOB_CLK_DISABLE()

/*---------------------------------------------------------------------------*/
#define DW1000_SPI_OPEN_ERROR  0
#define DW1000_SPI_OPEN_OK     1
/*---------------------------------------------------------------------------*/
#define DW1000_SPI_SLOW        SPI_BaudRatePrescaler_16 /* 2 MHz */
#define DW1000_SPI_FAST        SPI_BaudRatePrescaler_2 /* 16 MHz */
/*---------------------------------------------------------------------------*/
/* DW1000 IRQ (EXTI9_5_IRQ) handler type. */
typedef void (*dw1000_isr_t)(void);
/* DW1000 IRQ handler declaration. */
extern dw1000_isr_t dw1000_isr;
/* Function to set a new DW1000 EXTI ISR handler */ 
void dw1000_set_isr(dw1000_isr_t new_dw1000_isr);
/*---------------------------------------------------------------------------*/
void dw1000_arch_init();
void dw1000_arch_reset();
void dw1000_spi_open(void);
void dw1000_spi_close(void);
void dw1000_spi_read(uint16_t hdrlen, const uint8_t *hdrbuf, uint32_t len, uint8_t *buf);
void dw1000_spi_write(uint16_t hdrlen, const uint8_t *hdrbuf, uint32_t len, const uint8_t *buf);
void dw1000_set_spi_bit_rate(uint16_t brate);
int8_t dw1000_disable_interrupt(void);
void dw1000_enable_interrupt(int8_t irqn_status);
void dw1000_load_transmitter_calibration(uint8_t channel, uint8_t prf);

#define CALIB_XTAL_TRIM 0x16

#define PGDELAY_CH1_PRF16 0xc9
#define PGDELAY_CH2_PRF16 0xd4
#define PGDELAY_CH3_PRF16 0xd1
#define PGDELAY_CH4_PRF16 0x95
#define PGDELAY_CH5_PRF16 0xd2
#define PGDELAY_CH7_PRF16 0x93

#define PGDELAY_CH1_PRF64 0xd0
#define PGDELAY_CH2_PRF64 0xd4
#define PGDELAY_CH3_PRF64 0xd1
#define PGDELAY_CH4_PRF64 0x95
#define PGDELAY_CH5_PRF64 0xd2
#define PGDELAY_CH7_PRF64 0x93

#define TXPOWER_CH1_PRF16 0x75757575
#define TXPOWER_CH2_PRF16 0x73737373
#define TXPOWER_CH3_PRF16 0x72727272
#define TXPOWER_CH4_PRF16 0x38383838
#define TXPOWER_CH5_PRF16 0x46464646
#define TXPOWER_CH7_PRF16 0x9a9a9a9a

#define TXPOWER_CH1_PRF64 0x67676767
#define TXPOWER_CH2_PRF64 0x67676767
#define TXPOWER_CH3_PRF64 0x8c8c8c8c
#define TXPOWER_CH4_PRF64 0x97979797
#define TXPOWER_CH5_PRF64 0x83838383
#define TXPOWER_CH7_PRF64 0xd8d8d8d8

/*---------------------------------------------------------------------------*/
/* Platform-specific bindings for the DW1000 driver */
#define writetospi(cnt, header, length, buffer) dw1000_spi_write(cnt, header, length, buffer)
#define readfromspi(cnt, header, length, buffer) dw1000_spi_read(cnt, header, length, buffer)
#define decamutexon() dw1000_disable_interrupt()
#define decamutexoff(stat) dw1000_enable_interrupt(stat)
#define deca_sleep(t) clock_wait(t)
/*---------------------------------------------------------------------------*/
#endif /* DW1000_ARCH_H_ */
