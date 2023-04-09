/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "Invn/Devices/Drivers/Iam20680/Iam20680Product.h"
#include "example_config.h"

/******************************************************************************/
/* Begin Example Configuration - User Configurable                            */
/******************************************************************************/
/* Select communication between Atmel and INV device by setting 0/1 to one of the following defines */
#define SERIF_TYPE_SPI              0
#define SERIF_TYPE_I2C              1  /* For I2C, configure J2 pin [1==2(closed), 3==4(closed)] on TDK SmartMotion Board */

/* Select whether to use onboard/inbuilt sensor or external DB/EVB */
#define TDK_BOARD_INBUILT_SENSOR    0  /* Set 0 for external, 1 for in-built sensor on TDK SmartMotion Board */

/* Select board version for TDK SmartMotion board */
#define TDK_BOARD_REVISION          1  /* Set 0 for TDK SmartMotion RevA, 1 for TDK SmartMotion RevB/B+/C */
                                       /* Configure J7 pins closed on TDK SmartMotion Board RevB/B+/C with external sensor board for SPI configuration */
									   
#define ENABLE_INT2_PIN             1  /* Set 1 to enable INT2 so that all interrupts except for data ready appear on the INT2 pin, and data ready interrupt appears on the INT interrupt pin. */
                                       /* Set 0 to disable INT2 so that all of the interrupts appear on the INT pin, and INT2 interrupt pin is unused. */

/* Configure serial port for host-mcu communication */
#define CONFIG_UART_DBG_PORT        1  /* Primary port for data in raw_accel_gyro, WOM, SelfTest & secondary port for debugging in simple-cli applications */
#define CONFIG_UART_CMD_PORT        1  /* Primary port for command handling in simple-cli application */

/******************************************************************************/
/* End Example Configuration - User Configurable                              */
/******************************************************************************/
#define USART_TX_DMA_XFER_EN		0
#define BUFFER_SIZE					256

/* SysTick timer period in microseconds */
#define SYSTICK_TIMER_PERIOD_US		10

/* Macros and defines */

/* Rev B/B+/C with DB/EVB */
#if ((TDK_BOARD_INBUILT_SENSOR == 0) && (TDK_BOARD_REVISION==1))

 /** Chip select. */
 #define CHIP_SELECT			0		/* Ensure J7 closed on TDK SmartMotion Board RevB with external sensor board */

 /** External interrupt pin definition */
 
 /* DB/EVB INT1 */
 #define PIN_EXT_INTERRUPT       {PIO_PA30, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT | PIO_IT_RISE_EDGE}
 #define PIN_EXT_INTERRUPT_MASK  PIO_PA30
 #define PIN_EXT_INTERRUPT_PIO   PIOA
 #define PIN_EXT_INTERRUPT_ID    ID_PIOA
 #define PIN_EXT_INTERRUPT_TYPE  PIO_INPUT
 #define PIN_EXT_INTERRUPT_ATTR  (PIO_DEFAULT | PIO_IT_RISE_EDGE)
 #define PIN_EXT_INTERRUPT_IRQn  PIOA_IRQn

 /* DB/EVB INT2 */
 #if ((IAM20680_HT==1) && (ENABLE_INT2_PIN==1))
 #define PIN_EXT_INTERRUPT2       {PIO_PA20, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT | PIO_IT_RISE_EDGE}
 #define PIN_EXT_INTERRUPT2_MASK  PIO_PA20
 #define PIN_EXT_INTERRUPT2_PIO   PIOA
 #define PIN_EXT_INTERRUPT2_ID    ID_PIOA
 #define PIN_EXT_INTERRUPT2_TYPE  PIO_INPUT
 #define PIN_EXT_INTERRUPT2_ATTR  (PIO_DEFAULT | PIO_IT_RISE_EDGE)
 #define PIN_EXT_INTERRUPT2_IRQn  PIOA_IRQn
 #endif

#else /* Rev B/B+/C with on-board sensor or Rev A */

 /** Chip select. */
 #define CHIP_SELECT			1

 /** External interrupt pin definition */
 
 /* Onboard sensor INT1 */
 #define PIN_EXT_INTERRUPT       {PIO_PB3, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT | PIO_IT_RISE_EDGE}
 #define PIN_EXT_INTERRUPT_MASK  PIO_PB3
 #define PIN_EXT_INTERRUPT_PIO   PIOB
 #define PIN_EXT_INTERRUPT_ID    ID_PIOB
 #define PIN_EXT_INTERRUPT_TYPE  PIO_INPUT
 #define PIN_EXT_INTERRUPT_ATTR  (PIO_DEFAULT | PIO_IT_RISE_EDGE)
 #define PIN_EXT_INTERRUPT_IRQn  PIOB_IRQn

 /* Onboard sensor INT2 */
 #if ((IAM20680_HT==1) && (ENABLE_INT2_PIN==1))
 #define PIN_EXT_INTERRUPT2       {PIO_PB15, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT | PIO_IT_RISE_EDGE}
 #define PIN_EXT_INTERRUPT2_MASK  PIO_PB15
 #define PIN_EXT_INTERRUPT2_PIO   PIOB
 #define PIN_EXT_INTERRUPT2_ID    ID_PIOB
 #define PIN_EXT_INTERRUPT2_TYPE  PIO_INPUT
 #define PIN_EXT_INTERRUPT2_ATTR  (PIO_DEFAULT | PIO_IT_RISE_EDGE)
 #define PIN_EXT_INTERRUPT2_IRQn  PIOB_IRQn
 #endif 

#endif

#if (CONFIG_UART_CMD_PORT == 1)
 #define CONSOLE_UART_IRQn           FLEXCOM0_IRQn		/* Configuration for console uart IRQ for command handling */
 #define console_uart_irq_handler    FLEXCOM0_Handler	/* Configuration for console uart IRQ handler for command handling */
#endif

#if (SERIF_TYPE_SPI == 1)
 #define SPI_Handler				FLEXCOM5_Handler
 #define SPI_IRQn					FLEXCOM5_IRQn
 #define SPI_CHIP_SEL				CHIP_SELECT	/* Chip select. */
 #define SPI_CHIP_PCS				spi_get_pcs(SPI_CHIP_SEL)
 #define SPI_CLK_POLARITY			0		/* Clock polarity. */
 #define SPI_CLK_PHASE				1		/* Clock phase. */
 #define SPI_DLYBS					0x40	/* Delay before SPCK. */
 #define SPI_DLYBCT					0x10	/* Delay between consecutive transfers. */
 #define SPI_CLK_SPEED				6000000 /* SPI clock setting (Hz). */
 #define READ_BIT_MASK				0x80
 #define WRITE_BIT_MASK				0x7F
#endif

#if (SERIF_TYPE_I2C == 1)
 #define IAM_I2C_ADDR_AD0_LOW		0x68	/* Default I2C slave address for IAM20680 board */
 #define IAM_I2C_ADDR_AD0_HIGH		0x69

/* GPIO PA15 pin (INT DB) has been used to recover I2C line stuck.
 * Jumper connection is required between EXT1 PA14 and CN2 Pin1 (INT DB))
 */
 #define IAM_I2C_ADHOC_CLK_GPIO				(PIO_PA15_IDX)
 #define IAM_I2C_ADHOC_CLK_FLAGS			(PIO_OUTPUT_1 | PIO_DEFAULT)
 #define IAM_I2C_ADHOC_CLK_PIN				IOPORT_CREATE_PIN(PIOA, 15)
 #define IAM_I2C_ADHOC_CLK_ACTIVE_LEVEL		false
 #define IAM_I2C_ADHOC_CLK_INACTIVE_LEVEL	!TEST_ACTIVE_LEVEL
#endif

/* Function prototype declarations */

void clear_irq(void);
void configure_console(void);
void external_interrupt_initialize(void (*handler_function)(void));
void timer_create_ringbuffer_timestamp(uint32_t value);
int32_t timer_clear_irq_timestamp(void);
uint32_t timer_get_irq_timestamp_us(void);

#if (SERIF_TYPE_SPI == 1)
 void spi_master_initialize(void);
 int spi_master_read_register(void * context, uint8_t register_addr, uint8_t * value, uint32_t len);
 int spi_master_write_register(void * context, uint8_t register_addr, const uint8_t * value, uint32_t len);
#endif

#if (SERIF_TYPE_I2C == 1)
 void i2c_master_initialize(void);
 unsigned long i2c_master_read_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue);
 unsigned long i2c_master_write_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue);
 uint32_t twi_clear_bus(void);
#endif

#endif /* !_SYSTEM_H_ */
