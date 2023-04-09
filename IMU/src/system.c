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
//#include <asf.h>
#include <stdio.h>

#include "conf_uart_serial.h"
#include "conf_board.h"
#include "system.h"

#include "ASF/sam/drivers/pio/pio.h"
#include "ASF/sam/drivers/pio/pio_handler.h"
#include "ASF/sam/drivers/twi/twi.h"
#include "ASF/sam/drivers/tc/tc.h"

/* InvenSense Drivers and Utils */
#include "Invn/EmbUtils/InvScheduler.h"
#include "Invn/EmbUtils/RingByteBuffer.h"
#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/RingBuffer.h"

/* Externs and Globals */
#if (SERIF_TYPE_I2C == 1)
 uint8_t I2C_Address = IAM_I2C_ADDR_AD0_LOW; /* Default IAM20680 (Slave) Address */
 twi_options_t opt;
 twi_packet_t packet_tx, packet_rx;
#endif

#if (CONFIG_UART_CMD_PORT == 1)
RingByteBuffer uart_rx_rb;
static uint8_t uart_rx_rb_buffer[2*BUFFER_SIZE];
#endif

static RINGBUFFER(timestamp_buffer, BUFFER_SIZE, uint32_t);

volatile uint32_t	timer_ticks = 0;
InvScheduler 		scheduler;
InvSchedulerTask	blinkerLedTask;

/* Pdc transfer buffer */
uint8_t g_uc_pdc_buffer[BUFFER_SIZE];

/* PDC data packet for transfer */
pdc_packet_t g_pdc_usart_packet;

/* Pointer to USART PDC register base */
Pdc *g_p_usart_pdc;

void external_interrupt_initialize(void (*handler_function)(void)) {

	/* Enable the peripheral clock for the board interrupt pin. */
	pmc_enable_periph_clk(PIN_EXT_INTERRUPT_ID);

	/* Configure PIOs as input pins. */
	pio_configure(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_TYPE, PIN_EXT_INTERRUPT_MASK, PIN_EXT_INTERRUPT_ATTR);

	/* Initialize PIO interrupt handler, interrupt on rising edge. */
	pio_handler_set(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_ID, PIN_EXT_INTERRUPT_MASK,
						PIN_EXT_INTERRUPT_ATTR, (void (*) (uint32_t, uint32_t))handler_function);

	/* Initialize and enable push button (PIO) interrupt. */
	pio_handler_set_priority(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_IRQn, 0);
	pio_enable_interrupt(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_MASK);
	
#if ((IAM20680_HT==1) && (ENABLE_INT2_PIN==1))
	/* Enable the peripheral clock for the board interrupt pin. */
	pmc_enable_periph_clk(PIN_EXT_INTERRUPT2_ID);

	/* Configure PIOs as input pins. */
	pio_configure(PIN_EXT_INTERRUPT2_PIO, PIN_EXT_INTERRUPT2_TYPE, PIN_EXT_INTERRUPT2_MASK, PIN_EXT_INTERRUPT2_ATTR);

	/* Initialize PIO interrupt handler, interrupt on rising edge. */
	pio_handler_set(PIN_EXT_INTERRUPT2_PIO, PIN_EXT_INTERRUPT2_ID, PIN_EXT_INTERRUPT2_MASK,
	PIN_EXT_INTERRUPT2_ATTR, (void (*) (uint32_t, uint32_t))handler_function);

	/* Initialize and enable push button (PIO) interrupt. */
	pio_handler_set_priority(PIN_EXT_INTERRUPT2_PIO, PIN_EXT_INTERRUPT2_IRQn, 0);
	pio_enable_interrupt(PIN_EXT_INTERRUPT2_PIO, PIN_EXT_INTERRUPT2_MASK);
#endif	
}

void clear_irq(void) {

	/* Clear pio interrupt */
	pio_clear(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_MASK);
#if ((IAM20680_HT==1) && (ENABLE_INT2_PIN==1))
	pio_clear(PIN_EXT_INTERRUPT2_PIO, PIN_EXT_INTERRUPT2_MASK);
#endif
}

void configure_console(void) {

#if (CONFIG_UART_CMD_PORT == 1)
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART for command handling */
	sysclk_enable_peripheral_clock(CONF_UART_ID);
	usart_serial_init(CONF_UART, (usart_serial_options_t *)&uart_serial_options);

	RingByteBuffer_init(&uart_rx_rb, uart_rx_rb_buffer, sizeof(uart_rx_rb_buffer));

	/* Enable UART IRQ */
	usart_enable_interrupt(CONSOLE_UART, US_IER_RXRDY);

	/* Enable UART interrupt */
	NVIC_SetPriority(CONSOLE_UART_IRQn, 0);

	/* Enable UART interrupt */
	NVIC_EnableIRQ(CONSOLE_UART_IRQn);
#endif

#if (CONFIG_UART_DBG_PORT == 1)
	const usart_serial_options_t uart_serial_options_dbg = {
		.baudrate = DEBUG_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART for debugging */
	sysclk_enable_peripheral_clock(CONF_UART_ID_DBG);
	stdio_serial_init(CONF_UART_DBG, &uart_serial_options_dbg);

	/* Get pointer to USART PDC register base */
	g_p_usart_pdc = usart_get_pdc_base(DEBUG_UART);

	/* Initialize PDC data packet for transfer */
	g_pdc_usart_packet.ul_addr = (uint32_t) g_uc_pdc_buffer;
	g_pdc_usart_packet.ul_size = BUFFER_SIZE;

	/* Enable PDC transfers */
	pdc_enable_transfer(g_p_usart_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);
#endif
}

#if (CONFIG_UART_CMD_PORT == 1)
/**
 * \brief Interrupt handler for USART interrupt.
 */
void console_uart_irq_handler(void) {

	uint32_t ul_status;

	/* Read USART Status. */
	ul_status = usart_get_status(CONSOLE_UART);

	if((ul_status &  US_CSR_RXRDY ))
	{
		uint8_t rxbyte;
		usart_serial_getchar(CONSOLE_UART, &rxbyte);
		if(!RingByteBuffer_isFull(&uart_rx_rb))
			RingByteBuffer_pushByte(&uart_rx_rb, rxbyte);
	}
}
#endif

#if (SERIF_TYPE_I2C == 1)
void i2c_master_initialize(void) {

	/* Insert application code here, after the board has been initialized. */
	/* Enable the peripheral and set TWI mode. */
	flexcom_enable(BOARD_FLEXCOM_TWI);
	flexcom_set_opmode(BOARD_FLEXCOM_TWI, FLEXCOM_TWI);

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
	opt.speed = TWI_CLK;

	/* Release the SDA line if board-reset has caused the SDA line to get locked (pulled low)
	 * by the slave in the previous session */
	twi_clear_bus();

	twi_master_init(BOARD_BASE_TWI, &opt);
}

unsigned long i2c_master_read_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue) {

	twi_packet_t packet_read;

	packet_read.chip = Address;
	packet_read.addr[0] = RegisterAddr;
	packet_read.addr_length = 1;
	packet_read.buffer = RegisterValue;
	packet_read.length = RegisterLen;

	if(twi_master_read((Twi*)BOARD_BASE_TWI, &packet_read) == TWI_SUCCESS){
		return TWI_SUCCESS;
	}
	return TWI_BUSY;
}

unsigned long i2c_master_write_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue) {

	twi_packet_t packet_write;

	packet_write.chip = Address;
	packet_write.addr[0] = RegisterAddr;
	packet_write.addr_length = 1;
	packet_write.buffer = (void *)RegisterValue;
	packet_write.length = RegisterLen;

	return twi_master_write((Twi*)BOARD_BASE_TWI, &packet_write);
}

/* Sometimes board reset may cause the I2C lines to get locked by the slave (line-lock state: SDA=Low & SCL=High)
 * The below module generates CLK for the slave to release the SDA line (recovered state: SDA=High & SCL=High)
 *
 * GPIO CN5:pin2 [PA20] has been used to recover the I2C SDA line.
 * Jumper connection required between EXT1-pin18 [PA14] and CN5-pin2
 */
uint32_t twi_clear_bus(void) {

	Twi *p_twi = (Twi*)BOARD_BASE_TWI;
	uint32_t result = TWI_SUCCESS;

	if ((p_twi->TWI_SR & TWI_SR_SDA) == 0) {
		ioport_set_pin_dir(IAM_I2C_ADHOC_CLK_PIN, IOPORT_DIR_OUTPUT);
		for (int i = 0; i < 10; i++) {
			ioport_set_pin_level(IAM_I2C_ADHOC_CLK_PIN, IOPORT_PIN_LEVEL_LOW);
			delay_us(10);
			ioport_set_pin_level(IAM_I2C_ADHOC_CLK_PIN, IOPORT_PIN_LEVEL_HIGH);
			delay_us(10);
		}
		ioport_set_pin_dir(IAM_I2C_ADHOC_CLK_PIN, IOPORT_DIR_INPUT);
		delay_ms(10);

		result = ((p_twi->TWI_SR & TWI_SR_SDA)>>25 == 1) ? TWI_SUCCESS : -1;
	}

	return result;
}
#endif

#if (SERIF_TYPE_SPI == 1)
/* Function prototype declaration */
/**
 * \brief Initialize SPI as master.
 */
void spi_master_initialize(void) {

	/* Enable the peripheral and set SPI mode. */
	flexcom_enable(BOARD_FLEXCOM_SPI);
	flexcom_set_opmode(BOARD_FLEXCOM_SPI, FLEXCOM_SPI);

	spi_disable(SPI_MASTER_BASE);
	spi_reset(SPI_MASTER_BASE);
	spi_set_lastxfer(SPI_MASTER_BASE);
	spi_set_master_mode(SPI_MASTER_BASE);
	spi_disable_mode_fault_detect(SPI_MASTER_BASE);

	spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, SPI_CHIP_SEL);

	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL, (sysclk_get_peripheral_hz() / SPI_CLK_SPEED));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_DLYBS, SPI_DLYBCT);

	spi_enable(SPI_MASTER_BASE);
}

int spi_master_write_register(void * context, uint8_t register_addr, const uint8_t * value, uint32_t len) {

	uint8_t reg	= register_addr;
	const uint8_t *p_rbuf = value;
	uint32_t rsize = len;
	uint32_t i;
	uint8_t uc_pcs = 0;
	uint16_t data = 0;

	delay_us(1);
	reg &= WRITE_BIT_MASK;

	spi_write(SPI_MASTER_BASE, reg, 0, 0); /* write cmd/reg-addr */
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done */
	spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* dummy read */

	for (i = 0; i < rsize; i++) {
		spi_write(SPI_MASTER_BASE, p_rbuf[i], 0, 0); /* dummy write to generate clock */
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done. */
		spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* read actual register data */
	}

	return 0;
}

int spi_master_read_register(void * context, uint8_t register_addr, uint8_t * value, uint32_t len) {

	uint8_t reg	= register_addr;
	uint8_t *p_rbuf	= value;
	uint32_t rsize	= len;
	uint32_t i;
	uint8_t uc_pcs = 0;
	uint16_t data = 0;

	delay_us(1);
	reg |= READ_BIT_MASK;

	spi_write(SPI_MASTER_BASE, reg, 0, 0); /* write cmd/reg-addr */
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done */
	spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* dummy read */

	for (i = 0; i < rsize; i++) {
		spi_write(SPI_MASTER_BASE, 0x0, 0, 0); /* dummy write to generate clock */
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done. */
		spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* read actual register data */
		p_rbuf[i] = (uint8_t)(data & 0xFF);
	}

	return 0;
}
#endif

void SysTick_Handler(void) {

	timer_ticks += SYSTICK_TIMER_PERIOD_US;

	/* Update Scheduler Time */
	InvScheduler_updateTime(&scheduler);
}

int32_t timer_clear_irq_timestamp() {

	/* Clear the buffer used to store the timestamp catched on interrupt */
	RINGBUFFER_CLEAR(&timestamp_buffer);
	timer_ticks = 0;

	return 0;
}

void timer_create_ringbuffer_timestamp(uint32_t value) {

	if(!RINGBUFFER_FULL(&timestamp_buffer))
		RINGBUFFER_PUSH(&timestamp_buffer, &value);
}

/*
 * Accurate timestamp implementation for all 20680 data
 */
uint32_t timer_get_irq_timestamp_us(void) {

	uint64_t timestamp = 0;

	if(!RINGBUFFER_EMPTY(&timestamp_buffer))
		RINGBUFFER_POP(&timestamp_buffer, &timestamp);

	return timestamp;
}
