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
/* Atmel SAMG55 */
//#include <asf.h>

/* InvenSense drivers and utils */
#include "Invn/EmbUtils/InvScheduler.h"
#include "Invn/EmbUtils/RingBuffer.h"
#include "Invn/EmbUtils/RingByteBuffer.h"
#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/ErrorHelper.h"
#include "Invn/Devices/Drivers/Iam20680/Iam20680Product.h"

/* System/Board Drivers */
#include "main-selftest.h"
#include "example-selftest.h"
#include "system.h"

/* eMD firmware release version */
#include "eMDReleaseVersion.h"

/* Sanity checks */
#if (SERIF_TYPE_SPI && SERIF_TYPE_I2C) || (!SERIF_TYPE_SPI && !SERIF_TYPE_I2C)
	#error "You must choose between I2C or SPI for iam20680 device control"
#endif

#if ( IAM20680 == 1)
uint8_t display_banner[] = "     IAM20680 Self Test example       " ;
#elif (IAM20680_HP == 1)
uint8_t display_banner[] = "     IAM20680HP Self Test example     " ;
#elif (IAM20680_HT == 1)
uint8_t display_banner[] = "     IAM20680HT Self Test example     " ;
#endif

/* Externs and Globals */
extern InvScheduler 		scheduler;
extern InvSchedulerTask		blinkerLedTask;
extern uint8_t				I2C_Address;

int main (void) {
	
	int rc = 0;
	inv_bool_t isSPI = SERIF_TYPE_SPI;
	
	/* Hardware Initialization */
	sysclk_init();
	board_init(); 

	/* Configure Timer - 10us resolution */
	SysTick_Config(sysclk_get_cpu_hz() / (1000000 / SYSTICK_TIMER_PERIOD_US) );

	/* Initialize serial interface */
	configure_console();

	/* Setup message facility to see internal traces from FW */
	INV_MSG_SETUP(INV_MSG_ENABLE, msg_printer);
	
	INV_MSG(INV_MSG_LEVEL_INFO, "######################################");
	INV_MSG(INV_MSG_LEVEL_INFO, "%s",display_banner);
	INV_MSG(INV_MSG_LEVEL_INFO, "     Ver: %s", EMD_RELEASE_VERSION_STRING);
	INV_MSG(INV_MSG_LEVEL_INFO, "######################################");
	
	/* Initialize SPI/I2C Interface */ 
	interface_initialize();	
	
	rc = SetupInvDevice(idd_io_hal_read_reg, idd_io_hal_write_reg, isSPI);
	check_rc(rc, "error while setting up device");

	/* Perform Self-Test */
	RunSelfTest();

	rc = RunFactoryCalib();
	if ( !rc )
		ApplyOffset(0,0); // apply offset for accel and gyro LN mode

	InvScheduler_init(&scheduler);
	InvScheduler_initTask(&scheduler, &blinkerLedTask, "blinkerLedTask", inv_blinkerLedTask_handler, 0, INVSCHEDULER_TASK_PRIO_MIN+1, 1000000/SYSTICK_TIMER_PERIOD_US);
	InvScheduler_startTask(&blinkerLedTask, 0);

	while (1) {
		InvScheduler_dispatchTasks(&scheduler);
	}

	return 0;
}

static int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen) {
	(void)context;
	
#if (SERIF_TYPE_SPI == 1)
	return spi_master_read_register(NULL, reg, rbuffer, rlen);
#elif (SERIF_TYPE_I2C == 1)
	return i2c_master_read_register(I2C_Address, reg, rlen, rbuffer);
#endif
}

static int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) {
	(void)context;
	
#if (SERIF_TYPE_SPI == 1)
	return spi_master_write_register(NULL, reg, wbuffer, wlen);
#elif (SERIF_TYPE_I2C == 1)
	return i2c_master_write_register(I2C_Address, reg, wlen, wbuffer);
#endif
}

static void interface_initialize(void) {
	
#if (SERIF_TYPE_SPI == 1)
	
	spi_master_initialize();
	
#elif (SERIF_TYPE_I2C == 1)
	/* 	 
	 * Default IAM20680 (Slave) Address = 0x68 (pin AD0 is logic low)
	 * Change Slave Address to 0x69 (pin AD0 is logic high) 
	 */
#if ((TDK_BOARD_INBUILT_SENSOR == 1) && (TDK_BOARD_REVISION == 1))
	I2C_Address = IAM_I2C_ADDR_AD0_HIGH;
#endif

	i2c_master_initialize();
#endif
}

/*
 * BlinkerLedTaskMain - Task that blinks the LED.
 */
static void inv_blinkerLedTask_handler(void * arg) {

	(void)arg;

//	ioport_toggle_pin_level(LED_0_PIN);	// toggle the LED
}

/* Helper function to check RC value and block program execution */
static void check_rc(int rc, const char * msg_context) {
	
	if(rc < 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "%s: error %d (%s)", msg_context, rc, inv_error_str(rc));
		while(1);
	}
}

/*
 * Printer function for message facility
 */
static void msg_printer(int level, const char * str, va_list ap) {
#ifdef INV_MSG_ENABLE
	static char out_str[BUFFER_SIZE]; /* static to limit stack usage */
	unsigned idx = 0;
	const char * ptr = out_str;
	const char * s[INV_MSG_LEVEL_MAX] = {
		"",    // INV_MSG_LEVEL_OFF
		"[E] ", // INV_MSG_LEVEL_ERROR
		"[W] ", // INV_MSG_LEVEL_WARNING
		"[I] ", // INV_MSG_LEVEL_INFO
		"[V] ", // INV_MSG_LEVEL_VERBOSE
		"[D] ", // INV_MSG_LEVEL_DEBUG
	};
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
	if(idx >= (sizeof(out_str)))
		return;
	idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
	if(idx >= (sizeof(out_str)))
		return;
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
	if(idx >= (sizeof(out_str)))
		return;

	while(*ptr != '\0') {
		usart_serial_putchar(CONF_UART_DBG, *ptr);
		++ptr;
	}
#else
	(void)level, (void)str, (void)ap;
#endif
}

/* Sleep implementation for IAM20680 */
void inv_iam20680_sleep_us(int us) {
	
	delay_us(us);
}

/* Sleep implementation for IAM20680 */
void inv_iam20680_sleep_ms(int ms) {
	
	delay_us(ms);
}
