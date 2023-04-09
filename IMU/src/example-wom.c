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
#include <asf.h>
/* InvenSense drivers and utils */
#include "Invn/Devices/Drivers/Iam20680/Iam20680Defs.h"
#include "Invn/Devices/Drivers/Iam20680/Iam20680ExtFunc.h"
#include "Invn/Devices/Drivers/Iam20680/Iam20680Driver_HL.h"
#include "Invn/Devices/Drivers/Iam20680/Iam20680Product.h"
#include "Invn/EmbUtils/Message.h"

#include "example-wom.h"
#include "system.h"

static uint8_t fifo_data[4096] = {0};   /** Buffer to hold raw data read from FIFO **/
	
#if ((IAM20680_HP == 1) || (IAM20680_HT == 1))
/* Configure the FIFO size. Only for IAM20680-HP and IAM20680-HT. */
IAM20680_ACCEL_CONFIG2_FIFO_SIZE_t   Iam20680_Default_FIFO_Size = IAM20680_ACCEL_CONFIG2_FIFO_SIZE_4KB ;
#endif

/* Just a handy variable to handle the iam20680 object */
static inv_iam20680_t icm_device;

int SetupInvDevice(int (*read_reg)(void * context, uint8_t reg, uint8_t * buf, uint32_t len),
				   int (*write_reg)(void * context, uint8_t reg, const uint8_t * buf, uint32_t len),
				   inv_bool_t isSPI)
{
	int rc = 0;
	uint8_t who_am_i;

	/* Initialize iam20680 serif structure */
	struct inv_iam20680_serif iam20680_serif;

	iam20680_serif.context   = 0; /* no need */
	iam20680_serif.read_reg  = (*read_reg);
	iam20680_serif.write_reg = (*write_reg);
	iam20680_serif.is_spi    = isSPI;
	if(isSPI) {
		/* Init SPI communication: SPI1 - SCK(PA5) / MISO(PA6) / MOSI(PA7) / CS(PB6) */
		iam20680_serif.max_read  = 1024*32; /* maximum number of bytes allowed per serial read */
		iam20680_serif.max_write = 1024*32; /* maximum number of bytes allowed per serial write */
		INV_MSG(INV_MSG_LEVEL_INFO, "Opening serial interface through SPI");
	}else {
		/* Init I2C communication: I2C1 - SCL(PB8) / SDA(PB9) */
		iam20680_serif.max_read  = 64; /* maximum number of bytes allowed per serial read */
		iam20680_serif.max_write = 64; /* maximum number of bytes allowed per serial write */
		INV_MSG(INV_MSG_LEVEL_INFO, "Opening serial interface through I2C");
	}

	/* Reset iam20680 driver states */
	memset(&icm_device, 0, sizeof(icm_device));
	icm_device.serif = iam20680_serif;

	/* Check WHOAMI */
	rc = inv_iam20680_get_who_am_i(&icm_device, &who_am_i);
#if (SERIF_TYPE_I2C == 1)	
	if((rc != INV_ERROR_SUCCESS) || (who_am_i != EXPECTED_WHOAMI)) {
		if(!isSPI) {
			/* Check i2c bus stuck and clear the bus */			
			twi_clear_bus();
		}
	
		/* Retry who_am_i check */
		rc = inv_iam20680_get_who_am_i(&icm_device, &who_am_i);
	}
#endif

	if(rc != INV_ERROR_SUCCESS)
		return rc;

	if(who_am_i != EXPECTED_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Bad WHOAMI value. Got 0x%02x. Expected 0x%02x.", who_am_i, EXPECTED_WHOAMI);
	}

	/* Init device */
	rc = inv_iam20680_init(&icm_device);
	
#if ((IAM20680_HT==1) && (ENABLE_INT2_PIN==1))
	rc |= inv_iam20680_wr_int_pin_cfg_int2_en(&icm_device, IAM20680_INT_PIN_CFG_INT2_EN_enable);
#endif
	
	return rc;
}

int ConfigureInvDeviceForWOM(void)
{
	int rc = 0;
	/** Disable all interrupts before configuring WOM **/
	rc |= inv_iam20680_wr_int_enable_data_rdy_int_en(&icm_device,IAM20680_INT_ENABLE_DATA_RDY_INT_EN_disable);
	rc |= inv_iam20680_wr_int_enable_wom_int_en(&icm_device,0);
	
	rc |= inv_iam20680_wr_fifo_en_accel_fifo_en(&icm_device, IAM20680_FIFO_EN_ACCEL_FIFO_EN_enable);
	rc |= inv_iam20680_wr_pwr_mgmt_2_fifo_lp_en(&icm_device, IAM20680_PWR_MGMT_2_FIFO_LP_enable);
	
#if ((IAM20680_HP == 1) || (IAM20680_HT == 1))
		rc |= inv_iam20680_wr_accel_config2_FIFO_size(&icm_device, Iam20680_Default_FIFO_Size);
#endif
	
	rc = inv_iam20680_configure_accel_wom(&icm_device, 25, 8);
	/* Reset then Enable FIFO */
	rc |= inv_iam20680_wr_user_ctrl_fifo_en(&icm_device, IAM20680_USER_CTRL_FIFO_EN_disable);
	rc |= inv_iam20680_wr_user_ctrl_fifo_rst(&icm_device);
	
	rc |= inv_iam20680_wr_user_ctrl_fifo_en(&icm_device, IAM20680_USER_CTRL_FIFO_EN_enable);
	
	return rc;
}


static void ConfigurePostWoM(void)
{
	inv_iam20680_wr_int_enable_data_rdy_int_en(&icm_device,IAM20680_INT_ENABLE_DATA_RDY_INT_EN_enable);
	inv_iam20680_wr_int_enable_wom_int_en(&icm_device,0);
}

static int GetDataFromFIFO(uint8_t int_status)
{
	int rc = 0;
	int16_t raw_acc[3];
	uint16_t packet_count = 0, packet_count_i = 0,fifo_len = 0;
	uint16_t packet_size = 0;
	uint8_t fifo_count[2];
	uint8_t ignore_bytes_cnt = 2;
	uint16_t idx = 0;
	uint32_t timestamp = 0;
	
	packet_size += ACCEL_DATA_SIZE;
	/* Read Timestamp from ring-buffer */
	timestamp = timer_get_irq_timestamp_us();
	inv_iam20680_rd_fifo_count(&icm_device, fifo_count);

	fifo_len = (fifo_count[0] << 8) | fifo_count[1];
	if ( fifo_len < packet_size)
	{
		/** Full packet is not available in FIFO just reset FIFO and return **/
		inv_iam20680_wr_user_ctrl_fifo_rst(&icm_device);
		return INV_ERROR_SUCCESS;
	}

	__disable_irq();
	/** Read all FIFO data at once **/
	rc = inv_iam20680_rd_fifo(&icm_device, fifo_len, fifo_data) ;
	__enable_irq();
	if(rc != INV_ERROR_SUCCESS) {
		/* sensor data is in FIFO according to FIFO_COUNT but failed to read FIFO,
			reset FIFO and try next chance */
		inv_iam20680_wr_user_ctrl_fifo_rst(&icm_device);
		return INV_ERROR_SUCCESS;
	}
	
	
	packet_count = fifo_len/packet_size; 	/* FIFO byte mode */
	
	if ( int_status & BIT_FIFO_OVERFLOW_MASK ) 	{
		/** On FIFO over flow and FIFO mode stream, Oldest packet will be over written by new packet.
		    So start of FIFO will be having partial old sample which needs to be ignored 
			For FIFO size 512 bytes and 2048 bytes , ignore 2 bytes
			For FIFO size 1024 bytes and 4096 bytes, ignore 4 bytes **/
#if ((IAM20680_HP == 1) || (IAM20680_HT == 1))
		do {
			if ((Iam20680_Default_FIFO_Size == IAM20680_ACCEL_CONFIG2_FIFO_SIZE_1KB) || (Iam20680_Default_FIFO_Size == IAM20680_ACCEL_CONFIG2_FIFO_SIZE_4KB))
				ignore_bytes_cnt = 4;	// Ignore first 4 bytes for 1024 and 4096 byte FIFOs
			else
				ignore_bytes_cnt = 2;	// Ignore first 2 bytes for 512 and 2048 byte FIFOs
		}while(0);
#elif ( IAM20680 == 1)
		ignore_bytes_cnt = 2; // Ignore first 2 bytes for IAM20680 as FIFO size is 512 bytes
#endif
	
		idx = ignore_bytes_cnt;
	}
	
		
	/** process packet **/
	for (packet_count_i=0; packet_count_i<packet_count; packet_count_i++) {
		
		/* raw sensor data output is in big endian */
		raw_acc[0] = (fifo_data[idx + 0] << 8) | fifo_data[idx +1];
		raw_acc[1] = (fifo_data[idx + 2] << 8) | fifo_data[idx +3];
		raw_acc[2] = (fifo_data[idx + 4] << 8) | fifo_data[idx +5];
		idx += packet_size;
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: a %d, %d, %d", timestamp, raw_acc[0], raw_acc[1], raw_acc[2]);
	} // for loop - packet processing
	

	return INV_ERROR_SUCCESS;
}
void ProcessIAM20680Irq(void)
{
	int rc = 0;
	uint8_t int_status = 0;
	static uint8_t isWomRx = 0;
	
	rc = inv_iam20680_rd_int_status(&icm_device, &int_status);
	if(rc != INV_ERROR_SUCCESS)
		return ;
	
	if ( !isWomRx && (int_status & BIT_WOM_INT_MASK) )
	{
		INV_MSG(INV_MSG_LEVEL_INFO, "WOM interrupt received.");
		GetDataFromFIFO(int_status);
		/** WOM interrupt is received **/
		ConfigurePostWoM();
		isWomRx = 1;
		
		
	}else if ( int_status & (BIT_FIFO_OVERFLOW_MASK |BIT_DATA_RDY_INT_MASK))
	{
		/** FIFO over flow / Data ready interrupt **/
		GetDataFromFIFO(int_status);
	}
}