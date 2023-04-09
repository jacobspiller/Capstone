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

/* InvenSense drivers and utils */
#include "Invn/Devices/Drivers/Iam20680/Iam20680Defs.h"
#include "Invn/Devices/Drivers/Iam20680/Iam20680ExtFunc.h"
#include "Invn/Devices/Drivers/Iam20680/Iam20680Driver_HL.h"
#include "Invn/Devices/Drivers/Iam20680/Iam20680Product.h"

#include "Invn/EmbUtils/Message.h"

#include "example-raw-ag.h"
#include "system.h"

//extern void Error_Handler(void);

struct MultipleVariables{
	float a_x;
	float a_y;
	float a_z;
	float g_x;
	float g_y;
	float g_z;
};

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
		//INV_MSG(INV_MSG_LEVEL_INFO, "Opening serial interface through SPI");
	}else {
		/* Init I2C communication: I2C1 - SCL(PB8) / SDA(PB9) */
		iam20680_serif.max_read  = 64; /* maximum number of bytes allowed per serial read */
		iam20680_serif.max_write = 64; /* maximum number of bytes allowed per serial write */
		//INV_MSG(INV_MSG_LEVEL_INFO, "Opening serial interface through I2C");
	}

	/* Reset iam20680 driver states */
	memset(&icm_device, 0, sizeof(icm_device));
	icm_device.serif = iam20680_serif;

	/* Check WHOAMI */
	rc = inv_iam20680_get_who_am_i(&icm_device, &who_am_i);
	#if (SERIF_TYPE_I2C == 1)
	if((rc != INV_ERROR_SUCCESS) || (who_am_i != 0x75)) {
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

	if(who_am_i != 0x75) {
		//INV_MSG(INV_MSG_LEVEL_ERROR, "Bad WHOAMI value. Got 0x%02x. Expected 0x%02x.", who_am_i, EXPECTED_WHOAMI);
	}

	/* Initialize device */
	rc = inv_iam20680_init(&icm_device);

	return rc;
}

int ConfigureInvDevice(inv_dev_config_t* device_config)
{
	int rc = 0;

	if (device_config->set_low_noise_mode)	{
		rc = inv_iam20680_set_gyro_low_noise_mode(&icm_device);
		rc |= inv_iam20680_set_accel_low_noise_mode(&icm_device);
	}
	else {
	  	rc = inv_iam20680_set_gyro_low_power_mode(&icm_device);
		if(device_config->enable_accel) {
			/* Accelerometer always works in low-noise mode when gyro is on */
			if(device_config->enable_gyro)
				rc |= inv_iam20680_set_accel_low_noise_mode(&icm_device);
			else {
				/* Accel Lp Mode configuration */
				rc |= inv_iam20680_wr_pwr_mgmt_2_fifo_lp_en(&icm_device, IAM20680_PWR_MGMT_2_FIFO_LP_enable);
				rc |= inv_iam20680_wr_pwr_mgmt_2_gyro_stby(&icm_device, (uint8_t)IAM20680_PWR_MGMT_2_XG_disable | (uint8_t)IAM20680_PWR_MGMT_2_YG_disable | (uint8_t)IAM20680_PWR_MGMT_2_ZG_disable);
			}
		}
	}

	inv_iam20680_sleep_us(2000);

	/* Set FCHOICE_B and ACCEL_FCHOICE_B to 0 since we're not supporting anything beyond 1kHz */
	rc |= inv_iam20680_wr_gyro_config_fchoice_b(&icm_device, 0);
	rc |= inv_iam20680_wr_accel_config2_accel_fchoice_b(&icm_device, 0);

	if (device_config->set_low_noise_mode) {
		/* Set DLPF_CFG */
		rc |= inv_iam20680_wr_config_dlpf_cfg(&icm_device, IAM20680_CONFIG_DLPF_CFG_176);
		rc |= inv_iam20680_wr_accel_config2_a_dlpf_cfg(&icm_device, IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_218);

		/* Set averaging filter. Should be set to 0 in low noise mode */
		rc |= inv_iam20680_wr_lp_mode_cfg_g_avgcfg(&icm_device, IAM20680_LP_MODE_CFG_G_AVGCFG_1x);
		rc |= inv_iam20680_wr_accel_config2_dec2_cfg(&icm_device, IAM20680_ACCEL_CONFIG2_DEC2_CFG_4x);
	}
	else {
		/* Set DLPF_CFG */
		rc |= inv_iam20680_wr_config_dlpf_cfg(&icm_device, IAM20680_CONFIG_DLPF_CFG_176);
		rc |= inv_iam20680_wr_accel_config2_a_dlpf_cfg(&icm_device, IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_420);

		/* Set averaging filter */
		rc |= inv_iam20680_wr_lp_mode_cfg_g_avgcfg(&icm_device, IAM20680_LP_MODE_CFG_G_AVGCFG_1x);
		rc |= inv_iam20680_wr_accel_config2_dec2_cfg(&icm_device, IAM20680_ACCEL_CONFIG2_DEC2_CFG_4x);
	}

	/* Set sample rate divider */
	rc |= inv_iam20680_wr_smplrt_div(&icm_device, device_config->odr_us/1000-1);

	/* Set full scale range */
	rc |= inv_iam20680_set_gyro_fsr(&icm_device, device_config->gyr_fsr_dps);
	rc |= inv_iam20680_set_accel_fsr(&icm_device, device_config->acc_fsr_g);

	/* Enable Accel and Gyro axes */
	if(device_config->enable_accel) {
		rc |= inv_iam20680_enable_accel(&icm_device);
		if (!(device_config->set_low_noise_mode) && !(device_config->enable_gyro)) {
			rc |= inv_iam20680_wr_lp_mode_cfg_set_lp_accel_odr(&icm_device, (double)1000000.0/device_config->odr_us);
			rc |= inv_iam20680_wr_accel_config2_a_dlpf_cfg(&icm_device, IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_420);
			rc |= inv_iam20680_set_accel_low_power_mode(&icm_device);
		}
	}

	if(device_config->enable_gyro) {
		rc |= inv_iam20680_enable_gyro(&icm_device);
	}

	/* Reset then Enable FIFO */
	rc |= inv_iam20680_wr_user_ctrl_fifo_en(&icm_device, IAM20680_USER_CTRL_FIFO_EN_disable);
	rc |= inv_iam20680_wr_user_ctrl_fifo_rst(&icm_device);
	if(device_config->enable_gyro) {
		rc |= inv_iam20680_wr_fifo_en_gyro_fifo_en(&icm_device, IAM20680_FIFO_EN_GYRO_FIFO_EN_enable);
	}
	if(device_config->enable_accel) {
		rc |= inv_iam20680_wr_fifo_en_accel_fifo_en(&icm_device, IAM20680_FIFO_EN_ACCEL_FIFO_EN_enable);
	}

	rc |= inv_iam20680_wr_user_ctrl_fifo_en(&icm_device, IAM20680_USER_CTRL_FIFO_EN_enable);

	return rc;
}

struct MultipleVariables GetDataFromFIFO(inv_dev_config_t* device_config)
{
	struct MultipleVariables RawData;
								RawData.a_x = 0;
								RawData.a_y = 0;
								RawData.a_z = 0;
								RawData.g_x = 0;
								RawData.g_y = 0;
								RawData.g_z = 0;
	int rc = 0;
	uint8_t int_status = 0;
	int16_t raw_acc[3], raw_gyro[3];
	uint16_t packet_count = 0, packet_count_i = 0;
	uint16_t packet_size = 0;
	uint32_t timestamp = 0;

	if(device_config->enable_accel) {
		packet_size += ACCEL_DATA_SIZE;
	}
	if(device_config->enable_gyro) {
		packet_size += GYRO_DATA_SIZE;
	}

	/* Ensure data ready status */
	rc = inv_iam20680_rd_int_status(&icm_device, &int_status);
	if(rc != INV_ERROR_SUCCESS)
		return RawData;

	if(int_status & BIT_DATA_RDY_INT_MASK) {

		uint8_t fifo_count[2];
		inv_iam20680_rd_fifo_count(&icm_device, fifo_count);

		packet_count = (fifo_count[0] << 8) | fifo_count[1];
		packet_count /= packet_size; 	/* FIFO byte mode */

		if(int_status & BIT_FIFO_OVERFLOW_MASK) {
			
			/* Overflow detected */
			INV_MSG(INV_MSG_LEVEL_WARNING, "FIFO overflow detected!");
			
			inv_iam20680_wr_user_ctrl_fifo_rst(&icm_device);
			clear_irq();
			
			return RawData;
		}
		else if(packet_count > 0) {
			/* Read FIFO only when data is expected in FIFO */
			//INV_MSG(INV_MSG_LEVEL_WARNING, "FIFO packet count:%d",packet_count);
			for (packet_count_i=0; packet_count_i<packet_count; packet_count_i++) {
				
				uint8_t data[ACCEL_DATA_SIZE + GYRO_DATA_SIZE];

				if((rc = inv_iam20680_rd_fifo(&icm_device, packet_size, data)) != INV_ERROR_SUCCESS) {
					/* sensor data is in FIFO according to FIFO_COUNT but failed to read FIFO,
					   reset FIFO and try next chance */
					inv_iam20680_wr_user_ctrl_fifo_rst(&icm_device);
					return RawData;
				}

				/* Read Timestamp from ring-buffer */
				timestamp = timer_get_irq_timestamp_us();
				
				/* raw sensor data output is in big endian */
				if(device_config->enable_accel && device_config->enable_gyro) {
                    raw_acc[0] = (data[0] << 8) | data[1];
					raw_acc[1] = (data[2] << 8) | data[3];
					raw_acc[2] = (data[4] << 8) | data[5];
					raw_gyro[0] = (data[6] << 8) | data[7];
					raw_gyro[1] = (data[8] << 8) | data[9];
					raw_gyro[2] = (data[10] << 8)| data[11];

					/*Storing these into variables to be used for filtering in main code*/
					struct MultipleVariables RawData;
					RawData.a_x = raw_acc[0];
					RawData.a_y = raw_acc[0];
					RawData.a_z = raw_acc[0];
					RawData.g_x = raw_acc[0];
					RawData.g_y = raw_acc[0];
					RawData.g_z = raw_acc[0];


					INV_MSG(INV_MSG_LEVEL_INFO, "%u: a/g %d, %d, %d, %d, %d, %d", timestamp, raw_acc[0], raw_acc[1], raw_acc[2], raw_gyro[0], raw_gyro[1], raw_gyro[2]);
				}
				else if(device_config->enable_accel && !device_config->enable_gyro) {
					raw_acc[0] = (data[0] << 8) | data[1];
					raw_acc[1] = (data[2] << 8) | data[3];
					raw_acc[2] = (data[4] << 8) | data[5];
					INV_MSG(INV_MSG_LEVEL_INFO, "%u: a %d, %d, %d", timestamp, raw_acc[0], raw_acc[1], raw_acc[2]);
				}
				else if(!device_config->enable_accel && device_config->enable_gyro) {
					raw_gyro[0] = (data[0] << 8) | data[1];
					raw_gyro[1] = (data[2] << 8) | data[3];
					raw_gyro[2] = (data[4] << 8) | data[5];
					INV_MSG(INV_MSG_LEVEL_INFO, "%u: g %d, %d, %d", timestamp, raw_gyro[0], raw_gyro[1], raw_gyro[2]);
				} 
			} // for loop - packet processing
		} // packet_count > 0
	} 

	return RawData;
}
