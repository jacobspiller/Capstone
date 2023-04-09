/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
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
#ifndef _EXAMPLE_RAW_AG_H_
#define _EXAMPLE_RAW_AG_H_

#include <stdint.h>
#include "Invn/EmbUtils/InvBool.h"

/** @brief Iam20680 device configurations
 */
typedef struct inv_dev_config {
	uint8_t enable_accel;       // 1: Enable Accel, 0: Disable Accel
	uint8_t enable_gyro;        // 1: Enable Gyro, 0: Disable Gyro
	uint8_t set_low_noise_mode; // 1: Set the device in low noise mode, 0: Set the device in low power mode
	int32_t acc_fsr_g;          // Accel Full Scale Range in g
	int32_t gyr_fsr_dps;        // Gyro Full Scale Range in dps
	uint32_t odr_us;            // Output Data Rate in micro second
} inv_dev_config_t;

/*!
 * \brief Set up the Invensense device
 * \param[in] read_reg Function pointer for reading register
 * \param[in] write_reg Function pointer for writing register
 * \param[in] isSPI 1: using SPI interface; 0: using I2C interface
 */
int SetupInvDevice(int (*read_reg)(void * context, uint8_t reg, uint8_t * buf, uint32_t len),
				   int (*write_reg)(void * context, uint8_t reg, const uint8_t * buf, uint32_t len),
				   inv_bool_t isSPI);

/*!
 * \brief Configure Invensense device to some specific settings
 * \param[in] device_config device configurations
 */
int ConfigureInvDevice(inv_dev_config_t* device_config);

/*!
 * \brief Read FIFO to get sensor data
 * \param[in] device_config device configurations
 */
int GetDataFromFIFO(inv_dev_config_t* device_config);


#endif /* !_EXAMPLE_RAW_AG_H_ */
