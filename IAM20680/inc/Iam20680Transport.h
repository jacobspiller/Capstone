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

/** @defgroup DriverIam20680Transport Iam20680 driver transport layer
 *  @brief    Low-level Iam20680 register access
 *  @ingroup  DriverIam20680
 *  @{
 */

#ifndef _INV_IAM20680_TRANSPORT_H_
#define _INV_IAM20680_TRANSPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "InvBool.h"
#include "InvError.h"

#include <stdint.h>

/** @brief basesensor serial interface
 */
struct inv_iam20680_serif {
	void *     context;   /*!< reserved */
	int      (*read_reg)(void * context, uint8_t reg, uint8_t * buf, uint32_t len);        /*!< pointer to the low-level serial interface read function */
	int      (*write_reg)(void * context, uint8_t reg, const uint8_t * buf, uint32_t len); /*!< pointer to the low-level serial interface write function */
	uint32_t   max_read;  /*!< maximum number of bytes allowed per serial read */
	uint32_t   max_write; /*!< maximum number of bytes allowed per serial write */
	inv_bool_t is_spi;
};

/* forward declaration */
struct inv_iam20680;

/** @brief Writes data to a register on mems.
 * @param[in]  s Pointer to the driver context structure inv_iam20680
 * @param[in]  reg    register address
 * @param[in]  length number of bytes to be written
 * @param[in]  data   data to be written to the register
 * @return     0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_write_reg(struct inv_iam20680 * s, uint8_t reg, uint32_t length, const uint8_t *data);

/** @brief Reads data from a register on mems.
 * @param[in]   s Pointer to the driver context structure inv_iam20680
 * @param[in]   reg    register address
 * @param[in]  	length length of data
 * @param[out]  data   output data from the register
 * @return     0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_read_reg(struct inv_iam20680 * s, uint8_t reg, uint32_t length, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IAM20680_TRANSPORT_H_ */

/** @} */
