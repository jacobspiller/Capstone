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

#include "Iam20680Defs.h"
#include "Iam20680ExtFunc.h"
#include "Iam20680Transport.h"
#include "Iam20680Driver_HL.h"

/* the following functions are used for accessing registers */

int inv_iam20680_read_reg(struct inv_iam20680 * s, uint8_t reg, uint32_t length, uint8_t *data)
{
	assert(s);

	struct inv_iam20680_serif * serif = &(s->serif);

	if(length > serif->max_read)
		return INV_ERROR_SIZE;

	if(serif->read_reg(serif->context, reg, data, length) != 0)
		return INV_ERROR_TRANSPORT;

	return INV_ERROR_SUCCESS;
}

int inv_iam20680_write_reg(struct inv_iam20680 * s, uint8_t reg, uint32_t length, const uint8_t *data)
{
	assert(s);

	struct inv_iam20680_serif * serif = &(s->serif);

	if(length > serif->max_write)
		return INV_ERROR_SIZE;

	if(serif->write_reg(serif->context, reg, data, length) != 0)
		return INV_ERROR_TRANSPORT;

	return INV_ERROR_SUCCESS;
}
