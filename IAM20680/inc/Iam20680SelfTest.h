/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
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

/** @defgroup DriverIam20680Selftest Iam20680 self test
 *  @brief Perform self test to a Iam20680 device
 *  @ingroup  DriverIam20680
 *  @{
 */

#ifndef _INV_IAM20680_SELFTEST_H_
#define _INV_IAM20680_SELFTEST_H_

#ifdef __cplusplus
extern "C" {
#endif

/* forward declaration */
struct inv_iam20680;

/**
*  @brief      Perform hardware self-test and collect Low Noise bias for Accel and Gyro
*  @param[in]  s Pointer to the driver context structure inv_iam20680
*  @return     Accel and Gyro self test status
*              [0] bit : Gyro status
*                        1 : Pass
*						 0 : Fail
*			   [1] bit  Accel status 
*                        1 : Pass
*						 0 : Fail
*/
int inv_iam20680_run_selftest(struct inv_iam20680 * s);

/**
*  @brief      Perform factory calib and collect Low Noise and Low Power bias for Accel and Gyro.
*  @param[in]  s Pointer to the driver context structure inv_iam20680
*  @return     0 in case of success, negative value on error. See enum inv_status
*/
int inv_iam20680_run_factory_calib(struct inv_iam20680 * s);

/**
*  @brief      Converts bias collected by factory calib to readable format and scale by 2^16.
*  @param[in]  s Pointer to the driver context structure inv_iam20680
*  @param[out] raw_gyro_bias bias scaled by 2^16,  gyro is dps.
*                      The buffer will be stuffed in order as below.
*                      Gyro normal mode X,Y,Z
*                      Gyro LP mode X,Y,Z
*  @param[out] raw_accel_bias bias scaled by 2^16, accel is gee 
*                      The buffer will be stuffed in order as below.
*                      Accel normal mode X,Y,Z
*                      Accel LP mode X,Y,Z
*/

void inv_iam20680_get_scaled_bias(struct inv_iam20680 * s, int * raw_gyro_bias, int *raw_accel_bias);

/**
*  @brief      Retrieve offset computed after factory calib and writes to offset registers.
*              Needs to be called every time sensors are enabled and accel/gyro power mode changes.
*  @param[in]  s Pointer to the driver context structure inv_iam20680
*  @param[out] isAccelLpMode accel power mode .
*                      0: accel LN mode is enabled
*                      1: accel LP mode is enabled
*  @param[out] isGyroLpmode gyro power mode 
*                      0: gyro LN mode is enabled
*                      1: gyro LP mode is enabled
*/
void inv_iam20680_apply_offset(struct inv_iam20680 * s, int isAccelLpMode, int isGyroLpmode);


/**
*  @brief      Compute accel/gyro offset and save to struct.
*              Gyro : For gyro convert 250 dps to 1000 dps and save 2's complement
*              Accel: saves diff from factory offset converted to 16g from 2g and removing gravity
*  @param[in]  s Pointer to the driver context structure inv_iam20680
*/
void inv_iam20680_compute_offset(struct inv_iam20680 * s);

/**
*  @brief      Retrieve accel offset register values and save to device struct.( gyro factory trim is 0 )
*              Need to be called on init to save trim offset values
*  @param[in]  s Pointer to the driver context structure inv_iam20680
*/
void inv_iam20680_get_trim_offset(struct inv_iam20680 * s);

/**
*  @brief      Writes trim offset values saved in struct to offset registers
*  @param[in]  s Pointer to the driver context structure inv_iam20680
*/
void inv_iam20680_set_trim_offset(struct inv_iam20680 * s);


#ifdef __cplusplus
}
#endif

#endif /* _INV_IAM20680_SELFTEST_H_ */

/** @} */
