/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
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

/** @defgroup DriverIam20680Unit Iam20680 driver unit functions
 *  @brief Unit functions to access Iam20680 device registers
 *  @ingroup  DriverIam20680
 *  @{
 */

#include "Iam20680Defs.h"
#include "Iam20680ExtFunc.h"
#include "Iam20680Transport.h"

#include "InvError.h"

/** @brief Set SMPLRT_DIV register
 *
 *  Divides the internal sample rate (see register CONFIG) to generate the sample rate
 *  that controls sensor data output rate, FIFO sample rate. NOTE: This register is only
 *  effective when FCHOICE_B register bits are 2�b00, and (0 < DLPF_CFG < 7).
 *
 *  This is the update rate of the sensor register: <br>
 *  SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV) <br>
 *  Where INTERNAL_SAMPLE_RATE = 1kHz.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value Value range 0 ~ 255
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_smplrt_div(struct inv_iam20680 * s, uint8_t new_value)
{
	return inv_iam20680_write_reg(s, MPUREG_SMPLRT_DIV, 1, &new_value);
}

/** @brief Set CONFIG register FIFO_MODE bit
 *
 *  When set to �1�, when the FIFO is full, additional writes will not be written to FIFO. <br>
 *  When set to �0�, when the FIFO is full, additional writes will be written to the FIFO, replacing the oldest data.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_CONFIG_FIFO_MODE_t
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_config_fifo_mode(struct inv_iam20680 * s, IAM20680_CONFIG_FIFO_MODE_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_CONFIG, 1, &value);
	if(status)
		return status;

	value &= ~BIT_FIFO_MODE_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_CONFIG, 1, &value);

	return status;
}

/** @brief Set CONFIG register DLPF_CFG bits
 *
 *  The DLPF is configured by DLPF_CFG, when FCHOICE_B [1:0] = 2b�00.
 *  The gyroscope and temperature sensor are filtered according to the value of DLPF_CFG and FCHOICE_B.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_CONFIG_DLPF_CFG_t
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_config_dlpf_cfg(struct inv_iam20680 * s, IAM20680_CONFIG_DLPF_CFG_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_CONFIG, 1, &value);
	if(status)
		return status;

	value &= ~BIT_DLPF_CFG_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_CONFIG, 1, &value);

	return status;
}

/** @brief Set GYRO_CONFIG register FS_SEL bits
 *
 *  Gyro Full Scale Select: <br>
 *  00 = �250dps <br>
 *  01 = �500dps <br>
 *  10 = �1000dps <br>
 *  11 = �2000dps
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_GYRO_CONFIG_FS_SEL_t.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_gyro_config_fs_sel(struct inv_iam20680 * s, IAM20680_GYRO_CONFIG_FS_SEL_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_GYRO_CONFIG, 1, &value);
	if(status)
		return status;

	value &= ~BIT_GYRO_FS_SEL_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_GYRO_CONFIG, 1, &value);

	return status;
}

/** @brief Get GYRO_CONFIG register FS_SEL bits
 *
 *  Gyro Full Scale Select: <br>
 *  00 = �250dps <br>
 *  01 = �500dps <br>
 *  10 = �1000dps <br>
 *  11 = �2000dps
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[out] value See enum IAM20680_GYRO_CONFIG_FS_SEL_t.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_rd_gyro_config_fs_sel(struct inv_iam20680 * s, IAM20680_GYRO_CONFIG_FS_SEL_t * value)
{
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_GYRO_CONFIG, 1, (uint8_t *)value);
	if(status)
		return status;

	*value &= BIT_GYRO_FS_SEL_MASK;

	return status;
}

/** @brief Set GYRO_CONFIG register FCHOICE_B bits
 *
 *  Used to bypass DLPF.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value Possible values: 0, 1, 2, 3. Please refer to datasheet for details.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_gyro_config_fchoice_b(struct inv_iam20680 * s, uint8_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_GYRO_CONFIG, 1, &value);
	if(status)
		return status;

	value &= ~BIT_FCHOICE_B_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_GYRO_CONFIG, 1, &value);

	return status;
}

/** @brief Set ACCEL_CONFIG register ACCEL_FS_SEL bits
 *
 *  Accel Full Scale Select: <br>
 *  �2g (00), �4g (01), �8g (10), �16g (11)
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_ACCEL_CONFIG_FS_SEL_t.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_accel_config_accel_fs_sel(struct inv_iam20680 * s, IAM20680_ACCEL_CONFIG_FS_SEL_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_ACCEL_CONFIG, 1, &value);
	if(status)
		return status;

	value &= ~BIT_ACCEL_FS_SEL_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_ACCEL_CONFIG, 1, &value);

	return status;
}

/** @brief Get ACCEL_CONFIG register ACCEL_FS_SEL bits
 *
 *  Accel Full Scale Select: <br>
 *  �2g (00), �4g (01), �8g (10), �16g (11)
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[out] value See enum IAM20680_ACCEL_CONFIG_FS_SEL_t.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_rd_accel_config_accel_fs_sel(struct inv_iam20680 * s, IAM20680_ACCEL_CONFIG_FS_SEL_t * value)
{
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_ACCEL_CONFIG, 1, (uint8_t *)value);
	if(status)
		return status;

	*value &= BIT_ACCEL_FS_SEL_MASK;

	return status;
}

/** @brief Set ACCEL_CONFIG2 register DEC2_CFG bits
 *
 *  Averaging filter settings for Low Power Accelerometer mode: <br>
 *  0 = Average 4 samples <br>
 *  1 = Average 8 samples <br>
 *  2 = Average 16 samples <br>
 *  3 = Average 32 samples
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_ACCEL_CONFIG2_DEC2_CFG_t.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_accel_config2_dec2_cfg(struct inv_iam20680 * s, IAM20680_ACCEL_CONFIG2_DEC2_CFG_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &value);
	if(status)
		return status;

	value &= ~BIT_DEC2_CFG_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &value);

	return status;
}

/** @brief Set ACCEL_CONFIG2 register ACCEL_FCHOICE_B bits
 *
 *  Used to bypass DLPF.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value Possible values: 0, 1. Please refer to datasheet for details.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_accel_config2_accel_fchoice_b(struct inv_iam20680 * s, uint8_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &value);
	if(status)
		return status;

	value &= ~BIT_ACCEL_FCHOICE_B_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &value);

	return status;
}

/** @brief Set ACCEL_CONFIG2 register A_DLPF_CFG bits
 *
 *  Accelerometer low pass filter setting.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_t.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_accel_config2_a_dlpf_cfg(struct inv_iam20680 * s, IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &value);
	if(status)
		return status;

	value &= ~BIT_A_DLPF_CFG_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &value);

	return status;
}
#if ((IAM20680_HP == 1) || (IAM20680_HT == 1))
/** @brief Set ACCEL_CONFIG2 register FIFO_SIZE bits
*
*  Accelerometer FIFO size setting.
*
*  @param[in] s Pointer to the driver context structure inv_iam20680
*  @param[in] new_value See enum IAM20680_ACCEL_CONFIG2_FIFO_SIZE_t.
*  @return 0 in case of success, negative value on error. See enum inv_status
*/
int inv_iam20680_wr_accel_config2_FIFO_size(struct inv_iam20680 * s, IAM20680_ACCEL_CONFIG2_FIFO_SIZE_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &value);
	if(status)
		return status;

	value &= ~BIT_FIFO_SIZE_CFG_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &value);
	
	return status;
}

/** @brief Read FIFO size from sensor
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[out] value. See enum IAM20680_ACCEL_CONFIG2_FIFO_SIZE_t.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_rd_accel_config2_FIFO_size(struct inv_iam20680 * s, IAM20680_ACCEL_CONFIG2_FIFO_SIZE_t * value)
{
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_ACCEL_CONFIG_2, 1, (uint8_t *)value);
	if(status)
		return status;

	*value &= BIT_FIFO_SIZE_CFG_MASK;

	return status;
}
#endif

/** @brief Set LP_MODE_CFG register GYRO_CYCLE bit
 *
 *  When set to �1� low-power gyroscope mode is enabled.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_LP_MODE_CFG_GYRO_CYCLE_t.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_lp_mode_cfg_gyro_cycle(struct inv_iam20680 * s, IAM20680_LP_MODE_CFG_GYRO_CYCLE_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_LP_MODE_CFG, 1, &value);
	if(status)
		return status;

	value &= ~BIT_GYRO_CYCLE_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_LP_MODE_CFG, 1, &value);

	return status;
}

/** @brief Set LP_MODE_CFG register G_AVGCFG bit
 *
 *  Averaging filter configuration for low-power gyroscope mode.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_LP_MODE_CFG_G_AVGCFG_t.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_lp_mode_cfg_g_avgcfg(struct inv_iam20680 * s, IAM20680_LP_MODE_CFG_G_AVGCFG_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_LP_MODE_CFG, 1, &value);
	if(status)
		return status;

	value &= ~BIT_G_AVGCFG_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_LP_MODE_CFG, 1, &value);

	return status;
}

/** @brief Set ODR for Low Power Accelerometer Mode
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] odr_hz Output Data Rate in Frequency
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_lp_mode_cfg_set_lp_accel_odr(struct inv_iam20680 * s, double odr_hz)
{
	uint8_t value;
	int status;
	int i, idx = 6; /* default odr set to 250Hz */
	double avg_odr_hz;
	const IAM20680_LP_MODE_CONFIG_ACCEL_WOM_ODR_CTRL_t lp_mode_config_accl_wom_odr_ctrl_tbl[8] =
	{{4, 3.9}, {5, 7.8}, {6, 15.6}, {7, 31.3}, {8, 62.5}, {9, 125.0}, {10,250.0}, {11,500.0}};
		
	int odr_tbl_len = sizeof(lp_mode_config_accl_wom_odr_ctrl_tbl)/sizeof(lp_mode_config_accl_wom_odr_ctrl_tbl[0]);

	for( i=0; i < odr_tbl_len-1; i++) {
		if( odr_hz <= lp_mode_config_accl_wom_odr_ctrl_tbl[i+1].odr_hz ){
			avg_odr_hz = (lp_mode_config_accl_wom_odr_ctrl_tbl[i].odr_hz + lp_mode_config_accl_wom_odr_ctrl_tbl[i+1].odr_hz)/2;
			idx = (odr_hz < avg_odr_hz) ? lp_mode_config_accl_wom_odr_ctrl_tbl[i].idx : lp_mode_config_accl_wom_odr_ctrl_tbl[i+1].idx;
			break;
		}
	}

	status = inv_iam20680_read_reg(s, MPUREG_LP_MODE_CFG, 1, &value);
	if(status)
		return status;

	value &= ~BIT_ACCEL_WOM_ODR_CTRL_MASK;
	value |= idx;
	status |= inv_iam20680_write_reg(s, MPUREG_LP_MODE_CFG, 1, &value);

	return status;
}

/** @brief Set ACCEL_WOM_THR register WOM_THR bits
 *
 *  This register holds the threshold value for the Wake on Motion Interrupt for accelerometer.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value accel wake on motion x-axis threshold
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_accel_wom_thr(struct inv_iam20680 * s, uint8_t new_value)
{
	return 	inv_iam20680_write_reg(s, MPUREG_ACCEL_WOM_THR, 1, &new_value);
}

/** @brief Set FIFO_EN register GYRO_FIFO_EN bit
 *
 *  <pre>
 *  1 � write TEMP_OUT_H, TEMP_OUT_L, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L,
 *      GYRO_ZOUT_H, and GYRO_ZOUT_L to the FIFO at the sample rate; If enabled, buffering
 *      of data occurs even if data path is in standby.
 *  0 � function is disabled
 *  </pre>
 *
 *  Note: If both GYRO_FIFO_EN And ACCEL_FIFO_EN are 1, write ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L,
 *  ACCEL_ZOUT_H, ACCEL_ZOUT_L, TEMP_OUT_H, TEMP_OUT_L, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H,
 *  and GYRO_ZOUT_L to the FIFO at the sample rate
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_FIFO_EN_GYRO_FIFO_EN_t.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_fifo_en_gyro_fifo_en(struct inv_iam20680 * s, IAM20680_FIFO_EN_GYRO_FIFO_EN_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_FIFO_EN, 1, &value);
	if(status)
		return status;

	value &= ~BIT_GYRO_FIFO_EN_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_FIFO_EN, 1, &value);

	return status;
}

/** @brief Set FIFO_EN register ACCEL_FIFO_EN bit
 *
 *  <pre>
 *  1 � write ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L,
 *      TEMP_OUT_H, and TEMP_OUT_L to the FIFO at the sample rate;
 *  0 � function is disabled
 *  </pre>
 *
 *  Note: If both GYRO_FIFO_EN And ACCEL_FIFO_EN are 1, write ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
 *  ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L, TEMP_OUT_H, TEMP_OUT_L, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H,
 *  GYRO_YOUT_L, GYRO_ZOUT_H, and GYRO_ZOUT_L to the FIFO at the sample rate
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_FIFO_EN_ACCEL_FIFO_EN_t.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_fifo_en_accel_fifo_en(struct inv_iam20680 * s, IAM20680_FIFO_EN_ACCEL_FIFO_EN_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_FIFO_EN, 1, &value);
	if(status)
		return status;

	value &= ~BIT_ACCEL_FIFO_EN_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_FIFO_EN, 1, &value);

	return status;
}

/** @brief Set FIFO_EN register TEMP_FIFO_EN bit
 *
 *  <pre>
 *  1 � write TEMP_OUT_H, and TEMP_OUT_L to the FIFO at the sample rate;
 *  0 � function is disabled
 *  </pre>
 *
 *  Note: If both GYRO_FIFO_EN And TEMP_FIFO_EN are 1, TEMP_OUT_H, TEMP_OUT_L, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H,
 *  GYRO_YOUT_L, GYRO_ZOUT_H, and GYRO_ZOUT_L to the FIFO at the sample rate
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_FIFO_EN_TEMP_FIFO_EN_t.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_fifo_en_temp_fifo_en(struct inv_iam20680 * s, IAM20680_FIFO_EN_TEMP_FIFO_EN_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_FIFO_EN, 1, &value);
	if(status)
		return status;

	value &= ~BIT_TEMP_FIFO_EN_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_FIFO_EN, 1, &value);

	return status;
}

#if (IAM20680_HT==1)
/** @brief Set INT_PIN_CFG Reg INT2_EN Bit
 *
 *  <pre>
 *  1 - all interrupts except for data ready appear on the INT2 pin, 
 *      and data ready interrupt appears on the INT interrupt pin.
 *  0 - all of the interrupts appear on the INT pin, and INT2 interrupt pin is unused.
 *  </pre>
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_INT_PIN_CFG_INT2_EN_t.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_int_pin_cfg_int2_en(struct inv_iam20680 * s, IAM20680_INT_PIN_CFG_INT2_EN_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_INT_PIN_CFG, 1, &value);
	if(status)
		return status;

	value &= ~BIT_INT2_EN_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_INT_PIN_CFG, 1, &value);

	return status;
	
}
#endif

/** @brief Set INT_ENABLE register WOM_INT_EN bits
 *
 *  <pre>
 *  [7:5] WOM_INT_EN 111 - Enable WoM interrupt on accelerometer. Default setting is 000.
 *  </pre>
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value Bit-wise OR of the values of WOM_INT_EN bits
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_int_enable_wom_int_en(struct inv_iam20680 * s, uint8_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_INT_ENABLE, 1, &value);
	if(status)
		return status;

   	value &= ~(BIT_WOM_INT_EN_MASK);
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_INT_ENABLE, 1, &value);

	return status;
}

/** @brief Set INT_ENABLE register DATA_RDY_INT_EN bit
 *
 *  1 � Enable Data ready interrupt; <br>
 *  0 � function is disabled
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_INT_ENABLE_DATA_RDY_INT_EN_t.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_int_enable_data_rdy_int_en(struct inv_iam20680 * s, IAM20680_INT_ENABLE_DATA_RDY_INT_EN_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_INT_ENABLE, 1, &value);
	if(status)
		return status;

	value &= ~BIT_DATA_RDY_INT_EN_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_INT_ENABLE, 1, &value);

	return status;
}

/** @brief Get INT_STATUS register value
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[out] value Interrupt Status
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_rd_int_status(struct inv_iam20680 * s, uint8_t* value)
{
	return inv_iam20680_read_reg(s, MPUREG_INT_STATUS, 1, value);
}

/** @brief Set ACCEL_INTEL_CTRL register ACCEL_INTEL_EN bit
 *
 *  This bit enables the Wake-on-Motion detection logic
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN_t
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_accel_intel_ctrl_accel_intel_en(struct inv_iam20680 * s, IAM20680_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_ACCEL_INTEL_CTRL, 1, &value);
	if(status)
		return status;

	value &= ~BIT_ACCEL_INTEL_EN_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_ACCEL_INTEL_CTRL, 1, &value);

	return status;
}

/** @brief Set ACCEL_INTEL_CTRL register ACCEL_INTEL_MODE bit
 *
 *  0 � Do not use <br>
 *  1 � Compare the current sample with the previous sample
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE_t
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_accel_intel_ctrl_accel_intel_mode(struct inv_iam20680 * s, IAM20680_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_ACCEL_INTEL_CTRL, 1, &value);
	if(status)
		return status;

	value &= ~BIT_ACCEL_INTEL_MODE_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_ACCEL_INTEL_CTRL, 1, &value);

	return status;
}

/** @brief Set USER_CTRL register FIFO_EN bit
 *
 *  1 � Enable FIFO operation mode. <br>
 *  0 � Disable FIFO access from serial interface.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_USER_CTRL_FIFO_EN_t
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_user_ctrl_fifo_en(struct inv_iam20680 * s, IAM20680_USER_CTRL_FIFO_EN_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_USER_CTRL, 1, &value);
	if(status)
		return status;

	value &= ~BIT_FIFO_EN_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_USER_CTRL, 1, &value);

	return status;
}

/** @brief Set USER_CTRL register FIFO_RST bit
 *
 *  1 � Reset FIFO module. Reset is asynchronous.
 *      This bit auto clears after one clock cycle of the internal 20MHz clock.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_user_ctrl_fifo_rst(struct inv_iam20680 * s)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_USER_CTRL, 1, &value);
	if(status)
		return status;

	value |= BIT_FIFO_RST_MASK;

	status = inv_iam20680_write_reg(s, MPUREG_USER_CTRL, 1, &value);

	return status;
}

/** @brief Set PWR_MGMT_1 register DEVICE_RESET bit
 *
 *  1 � Reset the internal registers and restores the default settings.
 *      The bit automatically clears to 0 once the reset is done.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_pwr_mgmt_1_device_reset(struct inv_iam20680 * s)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_PWR_MGMT_1, 1, &value);
	if(status)
		return status;

	value |= BIT_DEVICE_RESET_MASK;

	status = inv_iam20680_write_reg(s, MPUREG_PWR_MGMT_1, 1, &value);

	return status;
}

/** @brief Set PWR_MGMT_1 register SLEEP bit
 *
 *  When set to 1, the chip is set to sleep mode.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_PWR_MGMT_1_SLEEP_t
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_pwr_mgmt_1_sleep(struct inv_iam20680 * s, IAM20680_PWR_MGMT_1_SLEEP_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_PWR_MGMT_1, 1, &value);
	if(status)
		return status;

	value &= ~BIT_SLEEP_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_PWR_MGMT_1, 1, &value);

	return status;
}

/** @brief Set PWR_MGMT_1 register CYCLE bit
 *
 *  When set to 1, and SLEEP and STANDBY are not set to 1, the chip will cycle between sleep and
 *  taking a single accelerometer sample at a rate determined by SMPLRT_DIV
 *
 *  NOTE: When all accelerometer axes are disabled via PWR_MGMT_2 register bits and cycle is enabled,
 *  the chip will wake up at the rate determined by the respective registers above, but will not take any samples.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_PWR_MGMT_1_ACCEL_CYCLE_t
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_pwr_mgmt_1_accel_cycle(struct inv_iam20680 * s, IAM20680_PWR_MGMT_1_ACCEL_CYCLE_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_PWR_MGMT_1, 1, &value);
	if(status)
		return status;

	value &= ~BIT_ACCEL_CYCLE_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_PWR_MGMT_1, 1, &value);

	return status;
}

/** @brief Set PWR_MGMT_1 register GYRO_STANDBY bit
 *
 *  When set, the gyro drive and pll circuitry are enabled, but the sense paths are disabled.
 *  This is a low power mode that allows quick enabling of the gyros.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_PWR_MGMT_1_GYRO_STDBY_t
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_pwr_mgmt_1_gyro_stdby(struct inv_iam20680 * s, IAM20680_PWR_MGMT_1_GYRO_STDBY_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_PWR_MGMT_1, 1, &value);
	if(status)
		return status;

	value &= ~BIT_GYRO_STDBY_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_PWR_MGMT_1, 1, &value);

	return status;
}

/** @brief Set PWR_MGMT_1 register TEMP_DIS bit
 *
 *  When set to 1, this bit disables the temperature sensor.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value See enum IAM20680_PWR_MGMT_1_TEMP_DIS_t
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_pwr_mgmt_1_temp_dis(struct inv_iam20680 * s, IAM20680_PWR_MGMT_1_TEMP_DIS_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_PWR_MGMT_1, 1, &value);
	if(status)
		return status;

	value &= ~BIT_TEMP_DIS_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_PWR_MGMT_1, 1, &value);

	return status;
}

/** @brief Set PWR_MGMT_1 register CLKSEL bit
 *
 *  <pre>
 *  Code Clock Source
 *  0 - Internal 20MHz oscillator
 *  1 - Auto selects the best available clock source � PLL if ready, else use the Internal oscillator
 *  2 - Auto selects the best available clock source � PLL if ready, else use the Internal oscillator
 *  3 - Auto selects the best available clock source � PLL if ready, else use the Internal oscillator
 *  4 - Auto selects the best available clock source � PLL if ready, else use the Internal oscillator
 *  5 - Auto selects the best available clock source � PLL if ready, else use the Internal oscillator
 *  6 - Internal 20MHz oscillator
 *  7 - Stops the clock and keeps timing generator in reset
 *  </pre>
 *
 *  Note: The default value of CLKSEL[2:0] is 001. It is required that CLKSEL[2:0]
 *  be set to 001 to achieve full gyroscope performance.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value Possible values: 0 ~ 7;
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_pwr_mgmt_1_clksel(struct inv_iam20680 * s, uint8_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_PWR_MGMT_1, 1, &value);
	if(status)
		return status;

	value &= ~BIT_CLKSEL_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_PWR_MGMT_1, 1, &value);

	return status;
}

/** @brief Get PWR_MGMT_1 register value
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[out] value the value read from PWR_MGMT_1 register
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_rd_pwr_mgmt_1(struct inv_iam20680 * s, uint8_t* value)
{
	return inv_iam20680_read_reg(s, MPUREG_PWR_MGMT_1, 1, value);
}

/** @brief Set PWR_MGMT_2 register STBY_XA, STBY_YA, STBY_ZA bits
 *
 * <pre>
 * [5] STBY_XA 1 � X accelerometer is disabled
 *             0 � X accelerometer is on
 * [4] STBY_YA 1 � Y accelerometer is disabled
 *             0 � Y accelerometer is on
 * [3] STBY_ZA 1 � Z accelerometer is disabled
 *             0 � Z accelerometer is on
 * </pre>
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value An bitwise OR of IAM20680_PWR_MGMT_2_STBY_XA_t, IAM20680_PWR_MGMT_2_STBY_YA_t and IAM20680_PWR_MGMT_2_STBY_ZA_t;
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_pwr_mgmt_2_accel_stby(struct inv_iam20680 * s, uint8_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_PWR_MGMT_2, 1, &value);
	if(status)
		return status;

	value &= ~BIT_PWR_ACCEL_STBY_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_PWR_MGMT_2, 1, &value);

	return status;
}

/** @brief Set PWR_MGMT_2 register FIFO_LP_EN bit
 *
 * <pre>
 * [7] FIFO_LP_EN 1 � FIFO_LP is on
 *             	  0 � FIFO_LP is disabled
 * </pre>
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_pwr_mgmt_2_fifo_lp_en(struct inv_iam20680 * s, uint8_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_PWR_MGMT_2, 1, &value);
	if(status)
		return status;

	value &= ~BIT_PWR_MGMT_2_FIFO_LP_EN_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_PWR_MGMT_2, 1, &value);

	return status;
}

/** @brief Set PWR_MGMT_2 register STBY_XG, STBY_YG, STBY_ZG bits
 *
 * <pre>
 * [2] STBY_XG 1 � X gyro is disabled
 *             0 � X gyro is on
 * [1] STBY_YG 1 � Y gyro is disabled
 *             0 � Y gyro is on
 * [0] STBY_ZG 1 � Z gyro is disabled
 *             0 � Z gyro is on
 * </pre>
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value An bitwise OR of IAM20680_PWR_MGMT_2_STBY_XG_t, IAM20680_PWR_MGMT_2_STBY_YG_t and IAM20680_PWR_MGMT_2_STBY_ZG_t;
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_pwr_mgmt_2_gyro_stby(struct inv_iam20680 * s, uint8_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_PWR_MGMT_2, 1, &value);
	if(status)
		return status;

	value &= ~BIT_PWR_GYRO_STBY_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_PWR_MGMT_2, 1, &value);

	return status;
}

/** @brief Set PWR_MGMT_1 register TEMP_DIS bit
 *
 * <pre>
 * [3] TEMP_DIS 1 � Temp is disabled
 *              0 � Temp is on
 * </pre>
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in] new_value
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_wr_pwr_mgmt_1_temp_ctrl(struct inv_iam20680 * s, uint8_t new_value)
{
	uint8_t value;
	int status;

	status = inv_iam20680_read_reg(s, MPUREG_PWR_MGMT_1, 1, &value);
	if(status)
		return status;

	value &= ~BIT_PWR_TEMP_DIS_MASK;
	value |= new_value;

	status = inv_iam20680_write_reg(s, MPUREG_PWR_MGMT_1, 1, &value);

	return status;
}

/** @brief Get FIFO count from FIFO_COUNTH and FIFO_COUNTL registers
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[out] value 13-bit FIFO count.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_rd_fifo_count(struct inv_iam20680 * s, uint8_t* value)
{
	return inv_iam20680_read_reg(s, MPUREG_FIFO_COUNTH, 2, value);
}

/** @brief Read FIFO
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[in]  len Length of data to read, in bytes.
 *  @param[out] value FIFO data.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_rd_fifo(struct inv_iam20680 * s, int len, uint8_t* value)
{
	return inv_iam20680_read_reg(s, MPUREG_FIFO_R_W, len, value);
}

/** @brief Get WHO_AM_I register value
 *
 *  Register to indicate to user which device is being accessed.
 *
 *  @param[in] s Pointer to the driver context structure inv_iam20680
 *  @param[out] value The whoami of the device.
 *  @return 0 in case of success, negative value on error. See enum inv_status
 */
int inv_iam20680_rd_who_am_i(struct inv_iam20680 * s, uint8_t * value)
{
	return inv_iam20680_read_reg(s, MPUREG_WHO_AM_I, 1, value);
}


/** @} */

