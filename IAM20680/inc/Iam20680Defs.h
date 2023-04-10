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

#ifndef _INV_IAM20680_DEFS_H_
#define _INV_IAM20680_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "Iam20680Product.h"

/* Sanity checks */
#if ((IAM20680 + IAM20680_HP + IAM20680_HT) != 1)
#error "Please define one and only one targeted IAM20680 variant in Iam20680Product.h"
#endif

/* WHOAMI value for IAM20680 or IAM20680HP*/
#define IAM20680_WHOAMI     0XA9
#define IAM20680HP_WHOAMI   0XF8
#define IAM20680HT_WHOAMI   0XFA

#if ( IAM20680 == 1)
#define EXPECTED_WHOAMI IAM20680_WHOAMI /* WHOAMI value for IAM20680 */
#elif (IAM20680_HP == 1)
#define EXPECTED_WHOAMI IAM20680HP_WHOAMI /* WHOAMI value for IAM20680 HP */
#elif (IAM20680_HT == 1)
#define EXPECTED_WHOAMI IAM20680HT_WHOAMI /* WHOAMI value for IAM20680 HT */
#endif


/* forward declaration */
struct inv_iam20680;

/* Device Registers */
#define MPUREG_SELF_TEST_X_GYRO   0x00
#define MPUREG_SELF_TEST_Y_GYRO   0x01
#define MPUREG_SELF_TEST_Z_GYRO   0x02
#define MPUREG_SELF_TEST_X_ACCEL  0x0D
#define MPUREG_SELF_TEST_Y_ACCEL  0x0E
#define MPUREG_SELF_TEST_Z_ACCEL  0x0F
#define MPUREG_XG_OFFS_USRH       0x13
#define MPUREG_YG_OFFS_USRH       0x15
#define MPUREG_ZG_OFFS_USRH       0x17
#define MPUREG_SMPLRT_DIV         0x19
#define MPUREG_CONFIG             0x1A
#define MPUREG_GYRO_CONFIG        0x1B
#define MPUREG_ACCEL_CONFIG       0x1C
#define MPUREG_ACCEL_CONFIG_2     0x1D
#define MPUREG_LP_MODE_CFG        0x1E
#define MPUREG_ACCEL_WOM_THR      0x1F
#define MPUREG_FIFO_EN            0x23
#define MPUREG_INT_PIN_CFG        0x37
#define MPUREG_INT_ENABLE         0x38
#define MPUREG_INT_STATUS         0x3A
#define MPUREG_ACCEL_XOUT_H       0x3B
#define MPUREG_ACCEL_XOUT_L       0x3C
#define MPUREG_ACCEL_YOUT_H       0x3D
#define MPUREG_ACCEL_YOUT_L       0x3E
#define MPUREG_ACCEL_ZOUT_H       0x3F
#define MPUREG_ACCEL_ZOUT_L       0x40
#define MPUREG_TEMP_OUT_H         0x41
#define MPUREG_TEMP_OUT_L         0x42
#define MPUREG_GYRO_XOUT_H        0x43
#define MPUREG_GYRO_XOUT_L        0x44
#define MPUREG_GYRO_YOUT_H        0x45
#define MPUREG_GYRO_YOUT_L        0x46
#define MPUREG_GYRO_ZOUT_H        0x47
#define MPUREG_GYRO_ZOUT_L        0x48
#define MPUREG_SIGNAL_PATH_RESET  0x68
#define MPUREG_ACCEL_INTEL_CTRL   0x69
#define MPUREG_USER_CTRL          0x6A
#define MPUREG_PWR_MGMT_1         0x6B
#define MPUREG_PWR_MGMT_2         0x6C
#define MPUREG_FIFO_COUNTH        0x72
#define MPUREG_FIFO_COUNTL        0x73
#define MPUREG_FIFO_R_W           0x74
#define MPUREG_WHO_AM_I           0x75
#define MPUREG_XA_OFFSET_H        0x77
#define MPUREG_YA_OFFSET_H        0x7A
#define MPUREG_ZA_OFFSET_H        0x7D

/**
 * @addtogroup DriverIam20680Unit
 * @{
 */

/* Associated Bit and Function Definitions */

/********************************************
Register Name: SMPLRT_DIV
Register Type: READ/WRITE
Register Address: 25 (Decimal); 19 (Hex)
********************************************/
int inv_iam20680_wr_smplrt_div(struct inv_iam20680 * s, uint8_t new_value);

/********************************************
Register Name: CONFIG
Register Type: READ/WRITE
Register Address: 26 (Decimal); 1A (Hex)
********************************************/
/**
  * @brief  CONFIG Reg FIFO_MODE Bit enumeration
  */
typedef enum
{
  IAM20680_CONFIG_FIFO_MODE_SNAPSHOT    = 0x40, /*!< snapshot mode */
  IAM20680_CONFIG_FIFO_MODE_STREAM      = 0x00, /*!< stream mode */
} IAM20680_CONFIG_FIFO_MODE_t;

#define BIT_FIFO_MODE_MASK               0x40
#define BIT_FIFO_MODE_POS                   6
int inv_iam20680_wr_config_fifo_mode(struct inv_iam20680 * s, IAM20680_CONFIG_FIFO_MODE_t new_value);

#define BIT_EXT_SYNC_SET_MASK            0x38	
#define BIT_EXT_SYNC_SET_POS                3

/**
  * @brief  CONFIG Reg DLPF_CFG Bits enumeration
  */
typedef enum
{
	IAM20680_CONFIG_DLPF_CFG_250    = 0, /*!< gyro 3-dB BW(Hz) = 250,  Rate(kHz) = 8 */
	IAM20680_CONFIG_DLPF_CFG_176    = 1, /*!< gyro 3-dB BW(Hz) = 176,  Rate(kHz) = 1 */
	IAM20680_CONFIG_DLPF_CFG_92 	= 2, /*!< gyro 3-dB BW(Hz) = 92,   Rate(kHz) = 1 */
	IAM20680_CONFIG_DLPF_CFG_41 	= 3, /*!< gyro 3-dB BW(Hz) = 41,   Rate(kHz) = 1 */
	IAM20680_CONFIG_DLPF_CFG_20 	= 4, /*!< gyro 3-dB BW(Hz) = 20,   Rate(kHz) = 1 */
	IAM20680_CONFIG_DLPF_CFG_10 	= 5, /*!< gyro 3-dB BW(Hz) = 10,   Rate(kHz) = 1 */
	IAM20680_CONFIG_DLPF_CFG_5		= 6, /*!< gyro 3-dB BW(Hz) = 5,    Rate(kHz) = 1 */
	IAM20680_CONFIG_DLPF_CFG_3281	= 7, /*!< gyro 3-dB BW(Hz) = 3281, Rate(kHz) = 8 */
} IAM20680_CONFIG_DLPF_CFG_t;
#define BIT_DLPF_CFG_MASK                0x07
int inv_iam20680_wr_config_dlpf_cfg(struct inv_iam20680 * s, IAM20680_CONFIG_DLPF_CFG_t new_value);

/********************************************
Register Name: GYRO_CONFIG
Register Type: READ/WRITE
Register Address: 27 (Decimal); 1B (Hex)
********************************************/
#define BIT_XG_ST_MASK                   0x80
#define BIT_YG_ST_MASK                   0x40
#define BIT_ZG_ST_MASK                   0x20

/**
  * @brief  GYRO_CONFIG Reg FS_SEL Bits enumeration
  */
typedef enum
{
  IAM20680_GYRO_CONFIG_FS_SEL_250dps       = 0x00, /*!< gyro full scale range = +/-250dps */
  IAM20680_GYRO_CONFIG_FS_SEL_500dps       = 0x08, /*!< gyro full scale range = +/-500dps */
  IAM20680_GYRO_CONFIG_FS_SEL_1000dps      = 0x10, /*!< gyro full scale range = +/-1000dps */
  IAM20680_GYRO_CONFIG_FS_SEL_2000dps      = 0x18, /*!< gyro full scale range = +/-2000dps */
} IAM20680_GYRO_CONFIG_FS_SEL_t;
#define BIT_GYRO_FS_SEL_MASK             0x18
#define BIT_GYRO_FS_SEL_POS                 3
int inv_iam20680_wr_gyro_config_fs_sel(struct inv_iam20680 * s, IAM20680_GYRO_CONFIG_FS_SEL_t new_value);
int inv_iam20680_rd_gyro_config_fs_sel(struct inv_iam20680 * s, IAM20680_GYRO_CONFIG_FS_SEL_t * value);

#define BIT_FCHOICE_B_MASK               0x03
int inv_iam20680_wr_gyro_config_fchoice_b(struct inv_iam20680 * s, uint8_t new_value);

/********************************************
Register Name: ACCEL_CONFIG
Register Type: READ/WRITE
Register Address: 28 (Decimal); 1C (Hex)
********************************************/
#define BIT_XA_ST_MASK                   0x80
#define BIT_YA_ST_MASK                   0x40
#define BIT_ZA_ST_MASK                   0x20

/**
  * @brief  ACCEL_CONFIG Reg ACCEL_FS_SEL Bits enumeration
  */
typedef enum
{
  IAM20680_ACCEL_CONFIG_FS_SEL_2g       = 0x00, /*!< accel full scale range = +/-2g */
  IAM20680_ACCEL_CONFIG_FS_SEL_4g       = 0x08, /*!< accel full scale range = +/-4g */
  IAM20680_ACCEL_CONFIG_FS_SEL_8g       = 0x10, /*!< accel full scale range = +/-8g */
  IAM20680_ACCEL_CONFIG_FS_SEL_16g      = 0x18, /*!< accel full scale range = +/-16g */
} IAM20680_ACCEL_CONFIG_FS_SEL_t;
#define BIT_ACCEL_FS_SEL_MASK            0x18
#define BIT_ACCEL_FS_SEL_POS                3
int inv_iam20680_wr_accel_config_accel_fs_sel(struct inv_iam20680 * s, IAM20680_ACCEL_CONFIG_FS_SEL_t new_value);
int inv_iam20680_rd_accel_config_accel_fs_sel(struct inv_iam20680 * s, IAM20680_ACCEL_CONFIG_FS_SEL_t * value);

/********************************************
Register Name: ACCEL_CONFIG2
Register Type: READ/WRITE
Register Address: 29 (Decimal); 1D (Hex)
********************************************/
#if ((IAM20680_HP == 1) || (IAM20680_HT == 1))
#define BIT_FIFO_SIZE_CFG_MASK              0xC0
#define BIT_FIFO_SIZE_CFG_POS                  6
/**
  * @brief  ACCEL_CONFIG2 Reg FIFO_SIZE Bits enumeration
  */
typedef enum
{
	IAM20680_ACCEL_CONFIG2_FIFO_SIZE_512B   = 0 << BIT_FIFO_SIZE_CFG_POS, /*!< 512 Byte FIFO */
	IAM20680_ACCEL_CONFIG2_FIFO_SIZE_1KB	= 1 << BIT_FIFO_SIZE_CFG_POS, /*!< 1 kByte FIFO */
	IAM20680_ACCEL_CONFIG2_FIFO_SIZE_2KB	= 2 << BIT_FIFO_SIZE_CFG_POS, /*!< 2 kByte FIFO */
	IAM20680_ACCEL_CONFIG2_FIFO_SIZE_4KB	= 3 << BIT_FIFO_SIZE_CFG_POS, /*!< 4 kByte FIFO */
} IAM20680_ACCEL_CONFIG2_FIFO_SIZE_t;

int inv_iam20680_wr_accel_config2_FIFO_size(struct inv_iam20680 * s, IAM20680_ACCEL_CONFIG2_FIFO_SIZE_t new_value);
int inv_iam20680_rd_accel_config2_FIFO_size(struct inv_iam20680 * s, IAM20680_ACCEL_CONFIG2_FIFO_SIZE_t * value);
#endif

/**
  * @brief  ACCEL_CONFIG2 Reg DEC2_CFG Bits enumeration
  */
typedef enum
{
  IAM20680_ACCEL_CONFIG2_DEC2_CFG_4x       = 0x00, /*!< Average 4 samples */
  IAM20680_ACCEL_CONFIG2_DEC2_CFG_8x       = 0x10, /*!< Average 8 samples */
  IAM20680_ACCEL_CONFIG2_DEC2_CFG_16x      = 0x20, /*!< Average 16 samples */
  IAM20680_ACCEL_CONFIG2_DEC2_CFG_32x      = 0x30, /*!< Average 32 samples */
} IAM20680_ACCEL_CONFIG2_DEC2_CFG_t;
#define BIT_DEC2_CFG_MASK                0x30
#define BIT_DEC2_CFG_POS                    4
int inv_iam20680_wr_accel_config2_dec2_cfg(struct inv_iam20680 * s, IAM20680_ACCEL_CONFIG2_DEC2_CFG_t new_value);

#define BIT_ACCEL_FCHOICE_B_MASK         0x08
int inv_iam20680_wr_accel_config2_accel_fchoice_b(struct inv_iam20680 * s, uint8_t new_value);

/**
  * @brief  ACCEL_CONFIG2 Reg A_DLPF_CFG Bits enumeration
  */
typedef enum
{
	IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_218   = 1, /*!< accel 3-dB BW(Hz) = 218.1,  Rate(kHz) = 1 */
	IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_99	= 2, /*!< accel 3-dB BW(Hz) = 99.0,   Rate(kHz) = 1 */
	IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_45	= 3, /*!< accel 3-dB BW(Hz) = 44.8,   Rate(kHz) = 1 */
	IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_21	= 4, /*!< accel 3-dB BW(Hz) = 21.2,   Rate(kHz) = 1 */
	IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_10	= 5, /*!< accel 3-dB BW(Hz) = 10.2,   Rate(kHz) = 1 */
	IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_5		= 6, /*!< accel 3-dB BW(Hz) = 5.1,    Rate(kHz) = 1 */
	IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_420	= 7, /*!< accel 3-dB BW(Hz) = 420.0,  Rate(kHz) = 1 */
} IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_t;
#define BIT_A_DLPF_CFG_MASK              0x07
int inv_iam20680_wr_accel_config2_a_dlpf_cfg(struct inv_iam20680 * s, IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_t new_value);


/********************************************
Register Name: LP_MODE_CFG
Register Type: READ/WRITE
Register Address: 30 (Decimal); 1E (Hex)
********************************************/
/**
  * @brief  LP_MODE_CFG Reg GYRO_CYCLE Bit enumeration
  */
typedef enum
{
  IAM20680_LP_MODE_CFG_GYRO_CYCLE_on     = 0x80, /*!< turn on gyro low power mode */
  IAM20680_LP_MODE_CFG_GYRO_CYCLE_off    = 0x00, /*!< turn off gyro low power mode  */
} IAM20680_LP_MODE_CFG_GYRO_CYCLE_t;
#define BIT_GYRO_CYCLE_MASK              0x80
int inv_iam20680_wr_lp_mode_cfg_gyro_cycle(struct inv_iam20680 * s, IAM20680_LP_MODE_CFG_GYRO_CYCLE_t new_value);

/**
  * @brief  LP_MODE_CFG Reg G_AVGCFG Bits enumeration
  */
typedef enum
{
  IAM20680_LP_MODE_CFG_G_AVGCFG_1x        = 0x00, /*!< Average 1 samples */
  IAM20680_LP_MODE_CFG_G_AVGCFG_2x        = 0x10, /*!< Average 2 samples */
  IAM20680_LP_MODE_CFG_G_AVGCFG_4x        = 0x20, /*!< Average 4 samples */
  IAM20680_LP_MODE_CFG_G_AVGCFG_8x        = 0x30, /*!< Average 8 samples */
  IAM20680_LP_MODE_CFG_G_AVGCFG_16x       = 0x40, /*!< Average 16 samples */
  IAM20680_LP_MODE_CFG_G_AVGCFG_32x       = 0x50, /*!< Average 32 samples */
  IAM20680_LP_MODE_CFG_G_AVGCFG_64x       = 0x60, /*!< Average 64 samples */
  IAM20680_LP_MODE_CFG_G_AVGCFG_128x      = 0x70, /*!< Average 128 samples */
} IAM20680_LP_MODE_CFG_G_AVGCFG_t;
#define BIT_G_AVGCFG_MASK                0x70
#define BIT_G_AVGCFG_POS	                4
int inv_iam20680_wr_lp_mode_cfg_g_avgcfg(struct inv_iam20680 * s, IAM20680_LP_MODE_CFG_G_AVGCFG_t new_value);

/**
  * @brief  IAM20680_LP_MODE_CONFIG_ACCEL_WOM_ODR_CTRL_t structure definition
  */
typedef struct {
    int idx;        /*!< LPOSC_CLKSEL index */
    double odr_hz;  /*!< Output Frequency in Hz */
} IAM20680_LP_MODE_CONFIG_ACCEL_WOM_ODR_CTRL_t;

#define BIT_ACCEL_WOM_ODR_CTRL_MASK              0x0F
int inv_iam20680_wr_lp_mode_cfg_set_lp_accel_odr(struct inv_iam20680 * s, double odr_hz);

/********************************************
Register Name: ACCEL_WOM_THR
Register Type: READ/WRITE
Register Address: 31 (Decimal); 1F (Hex)
********************************************/
#define BIT_WOM_THR_MASK                0xff
int inv_iam20680_wr_accel_wom_thr(struct inv_iam20680 * s, uint8_t new_value);

/********************************************
Register Name: FIFO_EN
Register Type: READ/WRITE
Register Address: 35 (Decimal); 23 (Hex)
********************************************/
/**
  * @brief  FIFO_EN Reg GYRO_FIFO_EN Bit enumeration
  */
typedef enum
{
  IAM20680_FIFO_EN_GYRO_FIFO_EN_enable      = 0x70, /*!< enable gyro in FIFO */
  IAM20680_FIFO_EN_GYRO_FIFO_EN_disable     = 0x00, /*!< disable gyro in FIFO */
} IAM20680_FIFO_EN_GYRO_FIFO_EN_t;
#define BIT_GYRO_FIFO_EN_MASK            0x70
int inv_iam20680_wr_fifo_en_gyro_fifo_en(struct inv_iam20680 * s, IAM20680_FIFO_EN_GYRO_FIFO_EN_t new_value);

/**
  * @brief  FIFO_EN Reg ACCEL_FIFO_EN Bit enumeration
  */
typedef enum
{
  IAM20680_FIFO_EN_ACCEL_FIFO_EN_enable      = 0x08, /*!< enable accel in FIFO */
  IAM20680_FIFO_EN_ACCEL_FIFO_EN_disable     = 0x00, /*!< disable accel in FIFO */
} IAM20680_FIFO_EN_ACCEL_FIFO_EN_t;
#define BIT_ACCEL_FIFO_EN_MASK           0x08
int inv_iam20680_wr_fifo_en_accel_fifo_en(struct inv_iam20680 * s, IAM20680_FIFO_EN_ACCEL_FIFO_EN_t new_value);

/**
  * @brief  FIFO_EN Reg TEMP_FIFO_EN Bit enumeration
  */
typedef enum
{
  IAM20680_FIFO_EN_TEMP_FIFO_EN_enable      = 0x80, /*!< enable temp in FIFO */
  IAM20680_FIFO_EN_TEMP_FIFO_EN_disable     = 0x00, /*!< disable temp in FIFO */
} IAM20680_FIFO_EN_TEMP_FIFO_EN_t;
#define BIT_TEMP_FIFO_EN_MASK          	 0x80
int inv_iam20680_wr_fifo_en_temp_fifo_en(struct inv_iam20680 * s, IAM20680_FIFO_EN_TEMP_FIFO_EN_t new_value);

/********************************************
Register Name: INT_PIN_CFG
Register Type: READ/WRITE
Register Address: 55 (Decimal); 37 (Hex)
********************************************/
#define BIT_INT_LEVEL_MASK               0x80
#define BIT_INT_OPEN_MASK                0x40
#define BIT_LATCH_INT_EN_MASK            0x20
#define BIT_INT_RD_CLEAR_MASK            0x10
#define BIT_FSYNC_INT_LEVEL_MASK         0x08
#define BIT_FSYNC_INT_MODE_EN_MASK       0x04

#if (IAM20680_HT==1)
/**
  * @brief  INT_PIN_CFG Reg INT2_EN Bit enumeration
  */
typedef enum
{
  IAM20680_INT_PIN_CFG_INT2_EN_enable   = 0x01, /*!< enable INT2 */
  IAM20680_INT_PIN_CFG_INT2_EN_disable  = 0x00, /*!< disable INT2 */
} IAM20680_INT_PIN_CFG_INT2_EN_t;
#define BIT_INT2_EN_MASK                 0x01
int inv_iam20680_wr_int_pin_cfg_int2_en(struct inv_iam20680 * s, IAM20680_INT_PIN_CFG_INT2_EN_t new_value);
#endif

/********************************************
Register Name: INT_ENABLE
Register Type: READ/WRITE
Register Address: 56 (Decimal); 38 (Hex)
********************************************/
#define BIT_WOM_INT_EN_MASK              0xE0
int inv_iam20680_wr_int_enable_wom_int_en(struct inv_iam20680 * s, uint8_t new_value);

/**
  * @brief  INT_ENABLE Reg DATA_RDY_INT_EN Bit enumeration
  */
typedef enum
{
  IAM20680_INT_ENABLE_DATA_RDY_INT_EN_enable   = 0x01, /*!< enable data ready interrupt */
  IAM20680_INT_ENABLE_DATA_RDY_INT_EN_disable  = 0x00, /*!< disable data ready interrupt */
} IAM20680_INT_ENABLE_DATA_RDY_INT_EN_t;
#define BIT_DATA_RDY_INT_EN_MASK         0x01
int inv_iam20680_wr_int_enable_data_rdy_int_en(struct inv_iam20680 * s, IAM20680_INT_ENABLE_DATA_RDY_INT_EN_t new_value);

/********************************************
Register Name: INT_STATUS
Register Type: READ to CLEAR
Register Address: 58 (Decimal); 3A (Hex)
********************************************/
#define BIT_WOM_INT_MASK                 0xE0

#define BIT_FIFO_OVERFLOW_MASK           0x10
#define BIT_DATA_RDY_INT_MASK            0x01
int inv_iam20680_rd_int_status(struct inv_iam20680 * s, uint8_t* value);

/********************************************
Register Name: ACCEL_INTEL_CTRL
Register Type: READ/WRITE
Register Address: 105 (Decimal); 69 (Hex)
********************************************/
/**
  * @brief  ACCEL_INTEL_CTRL Reg ACCEL_INTEL_EN Bit enumeration
  */
typedef enum
{
  IAM20680_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN_enable   = 0x80, /*!< enable the Wake-on-Motion detection logic */
  IAM20680_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN_disable  = 0x00, /*!< disable the Wake-on-Motion detection logic */
} IAM20680_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN_t;
#define BIT_ACCEL_INTEL_EN_MASK          0x80
int inv_iam20680_wr_accel_intel_ctrl_accel_intel_en(struct inv_iam20680 * s, IAM20680_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN_t new_value);

/**
  * @brief  ACCEL_INTEL_CTRL Reg ACCEL_INTEL_MODE Bit enumeration
  */
typedef enum
{
  IAM20680_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE_comp_w_prev   = 0x40, /*!< compare the current sample with the previous sample */
  IAM20680_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE_comp_w_first  = 0x00, /*!< Compares the current sample to the first sample taken when entering in low-power mode */
} IAM20680_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE_t;
#define BIT_ACCEL_INTEL_MODE_MASK        0x40
int inv_iam20680_wr_accel_intel_ctrl_accel_intel_mode(struct inv_iam20680 * s, IAM20680_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE_t new_value);

/********************************************
Register Name: USER_CTRL
Register Type: READ/WRITE
Register Address: 106 (Decimal); 6A (Hex)
********************************************/
/**
  * @brief  USER_CTRL Reg FIFO_EN Bit enumeration
  */
typedef enum
{
  IAM20680_USER_CTRL_FIFO_EN_enable      = 0x40, /*!< enable FIFO */
  IAM20680_USER_CTRL_FIFO_EN_disable     = 0x00, /*!< disable FIFO */
} IAM20680_USER_CTRL_FIFO_EN_t;
#define BIT_FIFO_EN_MASK                 0x40
int inv_iam20680_wr_user_ctrl_fifo_en(struct inv_iam20680 * s, IAM20680_USER_CTRL_FIFO_EN_t new_value);

#define BIT_FIFO_RST_MASK                0x04
int inv_iam20680_wr_user_ctrl_fifo_rst(struct inv_iam20680 * s);

#define BIT_SIG_COND_RESET_MASK          0x01

/********************************************
Register Name: PWR_MGMT_1
Register Type: READ/WRITE
Register Address: 107 (Decimal); 6B (Hex)
********************************************/
#define BIT_DEVICE_RESET_MASK            0x80
int inv_iam20680_wr_pwr_mgmt_1_device_reset(struct inv_iam20680 * s);

/**
  * @brief  PWR_MGMT_1 Reg SLEEP Bit enumeration
  */
typedef enum
{
  IAM20680_PWR_MGMT_1_SLEEP_sleep      = 0x40, /*!< set the chip to sleep mode */
  IAM20680_PWR_MGMT_1_SLEEP_awake      = 0x00, /*!< set the chip to awake mode */
} IAM20680_PWR_MGMT_1_SLEEP_t;
#define BIT_SLEEP_MASK                   0x40
int inv_iam20680_wr_pwr_mgmt_1_sleep(struct inv_iam20680 * s, IAM20680_PWR_MGMT_1_SLEEP_t new_value);

/**
  * @brief  PWR_MGMT_1 Reg ACCEL_CYCLE Bit enumeration
  */
typedef enum
{
  IAM20680_PWR_MGMT_1_ACCEL_CYCLE_on         = 0x20, /*!< enable accel low power mode */
  IAM20680_PWR_MGMT_1_ACCEL_CYCLE_off        = 0x00, /*!< disable accel low power mode */
} IAM20680_PWR_MGMT_1_ACCEL_CYCLE_t;
#define BIT_ACCEL_CYCLE_MASK                   0x20
int inv_iam20680_wr_pwr_mgmt_1_accel_cycle(struct inv_iam20680 * s, IAM20680_PWR_MGMT_1_ACCEL_CYCLE_t new_value);

/**
  * @brief  PWR_MGMT_1 Reg GYRO_STDBY Bit enumeration
  */
typedef enum
{
  IAM20680_PWR_MGMT_1_GYRO_STDBY_on      = 0x40, /*!< enable gyro standby mode */
  IAM20680_PWR_MGMT_1_GYRO_STDBY_off     = 0x00, /*!< disable gyro standby mode */
} IAM20680_PWR_MGMT_1_GYRO_STDBY_t;
#define BIT_GYRO_STDBY_MASK              0x10
int inv_iam20680_wr_pwr_mgmt_1_gyro_stdby(struct inv_iam20680 * s, IAM20680_PWR_MGMT_1_GYRO_STDBY_t new_value);

/**
  * @brief  PWR_MGMT_1 Reg TEMP_DIS Bit enumeration
  */
typedef enum
{
  IAM20680_PWR_MGMT_1_TEMP_DIS_enable      = 0x00, /*!< enables the temperature sensor */
  IAM20680_PWR_MGMT_1_TEMP_DIS_disable     = 0x08, /*!< disables the temperature sensor */
} IAM20680_PWR_MGMT_1_TEMP_DIS_t;
#define BIT_TEMP_DIS_MASK                0x08
int inv_iam20680_wr_pwr_mgmt_1_temp_dis(struct inv_iam20680 * s, IAM20680_PWR_MGMT_1_TEMP_DIS_t new_value);

/**
  * @brief  PWR_MGMT_1 Reg TEMP_DIS Bit enumeration
  */
typedef enum
{
  IAM20680_PWR_MGMT_1_TEMP_disable     = 0x01, /*!< temp is disabled */
  IAM20680_PWR_MGMT_1_TEMP_enable      = 0x00, /*!< temp is enabled */
} IAM20680_PWR_MGMT_1_TEMP_t;
#define BIT_PWR_TEMP_DIS_MASK           0x08
int inv_iam20680_wr_pwr_mgmt_1_temp_ctrl(struct inv_iam20680 * s, uint8_t new_value);

#define BIT_CLKSEL_MASK                  0x07
int inv_iam20680_wr_pwr_mgmt_1_clksel(struct inv_iam20680 * s, uint8_t new_value);

int inv_iam20680_rd_pwr_mgmt_1(struct inv_iam20680 * s, uint8_t* value);

/********************************************
Register Name: PWR_MGMT_2
Register Type: READ/WRITE
Register Address: 108 (Decimal); 6C (Hex)
********************************************/
/**
  * @brief  PWR_MGMT_2 Reg STBY_XA Bit enumeration
  */
typedef enum
{
  IAM20680_PWR_MGMT_2_XA_disable      = 0x20, /*!< X accelerometer is disabled */
  IAM20680_PWR_MGMT_2_XA_enable       = 0x00, /*!< X accelerometer is on */
}IAM20680_PWR_MGMT_2_STBY_XA_t;
/**
  * @brief  PWR_MGMT_2 Reg STBY_YA Bit enumeration
  */
typedef enum
{
  IAM20680_PWR_MGMT_2_YA_disable      = 0x10, /*!< Y accelerometer is disabled */
  IAM20680_PWR_MGMT_2_YA_enable       = 0x00, /*!< Y accelerometer is on */
}IAM20680_PWR_MGMT_2_STBY_YA_t;
/**
  * @brief  PWR_MGMT_2 Reg STBY_ZA Bit enumeration
  */
typedef enum
{
  IAM20680_PWR_MGMT_2_ZA_disable      = 0x08, /*!< Z accelerometer is disabled */
  IAM20680_PWR_MGMT_2_ZA_enable       = 0x00, /*!< Z accelerometer is on */
} IAM20680_PWR_MGMT_2_STBY_ZA_t;
#define BIT_PWR_ACCEL_STBY_MASK          0x38
int inv_iam20680_wr_pwr_mgmt_2_accel_stby(struct inv_iam20680 * s, uint8_t new_value);
/**
  * @brief  PWR_MGMT_2 Reg FIFO_LP Bit enumeration
  */
typedef enum
{
  IAM20680_PWR_MGMT_2_FIFO_LP_enable  = 0x80, /*!< FIFO_LP is enabled */
  IAM20680_PWR_MGMT_2_FIFO_LP_disable = 0x00, /*!< FIFO_LP is disabled */
} IAM20680_PWR_MGMT_2_FIFO_LP_t;
#define BIT_PWR_MGMT_2_FIFO_LP_EN_MASK   0x80
int inv_iam20680_wr_pwr_mgmt_2_fifo_lp_en(struct inv_iam20680 * s, uint8_t new_value);

/**
  * @brief  PWR_MGMT_2 Reg STBY_XG Bit enumeration
  */
typedef enum
{
  IAM20680_PWR_MGMT_2_XG_disable      = 0x04, /*!< X gyro is disabled */
  IAM20680_PWR_MGMT_2_XG_enable       = 0x00, /*!< X gyro is on */
}IAM20680_PWR_MGMT_2_STBY_XG_t;
/**
  * @brief  PWR_MGMT_2 Reg STBY_YG Bit enumeration
  */
typedef enum
{
  IAM20680_PWR_MGMT_2_YG_disable      = 0x02, /*!< Y gyro is disabled */
  IAM20680_PWR_MGMT_2_YG_enable       = 0x00, /*!< Y gyro is on */
}IAM20680_PWR_MGMT_2_STBY_YG_t;
/**
  * @brief  PWR_MGMT_2 Reg STBY_ZG Bit enumeration
  */
typedef enum
{
  IAM20680_PWR_MGMT_2_ZG_disable      = 0x01, /*!< Z gyro is disabled */
  IAM20680_PWR_MGMT_2_ZG_enable       = 0x00, /*!< Z gyro is on */
} IAM20680_PWR_MGMT_2_STBY_ZG_t;
#define BIT_PWR_GYRO_STBY_MASK           0x07
int inv_iam20680_wr_pwr_mgmt_2_gyro_stby(struct inv_iam20680 * s, uint8_t new_value);

#define BIT_PWR_ALL_OFF_MASK             0x3f

/********************************************
Register Name: FIFO_COUNTH
Register Type: READ Only
Register Address: 114 (Decimal); 72 (Hex)

Register Name: FIFO_COUNTL
Register Type: READ Only
Register Address: 115 (Decimal); 73 (Hex)
********************************************/
int inv_iam20680_rd_fifo_count(struct inv_iam20680 * s, uint8_t* value);

/********************************************
Register Name: FIFO_R_W
Register Type: READ/WRITE
Register Address: 116 (Decimal); 74 (Hex)
********************************************/
int inv_iam20680_rd_fifo(struct inv_iam20680 * s, int len, uint8_t* value);

/********************************************
Register Name: WHO_AM_I
Register Type: READ Only
Register Address: 117 (Decimal); 75 (Hex)
********************************************/
int inv_iam20680_rd_who_am_i(struct inv_iam20680 * s, uint8_t* value);

/**
  * @}
  */
/* data definitions */
#define ACCEL_DATA_SIZE		6
#define GYRO_DATA_SIZE  	6
#define TEMP_DATA_SIZE		2

#define SAMPLE_RATE_DIVIDER	9    // default sample rate divider
#define CLK_SEL            	1    // default CLK_SEL value
#define RST_VAL_PWR_MGMT_1 						0x40 // the reset value for PWR_MGMT_1 register
#define RST_VAL_PWR_MGMT_1_TDK_INBUILT_SENSOR 	0x01 // the reset value for PWR_MGMT_1 register for TDK-Atmel board with In-built Sensor
#define WHO_AM_I_VAL       	0xA9 // the WHO_AM_I of IAM20680

// ODR range limited by 8-bit HW Gyro rate divider register "SMPLRT_DIV"
#define ODR_MIN_DELAY_LN   	1
#define ODR_MIN_DELAY_LP   	3     // In Low Power mode, max output rate for gyro is 333.33Hz
#define ODR_MAX_DELAY      	250

/* Helper macros */

#ifndef INV_MIN
#define INV_MIN(x,y) (((x)<(y))?(x):(y))
#endif

#ifndef INV_MAX
#define INV_MAX(x,y) (((x)>(y))?(x):(y))
#endif

#ifndef INV_ABS
#define INV_ABS(x) (((x) < 0) ? -(x) : (x))
#endif

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef _INV_IAM20680_DEFS_H_ */
