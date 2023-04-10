

/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : BLE_Beacon_main.c
* Author             : RF Application Team
* Version            : 1.1.0
* Date               : 15-January-2016
* Description        : Code demostrating the BLE Beacon application
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file BLE_Beacon_main.c
 * @brief This is a BLE beacon demo that shows how to configure a BlueNRG-1,2 device
 * in order to advertise specific manufacturing data and allow another BLE device to
 * know if it is in the range of the BlueNRG-1 beacon device.
 * It also provides a reference example about how using the
 * BLE Over-The-Air (OTA) Service Manager firmware upgrade capability.
 *

* \section WiSE-Studio_project WiSE-Studio project
  To use the project with WiSE-Studio , please follow the instructions below:
  -# Open the WiSE-Studio  and select File->Import.
  -# Select Existing Projects into Workspace.
  -# Go to Project Explorer section
  -# Select desired configuration to build from Project->Project->Build Project.
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect a SWD HW programmer in your board (if available).
  -# Download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu.
  -# Open the KEIL project
     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon\\MDK-ARM\\BlueNRG-1\\BLE_Beacon.uvprojx </tt> or
     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon\\MDK-ARM\\BlueNRG-2\\BLE_Beacon.uvprojx </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect a SWD HW programmer in your board (if available).
  -# Download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu.
  -# Open the IAR project
     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon\\EWARM\\BlueNRG-1\\BLE_Beacon.eww </tt> or
     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon\\EWARM\\BlueNRG-2\\BLE_Beacon.eww </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect a SWD HW programmer in your board (if available).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Release - Release configuration
- \c Use_OTA_ServiceManager - Configuration for Application using OTA Service Manager


* \section Board_supported Boards supported
- \c STEVAL-IDB007V1
- \c STEVAL-IDB007V2
- \c STEVAL-IDB008V1
- \c STEVAL-IDB008V1M
- \c STEVAL-IDB008V2
- \c STEVAL-IDB009V1


 * \section Power_settings Power configuration settings
@table

==========================================================================================================
|                                         STEVAL-IDB00XV1                                                |
----------------------------------------------------------------------------------------------------------
| Jumper name |            |  Description                                                                |
| JP1         |   JP2      |                                                                             |
----------------------------------------------------------------------------------------------------------
| ON 1-2      | ON 2-3     | USB supply power                                                            |
| ON 2-3      | ON 1-2     | The supply voltage must be provided through battery pins.                   |
| ON 1-2      |            | USB supply power to STM32L1, JP2 pin 2 external power to BlueNRG1           |


@endtable

* \section Jumper_settings Jumper settings
@table

========================================================================================================================================================================================
|                                                                             STEVAL-IDB00XV1                                                                                          |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Jumper name |                                                                Description                                                                                             |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| JP1         | 1-2: to provide power from USB (JP2:2-3). 2-3: to provide power from battery holder (JP2:1-2)                                                                          |
| JP2         | 1-2: to provide power from battery holder (JP1:2-3). 2-3: to provide power from USB (JP1:1-2). Pin2 to VDD  to provide external power supply to BlueNRG-1 (JP1: 1-2)   |
| JP3         | pin 1 and 2 UART RX and TX of MCU. pin 3 GND.                                                                                                                          |
| JP4         | Fitted: to provide VBLUE to BlueNRG1. It can be used also for current measurement.                                                                                     |
| JP5         | Fitted : TEST pin to VBLUE. Not fitted:  TEST pin to GND                                                                                                               |


@endtable

* \section Pin_settings Pin settings
@table
|            |                                                           Release                                                           ||||||                                                                     Use_OTA_ServiceManager                                                                      ||||||
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  PIN name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |      STEVAL-IDB007V1     |      STEVAL-IDB007V2     |      STEVAL-IDB008V1     |     STEVAL-IDB008V1M     |      STEVAL-IDB008V2     |      STEVAL-IDB009V1     |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|    ADC1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    ADC2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     GND    |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |           N.A.           |           N.A.           |           N.A.           |         Not Used         |           N.A.           |           N.A.           |
|     IO0    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO11    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO12    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO13    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO14    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO15    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO16    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO17    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO18    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO19    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|     IO2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO20    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO21    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO22    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO23    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO24    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO25    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|     IO3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO5    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO6    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO7    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO8    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|   RESETN   |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |           N.A.           |           N.A.           |           N.A.           |         Not Used         |           N.A.           |           N.A.           |
|    TEST1   |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    VBLUE   |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |           N.A.           |           N.A.           |           N.A.           |         Not Used         |           N.A.           |           N.A.           |

@endtable

* \section Serial_IO Serial I/O
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Stop bits       | 1                | bit       |
@endtable

* \section LEDs_description LEDs description
@table
|            |                                                           Release                                                           ||||||                                                                                                                           Use_OTA_ServiceManager                                                                                                                            ||||||
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |               STEVAL-IDB007V1              |               STEVAL-IDB007V2              |               STEVAL-IDB008V1              |              STEVAL-IDB008V1M              |               STEVAL-IDB008V2              |               STEVAL-IDB009V1              |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |

@endtable


* \section Buttons_description Buttons description
@table
|                |                                                           Release                                                           ||||||                                                                                                                                                                                                               Use_OTA_ServiceManager                                                                                                                                                                                                                ||||||
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |                             STEVAL-IDB007V1                            |                             STEVAL-IDB007V2                            |                             STEVAL-IDB008V1                            |                            STEVAL-IDB008V1M                            |                             STEVAL-IDB008V2                            |                             STEVAL-IDB009V1                            |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |  It allows BLE Beacon application to activate the OTA Service Manager  |  It allows BLE Beacon application to activate the OTA Service Manager  |  It allows BLE Beacon application to activate the OTA Service Manager  |  It allows BLE Beacon application to activate the OTA Service Manager  |  It allows BLE Beacon application to activate the OTA Service Manager  |  It allows BLE Beacon application to activate the OTA Service Manager  |
|      PUSH2     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |                                Not Used                                |                                Not Used                                |                                Not Used                                |                                Not Used                                |                                Not Used                                |                                Not Used                                |
|      RESET     |   Reset BlueNRG1   |   Reset BlueNRG1   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |                             Reset BlueNRG1                             |                             Reset BlueNRG1                             |                             Reset BlueNRG2                             |                             Reset BlueNRG2                             |                             Reset BlueNRG2                             |                             Reset BlueNRG2                             |

@endtable

* \section Usage Usage

The Beacon demo configures a BlueNRG-1,2 device in advertising mode (non-connectable mode) with specific manufacturing data.
It transmits advertisement packets at regular intervals which contain the following manufacturing data:
@table
------------------------------------------------------------------------------------------------------------------------
| Data field              | Description                       | Notes                                                  |
------------------------------------------------------------------------------------------------------------------------
| Company identifier code | SIG company identifier (1)        | Default is 0x0030 (STMicroelectronics)                 |
| ID                      | Beacon ID                         | Fixed value                                            |
| Length                  | Length of the remaining payload   | NA                                                     |
| Location UUID           | Beacons UUID                      | It is used to distinguish specific beacons from others |
| Major number            | Identifier for a group of beacons | It is used to group a related set of beacons           |
| Minor number            | Identifier for a single beacon    | It is used to identify a single beacon                 |
| Tx Power                | 2's complement of the Tx power    | It is used to establish how far you are from device    |
@endtable

 - (1): SIG company identifiers are available on https://www.bluetooth.org/en-us/specification/assigned-numbers/company-identifiers
 - NA : Not Applicable;
NOTEs:
     - OTA Service Manager support requires to build application by enabling only ST_USE_OTA_SERVICE_MANAGER_APPLICATION=1 (preprocessor, linker) options and through files: OTA_btl.[ch] (refer to Release_with_OTA_ServiceManager IAR workspace).

**/

/** @addtogroup BlueNRG1_demonstrations_applications
 *  BlueNRG-1,2 Beacon demo \see BLE_Beacon_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "sleep.h"
#include "SDK_EVAL_Config.h"
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_conf.h"
#include "GPIO.h"
#include "IAM20680_use.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_BEACON_VERSION_STRING "1.1.0"

/* Set to 1 for enabling Flags AD Type position at the beginning
   of the advertising packet */
#define ENABLE_FLAGS_AD_TYPE_AT_BEGINNING 1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



int main(void)
{
  uint8_t ret;

  /* System Init */
  SystemInit();

  /* Identify BlueNRG-1,2 platform */
//  SdkEvalIdentification();

  /* Init the UART peripheral */
//  SdkEvalComUartInit(UART_BAUDRATE);

  /* BlueNRG-1,2 stack init */
//  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
//  if (ret != BLE_STATUS_SUCCESS) {
//    SEGGER_RTT_printf(0,"Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
//    while(1);
//  }

  /* Init the BlueNRG-1,2 device */
//  Device_Init();

   LED_Init();

	/*Initialize IMU*/
   IAM20680_Init();


//    main_IMU();
//    SEGGER_RTT_printf(0, "%d.%3d    %d.%3d    %d.%3d    %d.%3d    %d.%3d   %d.%3d   \r\n",
//    				  PRINT_INT(a_x), PRINT_FLOAT(a_x), //this does not nicely print the float values, spaces print instead of 0s (cleaned data manually for test)
//    				  	  PRINT_INT(a_y), PRINT_FLOAT(a_y),
//    						  PRINT_INT(a_z), PRINT_FLOAT(a_z),
//    						  	  PRINT_INT(g_x), PRINT_FLOAT(g_x),
//    							  	  PRINT_INT(g_y), PRINT_FLOAT(g_y),
//    								  	  PRINT_INT(g_z), PRINT_FLOAT(g_z));

  }


/* Hardware Error event.
   This event is used to notify the Host that a hardware failure has occurred in the Controller.
   Hardware_Code Values:
   - 0x01: Radio state error
   - 0x02: Timer overrun error
   - 0x03: Internal queue overflow error
   After this event is recommended to force device reset. */

