/**
 * Copyright 2019 Wyres
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at
 *    http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, 
 * software distributed under the License is distributed on 
 * an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
 * either express or implied. See the License for the specific 
 * language governing permissions and limitations under the License.
*/
/**
 * Accelero API implementation for LPS22HB
 */
#include <stdint.h>

#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_i2c.h"

#include "wyres-generic/wutils.h"
#include "wyres-generic/ALTI_basic.h"

/* Only include this implementation of the altimeter api if the bsp says we have one of these */
#ifdef ALTI_LPS22HB
//****************************************************************//
//********************FUNCTIONS PROTOTYPES************************//
//**********************STRUCTS & ENUMS***************************//
//****************************************************************//

/* Uncomment the line below to expanse the "assert_param" macro in the  drivers code */
//#define USE_FULL_ASSERT_LPS22HB

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT_LPS22HB

/**
* @brief  The assert_param macro is used for function's parameters check.
* @param  expr: If expr is false, it calls assert_failed function which reports
*         the name of the source file and the source line number of the call
*         that failed. If expr is true, it returns no value.
* @retval None
*/
#define LPS22HB_assert_param(expr) ((expr) ? (void)0 : LPS22HB_assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void LPS22HB_assert_failed(uint8_t* file, uint32_t line);
#else
#define LPS22HB_assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT_LPS22HB */

  /**
  * @brief  Error type.
  */
  typedef enum {LPS22HB_OK = (uint8_t)0, LPS22HB_ERROR = !LPS22HB_OK} LPS22HB_Error_et;

  /**
  * @brief  Enable/Disable type.
  */
  typedef enum {LPS22HB_DISABLE = (uint8_t)0, LPS22HB_ENABLE = !LPS22HB_DISABLE} LPS22HB_State_et;
#define IS_LPS22HB_State(MODE) ((MODE == LPS22HB_ENABLE) || (MODE == LPS22HB_DISABLE) )

  /**
  * @brief  Bit status type.
  */
  typedef enum {LPS22HB_RESET = (uint8_t)0, LPS22HB_SET = !LPS22HB_RESET} LPS22HB_BitStatus_et;
#define IS_LPS22HB_BitStatus(MODE) ((MODE == LPS22HB_RESET) || (MODE == LPS22HB_SET))

  /*RES_CONF see LC_EN bit*/
 /**
* @brief  LPS22HB Power/Noise Mode configuration.
*/
typedef enum {
  LPS22HB_LowNoise   =  (uint8_t)0x00,       /*!< Low Noise mode */
  LPS22HB_LowPower   =  (uint8_t)0x01        /*!< Low Current mode */
} LPS22HB_PowerMode_et;

#define IS_LPS22HB_PowerMode(MODE) ((MODE == LPS22HB_LowNoise) || (MODE == LPS22HB_LowPower))

/**
* @brief  Output data rate configuration.
*/
typedef enum {

  LPS22HB_ODR_ONE_SHOT  = (uint8_t)0x00,         /*!< Output Data Rate: one shot */
  LPS22HB_ODR_1HZ       = (uint8_t)0x10,         /*!< Output Data Rate: 1Hz */
  LPS22HB_ODR_10HZ       = (uint8_t)0x20,         /*!< Output Data Rate: 10Hz */
  LPS22HB_ODR_25HZ    = (uint8_t)0x30,         /*!< Output Data Rate: 25Hz */
  LPS22HB_ODR_50HZ      = (uint8_t)0x40,          /*!< Output Data Rate: 50Hz */
  LPS22HB_ODR_75HZ      = (uint8_t)0x50          /*!< Output Data Rate: 75Hz */
} LPS22HB_Odr_et;

#define IS_LPS22HB_ODR(ODR) ((ODR == LPS22HB_ODR_ONE_SHOT) || (ODR == LPS22HB_ODR_1HZ) || \
(ODR == LPS22HB_ODR_10HZ) || (ODR == LPS22HB_ODR_25HZ)|| (ODR == LPS22HB_ODR_50HZ) || (ODR == LPS22HB_ODR_75HZ))

/**
* @brief  Low Pass Filter Cutoff Configuration.
*/
typedef enum {

  LPS22HB_ODR_9  = (uint8_t)0x00,         /*!< Filter Cutoff ODR/9 */
  LPS22HB_ODR_20 = (uint8_t)0x04          /*!< Filter Cutoff ODR/20 */
} LPS22HB_LPF_Cutoff_et;

#define IS_LPS22HB_LPF_Cutoff(CUTOFF) ((CUTOFF == LPS22HB_ODR_9) || (CUTOFF == LPS22HB_ODR_20) )

/**
* @brief  Block data update.
*/

typedef enum {
  LPS22HB_BDU_CONTINUOUS_UPDATE     =  (uint8_t)0x00,  /*!< Data updated continuously */
  LPS22HB_BDU_NO_UPDATE             =  (uint8_t)0x02   /*!< Data updated after a read operation */
} LPS22HB_Bdu_et;
#define IS_LPS22HB_BDUMode(MODE) ((MODE == LPS22HB_BDU_CONTINUOUS_UPDATE) || (MODE == LPS22HB_BDU_NO_UPDATE))

/**
* @brief  LPS22HB Spi Mode configuration.
*/
typedef enum {
  LPS22HB_SPI_4_WIRE   =  (uint8_t)0x00,
  LPS22HB_SPI_3_WIRE   =  (uint8_t)0x01
} LPS22HB_SPIMode_et;

#define IS_LPS22HB_SPIMode(MODE) ((MODE == LPS22HB_SPI_4_WIRE) || (MODE == LPS22HB_SPI_3_WIRE))

/**
* @brief  LPS22HB Interrupt Active Level Configuration (on High or Low)
*/
typedef enum
{
  LPS22HB_ActiveHigh = (uint8_t)0x00,
  LPS22HB_ActiveLow  = (uint8_t)0x80
}LPS22HB_InterruptActiveLevel_et;
#define IS_LPS22HB_InterruptActiveLevel(MODE) ((MODE == LPS22HB_ActiveHigh) || (MODE == LPS22HB_ActiveLow))

/**
* @brief  LPS22HB Push-pull/Open Drain selection on Interrupt pads.
*/
typedef enum
{
  LPS22HB_PushPull = (uint8_t)0x00,
  LPS22HB_OpenDrain  = (uint8_t)0x40
}LPS22HB_OutputType_et;
#define IS_LPS22HB_OutputType(MODE) ((MODE == LPS22HB_PushPull) || (MODE == LPS22HB_OpenDrain))

/**
* @brief  Data Signal on INT pad control bits.
*/
typedef enum
{
  LPS22HB_DATA = (uint8_t)0x00,
  LPS22HB_P_HIGH = (uint8_t)0x01,
  LPS22HB_P_LOW = (uint8_t)0x02,
  LPS22HB_P_LOW_HIGH = (uint8_t)0x03
}LPS22HB_OutputSignalConfig_et;
#define IS_LPS22HB_OutputSignal(MODE) ((MODE == LPS22HB_DATA) || (MODE == LPS22HB_P_HIGH)||\
(MODE == LPS22HB_P_LOW) || (MODE == LPS22HB_P_LOW_HIGH))

/**
* @brief  LPS22HB Interrupt Differential Status.
*/
typedef struct
{
  uint8_t PH;          /*!< High Differential Pressure event occured */
  uint8_t PL;          /*!< Low Differential Pressure event occured */
  uint8_t IA;          /*!< One or more interrupt events have been  generated.Interrupt Active */
  uint8_t BOOT;        /*!< i '1' indicates that the Boot (Reboot) phase is running */
}LPS22HB_InterruptDiffStatus_st;

/**
* @brief  LPS22HB Pressure and Temperature data status.
*/
typedef struct
{
  uint8_t TempDataAvailable;           /*!< Temperature data available bit */
  uint8_t PressDataAvailable;          /*!< Pressure data available bit */
  uint8_t TempDataOverrun;             /*!< Temperature data over-run bit */
  uint8_t PressDataOverrun;            /*!< Pressure data over-run bit */
}LPS22HB_DataStatus_st;

/**
* @brief  LPS22HB Clock Tree  configuration.
*/
typedef enum {
  LPS22HB_CTE_NotBalanced   =  (uint8_t)0x00,
  LPS22HB_CTE_Balanced   =  (uint8_t)0x20
} LPS22HB_CTE_et;
#define IS_LPS22HB_CTE(MODE) ((MODE == LPS22HB_CTE_NotBalanced) || (MODE == LPS22HB_CTE_Balanced))

/**
* @brief  LPS22HB Fifo Mode.
*/
typedef enum {
  LPS22HB_FIFO_BYPASS_MODE             	      = (uint8_t)0x00,	  /*!< The FIFO is disabled and empty. The pressure is read directly*/
  LPS22HB_FIFO_MODE                           = (uint8_t)0x20,    /*!< Stops collecting data when full */
  LPS22HB_FIFO_STREAM_MODE                    = (uint8_t)0x40,    /*!< Keep the newest measurements in the FIFO*/
  LPS22HB_FIFO_TRIGGER_STREAMTOFIFO_MODE      = (uint8_t)0x60,    /*!< STREAM MODE until trigger deasserted, then change to FIFO MODE*/
  LPS22HB_FIFO_TRIGGER_BYPASSTOSTREAM_MODE    = (uint8_t)0x80,    /*!< BYPASS MODE until trigger deasserted, then STREAM MODE*/
  LPS22HB_FIFO_TRIGGER_BYPASSTOFIFO_MODE      = (uint8_t)0xE0     /*!< BYPASS mode until trigger deasserted, then FIFO MODE*/
} LPS22HB_FifoMode_et;
#define IS_LPS22HB_FifoMode(MODE) ((MODE == LPS22HB_FIFO_BYPASS_MODE) || (MODE ==LPS22HB_FIFO_MODE)||\
(MODE == LPS22HB_FIFO_STREAM_MODE) || (MODE == LPS22HB_FIFO_TRIGGER_STREAMTOFIFO_MODE)||\
  (MODE == LPS22HB_FIFO_TRIGGER_BYPASSTOSTREAM_MODE) ||  (MODE == LPS22HB_FIFO_TRIGGER_BYPASSTOFIFO_MODE))

/**
* @brief  LPS22HB Fifo Satus.
*/
typedef struct {
  uint8_t FIFO_LEVEL;          /*!< FIFO Stored data level: 00000: FIFO empty; 10000: FIFO is FULL and ha 32 unread samples  */
  uint8_t FIFO_EMPTY;          /*!< Empty FIFO Flag .1 FIFO is empty (see FIFO_level)	*/
  uint8_t FIFO_FULL;          /*!< Full FIFO flag.1 FIFO is Full (see FIFO_level)	*/
  uint8_t FIFO_OVR;           /*!< Overrun bit status. 1 FIFO is full and at least one sample in the FIFO has been overwritten */
  uint8_t FIFO_FTH;            /*!< FIFO Threshold (Watermark) Status. 1 FIFO filling is equal or higher then FTH (wtm) level.*/
}LPS22HB_FifoStatus_st;

/**
* @brief  LPS22HB Configuration structure definition.
*/
typedef struct
{
  LPS22HB_PowerMode_et   PowerMode;                    /*!< Enable Low Current Mode (low Power) or Low Noise Mode*/
  LPS22HB_Odr_et         OutputDataRate;                /*!< Output Data Rate */
  LPS22HB_Bdu_et         BDU;                	        /*!< Enable to inhibit the output registers update between the reading of upper and lower register parts.*/
  LPS22HB_State_et   	 LowPassFilter;		        /*!< Enable/ Disable Low Pass Filter */
  LPS22HB_LPF_Cutoff_et  LPF_Cutoff;                    /*!< Low Pass Filter Configuration */
  LPS22HB_SPIMode_et 	 Sim;  			        /*!< SPI Serial Interface Mode selection */
  LPS22HB_State_et       IfAddInc;                       /*!< Enable/Disable Register address automatically inceremented during a multiple byte access */
}LPS22HB_ConfigTypeDef_st;

  /**
* @brief  LPS22HB Interrupt structure definition .
*/
typedef struct {
  LPS22HB_InterruptActiveLevel_et 		INT_H_L;                /*!< Interrupt active high, low. Default value: 0 */
  LPS22HB_OutputType_et 			PP_OD; 		        /*!< Push-pull/open drain selection on interrupt pads. Default value: 0 */
  LPS22HB_OutputSignalConfig_et 		OutputSignal_INT;	/*!< Data signal on INT Pad: Data,Pressure High, Preessure Low,P High or Low*/
  LPS22HB_State_et                              DRDY;                   /*!< Enable/Disable Data Ready Interrupt on INT_DRDY Pin*/
  LPS22HB_State_et                              FIFO_OVR;                /*!< Enable/Disable FIFO Overrun Interrupt on INT_DRDY Pin*/
  LPS22HB_State_et                              FIFO_FTH;                /*!< Enable/Disable FIFO threshold (Watermark) interrupt on INT_DRDY pin.*/
  LPS22HB_State_et                              FIFO_FULL;               /*!< Enable/Disable FIFO FULL interrupt on INT_DRDY pin.*/
  LPS22HB_State_et				LatchIRQ;		/*!< Latch Interrupt request in to INT_SOURCE reg*/
  int16_t 					THS_threshold;		/*!< Threshold value for pressure interrupt generation*/
  LPS22HB_State_et       AutoRifP;                                      /*!< Enable/Disable  AutoRifP function */
  LPS22HB_State_et       AutoZero;                                      /*!< Enable/Disable  AutoZero function */
}LPS22HB_InterruptTypeDef_st;

/**
* @brief  LPS22HB FIFO structure definition.
*/
typedef struct {
  LPS22HB_FifoMode_et 			FIFO_MODE;               /*!< Fifo Mode Selection */
  LPS22HB_State_et			WTM_INT; 		/*!< Enable/Disable the watermark interrupt*/
  uint8_t 				WTM_LEVEL;		/*!< FIFO threshold/Watermark level selection*/
}LPS22HB_FIFOTypeDef_st;
#define IS_LPS22HB_WtmLevel(LEVEL) ((LEVEL > 0) && (LEVEL <=31))

/**
* @brief  LPS22HB Measure Type definition.
*/
typedef struct {
  int16_t Tout;
  int32_t Pout;
}LPS22HB_MeasureTypeDef_st;

/**
* @brief  LPS22HB Driver Version Info structure definition.
*/
typedef struct {
  uint8_t   Major;
  uint8_t   Minor;
  uint8_t Point;
}LPS22HB_DriverVersion_st;

/**
* @brief Data structure contening Pressure Temperature and altitude
*/
 typedef struct AltimeterData_s
{
    uint32_t Pressure;         	//! Real time Pressure sample
    uint32_t Altitude;         	//! Real time Altitude sample
    int16_t Temperature;       //! Real time Temperature sample
}AltimeterData_t;

/**
* @brief  Bitfield positioning.
*/
#define LPS22HB_BIT(x) ((uint8_t)x)

/**
* @brief  Set the LPS22HB driver version.
*/
#define LPS22HB_DriverVersion_Major (uint8_t)1
#define LPS22HB_DriverVersion_Minor (uint8_t)0
#define LPS22HB_DriverVersion_Point (uint8_t)0

/**
* @brief Device Identification register.
* \code
* Read
* Default value: 0xB1
* 7:0 This read-only register contains the device identifier that, for LPS22HB, is set to B1h.
* \endcode
*/
#define LPS22HB_WHO_AM_I_REG         (uint8_t)0x0F
/**
* @brief Device Identification value.
*/
#define LPS22HB_WHO_AM_I_VAL         (uint8_t)0xB1
/**
* @brief Reference Pressure  Register(LSB data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL7-0: Lower part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define LPS22HB_REF_P_XL_REG         (uint8_t)0x15
/**
* @brief Reference Pressure Register (Middle data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL15-8: Middle part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define LPS22HB_REF_P_L_REG          (uint8_t)0x16
/**
* @brief Reference Pressure Register (MSB data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL23-16 Higest part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define LPS22HB_REF_P_H_REG          (uint8_t)0x17
/**
* @brief Pressure and temperature resolution mode Register
* \code
* Read/write
* Default value: 0x05
* 7:2 These bits must be set to 0 for proper operation of the device
* 1: Reserved
* 0 LC_EN: Low Current Mode Enable. Default 0
* \endcode
*/
#define LPS22HB_RES_CONF_REG     (uint8_t)0x1A

#define LPS22HB_LCEN_MASK        (uint8_t)0x01
/**
* @brief Control Register 1
* \code
* Read/write
* Default value: 0x00
* 7: This bit must be set to 0 for proper operation of the device
* 6:4 ODR2, ODR1, ODR0: output data rate selection.Default 000
*     ODR2  | ODR1  | ODR0  | Pressure output data-rate(Hz)  | Pressure output data-rate(Hz)
*   ----------------------------------------------------------------------------------
*      0    |  0    |  0    |         one shot               |         one shot
*      0    |  0    |  1    |            1                   |            1
*      0    |  1    |  0    |            10                  |           10
*      0    |  1    |  1    |            25                  |           25
*      1    |  0    |  0    |            50                  |           50
*      1    |  0    |  1    |            75                  |         75
*      1    |  1    |  0    |         Reserved               |         Reserved
*      1    |  1    |  1    |         Reserved               |         Reserved
*
* 3 EN_LPFP: Enable Low Pass filter on Pressure data. Default value:0
* 2:LPF_CFG Low-pass configuration register. (0: Filter cutoff is ODR/9; 1: filter cutoff is ODR/20)
* 1 BDU: block data update. 0 - continuous update; 1 - output registers not updated until MSB and LSB reading.
* 0 SIM: SPI Serial Interface Mode selection. 0 - SPI 4-wire; 1 - SPI 3-wire
* \endcode
*/
#define LPS22HB_CTRL_REG1      (uint8_t)0x10

#define LPS22HB_ODR_MASK                (uint8_t)0x70
#define LPS22HB_LPFP_MASK               (uint8_t)0x08
#define LPS22HB_LPFP_CUTOFF_MASK        (uint8_t)0x04
#define LPS22HB_BDU_MASK                (uint8_t)0x02
#define LPS22HB_SIM_MASK                (uint8_t)0x01

#define LPS22HB_LPFP_BIT    LPS22HB_BIT(3)


/**
* @brief Control  Register 2
* \code
* Read/write
* Default value: 0x10
* 7 BOOT:  Reboot memory content. 0: normal mode; 1: reboot memory content. Self-clearing upon completation
* 6 FIFO_EN: FIFO Enable. 0: disable; 1:  enable
* 5 STOP_ON_FTH: Stop on FIFO Threshold  FIFO Watermark level use. 0: disable; 1: enable
* 4 IF_ADD_INC: Register address automatically incrementeed during a multiple byte access with a serial interface (I2C or SPI). Default value 1.( 0: disable; 1: enable)
* 3 I2C DIS:  Disable I2C interface 0: I2C Enabled; 1: I2C disabled
* 2 SWRESET: Software reset. 0: normal mode; 1: SW reset. Self-clearing upon completation
* 1 AUTO_ZERO: Autozero enable. 0: normal mode; 1: autozero enable.
* 0 ONE_SHOT: One shot enable. 0: waiting for start of conversion; 1: start for a new dataset
* \endcode
*/
#define LPS22HB_CTRL_REG2      (uint8_t)0x11

#define LPS22HB_BOOT_BIT       LPS22HB_BIT(7)
#define LPS22HB_FIFO_EN_BIT    LPS22HB_BIT(6)
#define LPS22HB_WTM_EN_BIT     LPS22HB_BIT(5)
#define LPS22HB_ADD_INC_BIT    LPS22HB_BIT(4)
#define LPS22HB_I2C_BIT        LPS22HB_BIT(3)
#define LPS22HB_SW_RESET_BIT   LPS22HB_BIT(2)

#define LPS22HB_FIFO_EN_MASK   (uint8_t)0x40
#define LPS22HB_WTM_EN_MASK    (uint8_t)0x20
#define LPS22HB_ADD_INC_MASK   (uint8_t)0x10
#define LPS22HB_I2C_MASK       (uint8_t)0x08
#define LPS22HB_ONE_SHOT_MASK  (uint8_t)0x01


/**
* @brief CTRL Reg3 Interrupt Control Register
* \code
* Read/write
* Default value: 0x00
* 7 INT_H_L: Interrupt active high, low. 0:active high; 1: active low.
* 6 PP_OD: Push-Pull/OpenDrain selection on interrupt pads. 0: Push-pull; 1: open drain.
* 5 F_FSS5: FIFO full flag on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 4 F_FTH: FIFO threshold (watermark) status on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 3 F_OVR: FIFO overrun interrupt on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 2 DRDY: Data-ready signal on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 1:0 INT_S2, INT_S1: data signal on INT pad control bits.
*    INT_S2  | INT_S1  | INT pin
*   ------------------------------------------------------
*        0       |      0      |     Data signal( in order of priority:PTH_DRDY or F_FTH or F_OVR_or F_FSS5
*        0       |      1      |     Pressure high (P_high)
*        1       |      0      |     Pressure low (P_low)
*        1       |      1      |     P_low OR P_high
* \endcode
*/
#define LPS22HB_CTRL_REG3      (uint8_t)0x12

#define LPS22HB_PP_OD_BIT       LPS22HB_BIT(6)
#define LPS22HB_FIFO_FULL_BIT   LPS22HB_BIT(5)
#define LPS22HB_FIFO_FTH_BIT    LPS22HB_BIT(4)
#define LPS22HB_FIFO_OVR_BIT    LPS22HB_BIT(3)
#define LPS22HB_DRDY_BIT        LPS22HB_BIT(2)


#define LPS22HB_INT_H_L_MASK            (uint8_t)0x80
#define LPS22HB_PP_OD_MASK              (uint8_t)0x40
#define LPS22HB_FIFO_FULL_MASK          (uint8_t)0x20
#define LPS22HB_FIFO_FTH_MASK           (uint8_t)0x10
#define LPS22HB_FIFO_OVR_MASK           (uint8_t)0x08
#define LPS22HB_DRDY_MASK               (uint8_t)0x04
#define LPS22HB_INT_S12_MASK            (uint8_t)0x03


/**
* @brief Interrupt Differential configuration Register
* \code
* Read/write
* Default value: 0x00.
* 7 AUTORIFP: AutoRifP Enable ??
* 6 RESET_ARP: Reset AutoRifP function
* 4 AUTOZERO: Autozero enabled
* 5 RESET_AZ: Reset Autozero Function
* 3 DIFF_EN: Interrupt generation enable
* 2 LIR: Latch Interrupt request into INT_SOURCE register. 0 - interrupt request not latched; 1 - interrupt request latched
* 1 PL_E: Enable interrupt generation on differential pressure low event. 0 - disable; 1 - enable
* 0 PH_E: Enable interrupt generation on differential pressure high event. 0 - disable; 1 - enable
* \endcode
*/
#define LPS22HB_INTERRUPT_CFG_REG  (uint8_t)0x0B

#define LPS22HB_DIFF_EN_BIT       LPS22HB_BIT(3)
#define LPS22HB_LIR_BIT           LPS22HB_BIT(2)
#define LPS22HB_PLE_BIT           LPS22HB_BIT(1)
#define LPS22HB_PHE_BIT           LPS22HB_BIT(0)

#define LPS22HB_AUTORIFP_MASK     (uint8_t)0x80
#define LPS22HB_RESET_ARP_MASK    (uint8_t)0x40
#define LPS22HB_AUTOZERO_MASK     (uint8_t)0x20
#define LPS22HB_RESET_AZ_MASK     (uint8_t)0x10
#define LPS22HB_DIFF_EN_MASK      (uint8_t)0x08
#define LPS22HB_LIR_MASK          (uint8_t)0x04
#define LPS22HB_PLE_MASK          (uint8_t)0x02
#define LPS22HB_PHE_MASK          (uint8_t)0x01


/**
* @brief Interrupt source Register (It is cleared by reading it)
* \code
* Read
* Default value: ----.
* 7 BOOT_STATUS:  If 1 indicates that the Boot (Reboot) phase is running.
* 6:3 Reserved: Keep these bits at 0
* 2 IA: Interrupt Active.0: no interrupt has been generated; 1: one or more interrupt events have been generated.
* 1 PL: Differential pressure Low. 0: no interrupt has been generated; 1: Low differential pressure event has occurred.
* 0 PH: Differential pressure High. 0: no interrupt has been generated; 1: High differential pressure event has occurred.
* \endcode
*/
#define LPS22HB_INTERRUPT_SOURCE_REG   (uint8_t)0x25

#define LPS22HB_BOOT_STATUS_BIT        LPS22HB_BIT(7)
#define LPS22HB_IA_BIT                 LPS22HB_BIT(2)
#define LPS22HB_PL_BIT                 LPS22HB_BIT(1)
#define LPS22HB_PH_BIT                 LPS22HB_BIT(0)

#define LPS22HB_BOOT_STATUS_MASK      (uint8_t)0x80
#define LPS22HB_IA_MASK               (uint8_t)0x04
#define LPS22HB_PL_MASK               (uint8_t)0x02
#define LPS22HB_PH_MASK               (uint8_t)0x01


/**
* @brief  Status Register
* \code
* Read
* Default value: ---
* 7:6 Reserved: 0
* 5 T_OR: Temperature data overrun. 0: no overrun has occurred; 1: a new data for temperature has overwritten the previous one.
* 4 P_OR: Pressure data overrun. 0: no overrun has occurred; 1: new data for pressure has overwritten the previous one.
* 3:2 Reserved: 0
* 1 T_DA: Temperature data available. 0: new data for temperature is not yet available; 1: new data for temperature is available.
* 0 P_DA: Pressure data available. 0: new data for pressure is not yet available; 1: new data for pressure is available.
* \endcode
*/
#define LPS22HB_STATUS_REG         (uint8_t)0x27

#define LPS22HB_TOR_BIT            LPS22HB_BIT(5)
#define LPS22HB_POR_BIT            LPS22HB_BIT(4)
#define LPS22HB_TDA_BIT            LPS22HB_BIT(1)
#define LPS22HB_PDA_BIT            LPS22HB_BIT(0)

#define LPS22HB_TOR_MASK           (uint8_t)0x20
#define LPS22HB_POR_MASK           (uint8_t)0x10
#define LPS22HB_TDA_MASK           (uint8_t)0x02
#define LPS22HB_PDA_MASK           (uint8_t)0x01


/**
* @brief  Pressure data (LSB) register.
* \code
* Read
* Default value: 0x00.(To be verified)
* POUT7 - POUT0: Pressure data LSB (2's complement).
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/
#define LPS22HB_PRESS_OUT_XL_REG        (uint8_t)0x28
/**
* @brief  Pressure data (Middle part) register.
* \code
* Read
* Default value: 0x80.
* POUT15 - POUT8: Pressure data middle part (2's complement).
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/
#define LPS22HB_PRESS_OUT_L_REG        (uint8_t)0x29

/**
* @brief  Pressure data (MSB) register.
* \code
* Read
* Default value: 0x2F.
* POUT23 - POUT16: Pressure data MSB (2's complement).
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/
#define LPS22HB_PRESS_OUT_H_REG        (uint8_t)0x2A

/**
* @brief  Temperature data (LSB) register.
* \code
* Read
* Default value: 0x00.
* TOUT7 - TOUT0: temperature data LSB.
* Tout(degC)=TEMP_OUT/100
* \endcode
*/
#define LPS22HB_TEMP_OUT_L_REG         (uint8_t)0x2B

/**
* @brief  Temperature data (MSB) register.
* \code
* Read
* Default value: 0x00.
* TOUT15 - TOUT8: temperature data MSB.
* Tout(degC)=TEMP_OUT/100
* \endcode
*/
#define LPS22HBH_TEMP_OUT_H_REG         (uint8_t)0x2C

/**
* @brief Threshold pressure (LSB) register.
* \code
* Read/write
* Default value: 0x00.
* 7:0 THS7-THS0: LSB Threshold pressure Low part of threshold value for pressure interrupt
* generation. The complete threshold value is given by THS_P_H & THS_P_L and is
* expressed as unsigned number. P_ths(hPA)=(THS_P_H & THS_P_L)[dec]/16.
* \endcode
*/
#define LPS22HB_THS_P_LOW_REG           (uint8_t)0x0C

/**
* @brief Threshold pressure (MSB)
* \code
* Read/write
* Default value: 0x00.
* 7:0 THS15-THS8: MSB Threshold pressure. High part of threshold value for pressure interrupt
* generation. The complete threshold value is given by THS_P_H & THS_P_L and is
* expressed as unsigned number. P_ths(mbar)=(THS_P_H & THS_P_L)[dec]/16.
* \endcode
*/
#define LPS22HB_THS_P_HIGH_REG         (uint8_t)0x0D

/**
* @brief FIFO control register
* \code
* Read/write
* Default value: 0x00
* 7:5 F_MODE2, F_MODE1, F_MODE0: FIFO mode selection.
*     FM2   | FM1   | FM0   |    FIFO MODE
*   ---------------------------------------------------
*      0    |  0    |  0    | BYPASS MODE
*      0    |  0    |  1    | FIFO MODE. Stops collecting data when full
*      0    |  1    |  0    | STREAM MODE: Keep the newest measurements in the FIFO
*      0    |  1    |  1    | STREAM MODE until trigger deasserted, then change to FIFO MODE
*      1    |  0    |  0    | BYPASS MODE until trigger deasserted, then STREAM MODE
*      1    |  0    |  1    | Reserved for future use
*      1    |  1    |  0    | Reserved
*      1    |  1    |  1    | BYPASS mode until trigger deasserted, then FIFO MODE
*
* 4:0 WTM_POINT4-0 : FIFO Watermark level selection (0-31)
*/
#define LPS22HB_CTRL_FIFO_REG          (uint8_t)0x14

#define LPS22HB_FIFO_MODE_MASK        (uint8_t)0xE0
#define LPS22HB_WTM_POINT_MASK        (uint8_t)0x1F


/**
* @brief FIFO Status register
* \code
* Read
* Default value: ----
* 7 FTH_FIFO: FIFO threshold status. 0:FIFO filling is lower than FTH level; 1: FIFO is equal or higher than FTH level.
* 6 OVR: Overrun bit status. 0 - FIFO not full; 1 -FIFO is full and at least one sample in the FIFO has been overwritten.
* 5:0 FSS: FIFO Stored data level. 000000: FIFO empty, 100000: FIFO is full and has 32 unread samples.
* \endcode
*/
#define LPS22HB_STATUS_FIFO_REG        (uint8_t)0x26

#define LPS22HB_FTH_FIFO_BIT          LPS22HB_BIT(7)
#define LPS22HB_OVR_FIFO_BIT          LPS22HB_BIT(6)

#define LPS22HB_FTH_FIFO_MASK         (uint8_t)0x80
#define LPS22HB_OVR_FIFO_MASK         (uint8_t)0x40
#define LPS22HB_LEVEL_FIFO_MASK       (uint8_t)0x3F
#define LPS22HB_FIFO_EMPTY            (uint8_t)0x00
#define LPS22HB_FIFO_FULL             (uint8_t)0x20


/**
* @brief Pressure offset register  (LSB)
* \code
* Read/write
* Default value: 0x00
* 7:0 RPDS7-0:Pressure Offset for 1 point calibration (OPC) after soldering.
* This register contains the low part of the pressure offset value after soldering,for
* differential pressure computing. The complete value is given by RPDS_L & RPDS_H
* and is expressed as signed 2 complement value.
* \endcode
*/
#define LPS22HB_RPDS_L_REG        (uint8_t)0x18

/**
* @brief Pressure offset register (MSB)
* \code
* Read/write
* Default value: 0x00
* 7:0 RPDS15-8:Pressure Offset for 1 point calibration  (OPC) after soldering.
* This register contains the high part of the pressure offset value after soldering (see description RPDS_L)
* \endcode
*/
#define LPS22HB_RPDS_H_REG        (uint8_t)0x19


/**
* @brief Clock Tree Configuration register
* \code
* Read/write
* Default value: 0x00
* 7:6 Reserved.
* 5: CTE: Clock Tree Enhancement
* \endcode
*/

#define LPS22HB_CLOCK_TREE_CONFIGURATION        (uint8_t)0x43

#define LPS22HB_CTE_MASK           (uint8_t)0x20


/* Exported Functions -------------------------------------------------------------*/
/** @defgroup LPS22HB_Exported_Functions
* @{
*/

/**
* @brief  Init the HAL layer.
* @param  None
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
//#define LPS22HB_HalInit  (LPS22HB_Error_et)HAL_Init_I2C

/**
* @brief  DeInit the HAL layer.
* @param  None
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
//#define LPS22HB_HalDeInit  (LPS22HB_Error_et)HAL_DeInit_I2C

/**
* @brief  Read LPS22HB Registers
* @param  uint32_t RegAddr:       address of the first register to read
* @param  uint32_t NumByteToRead:   number of registers to read
* @param  uint8_t *Data:          pointer to the destination data buffer
* @retval LPS22HB_ERROR or LPS22HB_OK
*/
// the user must redefine the proper LPS22HB_ReadReg
static LPS22HB_Error_et LPS22HB_ReadReg(uint8_t regAddr, uint32_t numByteToRead, uint8_t* data);

/**
* @brief  Write LPS22HB Registers
* @param  uint32_t RegAddr:      address of the register to write
* @param  uint8_t *Data:         pointer to the source data buffer
* @param  uint32_t NumByteToWrite:           Number of bytes to write
* @retval LPS22HB_ERROR or LPS22HB_OK
*/
// the user must redefine the proper LPS22HB_WriteReg
static LPS22HB_Error_et LPS22HB_WriteReg(uint8_t regAddr, uint32_t numByteToWrite, uint8_t* data);


/**
* @brief  Get the LPS22HB driver version.
* @param  None
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_DriverVersion(LPS22HB_DriverVersion_st *Version);

/**
* @brief  Initialization function for LPS22HB.
*         This function make a memory boot.
*         Init the sensor with a standard basic confifuration.
*         Low Power, ODR 25 Hz, Low Pass Filter disabled; BDU enabled; I2C enabled;
*         NO FIFO; NO Interrupt Enabled.
* @param  None.
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Init(void);

/**
* @brief  DeInit the LPS2Hb driver.
* @param  None
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_DeInit(void);

/**
* @brief  Read identification code by WHO_AM_I register
* @param  Buffer to empty by Device identification Value.
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_DeviceID(uint8_t* deviceid);

/**
* @brief  Set LPS22HB Low Power or Low Noise Mode Configuration
* @param  LPS22HB_LowNoise or LPS22HB_LowPower mode
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_PowerMode(LPS22HB_PowerMode_et mode);

/**
* @brief  Get LPS22HB Power Mode
* @param   Buffer to empty with Mode: Low Noise or Low Current
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Get_PowerMode(LPS22HB_PowerMode_et* mode);

/**
* @brief  Set LPS22HB Output Data Rate
* @param  Output Data Rate
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_Odr(LPS22HB_Odr_et odr);

/**
* @brief  Get LPS22HB Output Data Rate
* @param  Buffer to empty with Output Data Rate
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Odr(LPS22HB_Odr_et* odr);

/**
* @brief  Enable/Disale low-pass filter on LPS22HB pressure data
* @param  state: enable or disable
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_LowPassFilter(LPS22HB_State_et state);

/**
* @brief  Set low-pass filter cutoff configuration on LPS22HB pressure data
* @param Filter Cutoff ODR/9 or ODR/20
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_LowPassFilterCutoff(LPS22HB_LPF_Cutoff_et cutoff);

/**
* @brief  Set Block Data Update mode
* @param  LPS22HB_BDU_CONTINUOS_UPDATE/ LPS22HB_BDU_NO_UPDATE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_Bdu(LPS22HB_Bdu_et bdu);

/**
* @brief  Get Block Data Update mode
* @param  Buffer to empty whit the bdu mode read from sensor
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Bdu(LPS22HB_Bdu_et* bdu);

/**
* @brief  Set SPI mode: 3 Wire Interface OR 4 Wire Interface
* @param  LPS22HB_SPI_4_WIRE/LPS22HB_SPI_3_WIRE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_SpiInterface(LPS22HB_SPIMode_et spimode);

/**
* @brief  Get SPI mode: 3 Wire Interface OR 4 Wire Interface
* @param  buffer to empty with SPI mode
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_SpiInterface(LPS22HB_SPIMode_et* spimode);

/**
* @brief Software Reset
* @param  void
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_SwReset(void);

/**
* @brief Reboot Memory Content.
* @param  void
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_MemoryBoot(void);

/**
* @brief Software Reset ann BOOT
* @param  void
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_SwResetAndMemoryBoot(void);

/**
* @brief  Enable or Disable FIFO
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FifoModeUse(LPS22HB_State_et status);

/**
* @brief  Enable or Disable FIFO Watermark level use. Stop on FIFO Threshold
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FifoWatermarkLevelUse(LPS22HB_State_et status);

/**
* @brief  Enable or Disable the Automatic increment register address during a multiple byte access with a serial interface (I2C or SPI)
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE. Default is LPS22HB_ENABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_AutomaticIncrementRegAddress(LPS22HB_State_et status);

/**
* @brief  Set One Shot bit to start a new conversion (ODR mode has to be 000)
* @param  void
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_StartOneShotMeasurement(void);

/**
* @brief  Enable/Disable I2C
* @param  State. Enable (reset bit)/ Disable (set bit)
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_I2C(LPS22HB_State_et i2cstate);

/*CTRL_REG3 Interrupt Control*/
/**
* @brief  Set Interrupt Active on High or Low Level
* @param  LPS22HB_ActiveHigh/LPS22HB_ActiveLow
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_InterruptActiveLevel(LPS22HB_InterruptActiveLevel_et mode);

/**
* @brief  Set Push-pull/open drain selection on interrupt pads.
* @param  LPS22HB_PushPull/LPS22HB_OpenDrain
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_InterruptOutputType(LPS22HB_OutputType_et output);

/**
* @brief  Set Data signal on INT1 pad control bits.
* @param  LPS22HB_DATA,LPS22HB_P_HIGH_LPS22HB_P_LOW,LPS22HB_P_LOW_HIGH
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_InterruptControlConfig(LPS22HB_OutputSignalConfig_et config);

/**
* @brief   Enable/Disable Data-ready signal on INT_DRDY pin.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_DRDYInterrupt(LPS22HB_State_et status);

 /**
* @brief   Enable/Disable FIFO overrun interrupt on INT_DRDY pin.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FIFO_OVR_Interrupt(LPS22HB_State_et status);

 /**
* @brief   Enable/Disable FIFO threshold (Watermark) interrupt on INT_DRDY pin.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FIFO_FTH_Interrupt(LPS22HB_State_et status);

/**
* @brief   Enable/Disable FIFO FULL interrupt on INT_DRDY pin.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FIFO_FULL_Interrupt(LPS22HB_State_et status);

/**
* @brief   Enable AutoRifP function
* @param   none
* @detail When this function is enabled, an internal register is set with the current pressure values
*         and the content is subtracted from the pressure output value and result is used for the interrupt generation.
*        the AutoRifP is slf creared.
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_AutoRifP(void);

/**
* @brief   Disable AutoRifP
* @param   none
* @detail  the RESET_ARP bit is used to disable the AUTORIFP function. This bis i is selfdleared
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_ResetAutoRifP(void);

/**?????
* @brief  Set AutoZero Function bit
* @detail When set to ‘1’, the actual pressure output is copied in the REF_P reg (@0x15..0x17)
* @param  None
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_AutoZeroFunction(void);

/**???
* @brief  Set ResetAutoZero Function bit
* @details REF_P reg (@0x015..17) set pressure reference to default value RPDS reg (0x18/19).
* @param  None
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_ResetAutoZeroFunction(void);

/**
* @brief  Enable/ Disable the computing of differential pressure output (Interrupt Generation)
* @param  LPS22HB_ENABLE,LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_InterruptDifferentialGeneration(LPS22HB_State_et diff_en) ;

/**
* @brief  Get the DIFF_EN bit value
* @param  buffer to empty with the read value of DIFF_EN bit
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_InterruptDifferentialGeneration(LPS22HB_State_et* diff_en);

/**
* @brief  Latch Interrupt request to the INT_SOURCE register.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_LatchInterruptRequest(LPS22HB_State_et status);

/**
* @brief  Enable\Disable Interrupt Generation on differential pressure Low event
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_PLE(LPS22HB_State_et status);

/**
* @brief  Enable\Disable Interrupt Generation on differential pressure High event
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_PHE(LPS22HB_State_et status);

/**
* @brief   Get the Interrupt Generation on differential pressure status event and the Boot Status.
* @detail  The INT_SOURCE register is cleared by reading it.
* @param   Status Event Flag: BOOT, PH,PL,IA
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_InterruptDifferentialEventStatus(LPS22HB_InterruptDiffStatus_st* interruptsource);

/**
* @brief  Get the status of Pressure and Temperature data
* @param  Data Status Flag:  TempDataAvailable, TempDataOverrun, PressDataAvailable, PressDataOverrun
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_DataStatus(LPS22HB_DataStatus_st* datastatus);

/**
* @brief  Get the LPS22HB raw presure value
* @param  The buffer to empty with the pressure raw value
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Get_RawPressure(int32_t *raw_press);

/**
* @brief  Get the LPS22HB Pressure value in hPA.
* @param  The buffer to empty with the pressure value that must be divided by 100 to get the value in hPA
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Get_Pressure(int32_t* Pout);

/**
* @brief  Read LPS22HB output register, and calculate the raw temperature.
* @param  The buffer to empty with the temperature raw value
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Get_RawTemperature(int16_t *raw_data);

/**
* @brief  Read the Temperature value in °C.
* @param  The buffer to empty with the temperature value that must be divided by 10 to get the value in ['C]
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Get_Temperature(int16_t* Tout);

/**
* @brief  Get the threshold value used for pressure interrupt generation.
* @param  The buffer to empty with the temperature value
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Get_PressureThreshold(int16_t *P_ths);

/**
* @brief  Set the threshold value used for pressure interrupt generation.
* @param  The buffer to empty with the temperature value
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_PressureThreshold(int16_t P_ths);

/**
* @brief  Set Fifo Mode.
* @param  Fifo Mode struct
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FifoMode(LPS22HB_FifoMode_et fifomode);
/**
* @brief  Get Fifo Mode.
* @param  Buffer to empty with fifo mode value
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoMode(LPS22HB_FifoMode_et* fifomode);

/**
* @brief  Set Fifo Watermark Level.
* @param  Watermark level value [0 31]
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FifoWatermarkLevel(uint8_t wtmlevel);

/**
* @brief   Get FIFO Watermark Level
* @param   buffer to empty with watermak level[0,31] value read from sensor
* @retval  Status [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoWatermarkLevel(uint8_t *wtmlevel);

/**
* @brief  Get Fifo Status.
* @param  Buffer to empty with fifo status
* @retval Status [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoStatus(LPS22HB_FifoStatus_st* status);

/**
* @brief  Get the reference pressure after soldering for computing differential pressure (hPA)
* @param buffer to empty with the he pressure value (hPA)
* @retval  Status [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_PressureOffsetValue(int16_t *pressoffset);

/**
* @brief  Get the Reference Pressure value
* @detail  It is a 24-bit data added to the sensor output measurement to detect a measured pressure beyond programmed limits.
* @param  Buffer to empty with reference pressure value
* @retval  Status [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_ReferencePressure(int32_t* RefP);

/**
* @brief  Check if the single measurement has completed.
* @param  the returned value is set to 1, when the measurement is completed
* @retval Status [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_IsMeasurementCompleted(uint8_t* Is_Measurement_Completed);

/**
* @brief  Get the values of the last single measurement.
* @param  Pressure and temperature value
* @retvalStatus [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Get_Measurement(LPS22HB_MeasureTypeDef_st *Measurement_Value);

/**
* @brief   Set Generic Configuration
* @param   Struct to empty with the chosen values
* @retval  Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_GenericConfig(LPS22HB_ConfigTypeDef_st* pxLPS22HBInit);

/**
* @brief  Get Generic configuration
* @param  Struct to empty with configuration values
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_GenericConfig(LPS22HB_ConfigTypeDef_st* pxLPS22HBInit);

/**
* @brief  Set Interrupt configuration
* @param  Struct holding the configuration values
* @retval  Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_InterruptConfig(LPS22HB_InterruptTypeDef_st* pLPS22HBInt);

/**
* @brief  LPS22HBGet_InterruptConfig
* @param  Struct to empty with configuration values
* @retval S Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_InterruptConfig(LPS22HB_InterruptTypeDef_st* pLPS22HBInt);

/**
* @brief  Set Fifo configuration
* @param  Struct holding the configuration values
* @retval  Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FifoConfig(LPS22HB_FIFOTypeDef_st* pLPS22HBFIFO);

/**
* @brief  Get Fifo configuration
* @param  Struct to empty with the configuration values
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoConfig(LPS22HB_FIFOTypeDef_st* pLPS22HBFIFO);

#ifdef  USE_FULL_ASSERT_LPS22HB
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
void LPS22HB_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  //printf("Wrong parameters tmp: file %s on line %d\r\n", file, line);

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @brief  Clock Tree Confoguration
* @param  LPS22HB_CTE_NotBalanced, LPS22HB_CTE_ABalanced
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_ClockTreeConfifuration(LPS22HB_CTE_et mode);


//****************************************************************//
//*******************DRIVER IMPLEMENTATION************************//
//****************************************************************//
/**
* @brief  Read identification code by WHO_AM_I register
* @param  Buffer to empty by Device identification Value.
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_DeviceID(uint8_t* deviceid)
{
  if(LPS22HB_ReadReg(LPS22HB_WHO_AM_I_REG, 1, deviceid))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}
/**
* @brief  Get the LPS22HB driver version.
* @param  None
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_DriverVersion(LPS22HB_DriverVersion_st *Version)
{
  Version->Major = LPS22HB_DriverVersion_Major;
  Version->Minor = LPS22HB_DriverVersion_Minor;
  Version->Point = LPS22HB_DriverVersion_Point;

  return LPS22HB_OK;
}
/**
* @brief  Set LPS22HB Low Power or Low Noise Mode Configuration
* @param  LPS22HB_LowNoise or LPS22HB_LowPower mode
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_PowerMode(LPS22HB_PowerMode_et mode)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_PowerMode(mode));

  if(LPS22HB_ReadReg(LPS22HB_RES_CONF_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_LCEN_MASK;
  tmp |= (uint8_t)mode;

  if(LPS22HB_WriteReg(LPS22HB_RES_CONF_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}
/**
* @brief  Get LPS22HB Power Mode
* @param   Buffer to empty with Mode: Low Noise or Low Current
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Get_PowerMode(LPS22HB_PowerMode_et* mode)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_RES_CONF_REG, 1, &tmp))
    return LPS22HB_ERROR;

  *mode = (LPS22HB_PowerMode_et)(tmp & LPS22HB_LCEN_MASK);

  return LPS22HB_OK;
}
/**
* @brief  Set LPS22HB Output Data Rate
* @param  Output Data Rate
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_Odr(LPS22HB_Odr_et odr)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_ODR(odr));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_ODR_MASK;
  tmp |= (uint8_t)odr;

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Get LPS22HB Output Data Rate
* @param  Buffer to empty with Output Data Rate
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Odr(LPS22HB_Odr_et* odr)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  *odr = (LPS22HB_Odr_et)(tmp & LPS22HB_ODR_MASK);

  return LPS22HB_OK;
}

/**
* @brief  Enable/Disale low-pass filter on LPS22HB pressure data
* @param  state: enable or disable
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_LowPassFilter(LPS22HB_State_et state)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(state));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_LPFP_MASK;
  tmp |= ((uint8_t)state)<<LPS22HB_LPFP_BIT;


  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;


  return LPS22HB_OK;
}


/**
* @brief  Set low-pass filter cutoff configuration on LPS22HB pressure data
* @param  Filter Cutoff ODR/9 or ODR/20
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_LowPassFilterCutoff(LPS22HB_LPF_Cutoff_et cutoff){

  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_LPF_Cutoff(cutoff));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_LPFP_CUTOFF_MASK;
  tmp |= (uint8_t)cutoff;



  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;


  return LPS22HB_OK;

}

/**
* @brief  Set Block Data Mode
* @detail It is recommended to set BDU bit to �1�.
* @detail This feature avoids reading LSB and MSB related to different samples.
* @param  LPS22HB_BDU_CONTINUOUS_UPDATE, LPS22HB_BDU_NO_UPDATE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/

static LPS22HB_Error_et LPS22HB_Set_Bdu(LPS22HB_Bdu_et bdu)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_BDUMode(bdu));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_BDU_MASK;
  tmp |= ((uint8_t)bdu);


  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG1, 1, &tmp))
  return LPS22HB_OK;

   return LPS22HB_OK;
}

/**
* @brief  Get Block Data Mode
* @param Buffer to empty whit the bdu mode read from sensor
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Bdu(LPS22HB_Bdu_et* bdu)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  *bdu = (LPS22HB_Bdu_et)(tmp & LPS22HB_BDU_MASK);

  return LPS22HB_OK;
}

/**
* @brief  Set SPI mode: 3 Wire Interface or 4 Wire Interface
* @param LPS22HB_SPI_3_WIRE, LPS22HB_SPI_4_WIRE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_SpiInterface(LPS22HB_SPIMode_et spimode)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_SPIMode(spimode));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_SIM_MASK;
  tmp |= (uint8_t)spimode;

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Clock Tree Configuration
* @param  LPS22HB_CTE_NotBalanced, LPS22HB_CTE_ABalanced
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_ClockTreeConfifuration(LPS22HB_CTE_et mode)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_CTE(mode));

  if(LPS22HB_ReadReg(LPS22HB_CLOCK_TREE_CONFIGURATION, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_CTE_MASK;
  tmp |= (uint8_t)mode;


  if(LPS22HB_WriteReg(LPS22HB_CLOCK_TREE_CONFIGURATION, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief  Get SPI mode: 3 Wire Interface or 4 Wire Interface
* @param Buffet to empty with spi mode read from Sensor
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_SpiInterface(LPS22HB_SPIMode_et* spimode)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  *spimode = (LPS22HB_SPIMode_et)(tmp & LPS22HB_SIM_MASK);

  return LPS22HB_OK;
}

  /**
* @brief   Software Reset. Self-clearing upon completion
* @param  None
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_SwReset(void)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= (0x01<<LPS22HB_SW_RESET_BIT);

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Reboot Memory Content
* @param None
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_MemoryBoot(void)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= (0x01<<LPS22HB_BOOT_BIT);

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Software Reset ann Reboot Memory Content.
* @detail  The device is reset to the power on configuration if the SWRESET bit is set to �1�
 + and BOOT is set to �1�; Self-clearing upon completion.
* @param     None
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_SwResetAndMemoryBoot(void)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= ((0x01<<LPS22HB_SW_RESET_BIT) | (0x01<<LPS22HB_BOOT_BIT));

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief   Enable/Disable FIFO Mode
* @param LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FifoModeUse(LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_EN_MASK;
  tmp |= ((uint8_t)status)<<LPS22HB_FIFO_EN_BIT;

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}
/**
* @brief   Enable/Disable FIFO Watermark Level Use
* @param   LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FifoWatermarkLevelUse(LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_WTM_EN_MASK;
  tmp |= ((uint8_t)status)<<LPS22HB_WTM_EN_BIT;

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

 /**
* @brief  Enable or Disable the Automatic increment register address during a multiple byte access with a serial interface (I2C or SPI)
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE. Default is LPS22HB_ENABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_AutomaticIncrementRegAddress(LPS22HB_State_et status)
{

  uint8_t tmp=0;

 LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG2, 1, &tmp))
  {
    return LPS22HB_ERROR;
  }

  tmp &= ~LPS22HB_ADD_INC_MASK;
  tmp |= (((uint8_t)status)<<LPS22HB_ADD_INC_BIT);

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG2, 1, &tmp))
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;

}

/**
* @brief  Enable/Disable I2C Interface
* @param State: LPS22HB_ENABLE (reset bit)/ LPS22HB_DISABLE (set bit)
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_I2C(LPS22HB_State_et statei2c)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(statei2c));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  /*Reset Bit->I2C Enabled*/
  tmp &= ~LPS22HB_I2C_MASK;
  tmp|=((uint8_t)~statei2c)<<LPS22HB_I2C_BIT;

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief   Set the one-shot bit in order to start acquisition when the ONE SHOT mode
*          has been selected by the ODR configuration.
* @detail  Once the measurement is done, ONE_SHOT bit will self-clear.
* @param   None
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_StartOneShotMeasurement(void)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  /* Set the one shot bit */
  /* Once the measurement is done, one shot bit will self-clear*/
  tmp |= LPS22HB_ONE_SHOT_MASK;

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;

}

/**
* @brief  Set Interrupt Active on High or Low Level
* @param  LPS22HB_ActiveHigh/LPS22HB_ActiveLow
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_InterruptActiveLevel(LPS22HB_InterruptActiveLevel_et mode)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_InterruptActiveLevel(mode));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_INT_H_L_MASK;
  tmp |= ((uint8_t)mode);

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief   Push-pull/open drain selection on interrupt pads. Default tmp: 0
* @param   LPS22HB_PushPull/LPS22HB_OpenDrain
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_InterruptOutputType(LPS22HB_OutputType_et output)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_OutputType(output));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_PP_OD_MASK;
  tmp |= (uint8_t)output;

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Set Data signal on INT pad control bits.
* @param  LPS22HB_DATA,LPS22HB_P_HIGH_LPS22HB_P_LOW,LPS22HB_P_LOW_HIGH
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_InterruptControlConfig(LPS22HB_OutputSignalConfig_et config)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_OutputSignal(config));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG3, 1, &tmp))
  {
      return LPS22HB_ERROR;
  }

    tmp &= ~(LPS22HB_INT_S12_MASK);
    tmp |= (uint8_t)config;

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG3, 1, &tmp))
  {
      return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}


/**
* @brief   Enable/Disable Data-ready signal on INT_DRDY pin.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_DRDYInterrupt(LPS22HB_State_et status)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_DRDY_MASK;
  tmp |= ((uint8_t)status)<<LPS22HB_DRDY_BIT;

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

 /**
* @brief   Enable/Disable FIFO overrun interrupt on INT_DRDY pin.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FIFO_OVR_Interrupt(LPS22HB_State_et status)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_OVR_MASK;
  tmp |= ((uint8_t)status)<<LPS22HB_FIFO_OVR_BIT;

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

 /**
* @brief   Enable/Disable FIFO threshold (Watermark) interrupt on INT_DRDY pin.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FIFO_FTH_Interrupt(LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_FTH_MASK;
  tmp |= ((uint8_t)status)<<LPS22HB_FIFO_FTH_BIT;

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Enable/Disable FIFO FULL interrupt on INT_DRDY pin.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FIFO_FULL_Interrupt(LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_FULL_MASK;
  tmp |= ((uint8_t)status)<<LPS22HB_FIFO_FULL_BIT;

  if(LPS22HB_WriteReg(LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}



/**
* @brief   Enable AutoRifP function
* @param   none
* @detail When this function is enabled, an internal register is set with the current pressure values
*         and the content is subtracted from the pressure output value and result is used for the interrupt generation.
*               the AutoRifP is slf creared.
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_AutoRifP(void)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= ((uint8_t)LPS22HB_AUTORIFP_MASK);

  if(LPS22HB_WriteReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Disable AutoRifP function
* @param   none
* @detail  the RESET_ARP bit is used to disable the AUTORIFP function. This bis i is selfdleared
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_ResetAutoRifP(void)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;


  tmp |= ((uint8_t)LPS22HB_RESET_ARP_MASK);

  if(LPS22HB_WriteReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/*
* @brief  Set AutoZero Function bit
* @detail When set to �1�, the actual pressure output is copied in the REF_P reg (@0x15..0x17)
* @param  None
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_AutoZeroFunction(void)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= LPS22HB_AUTOZERO_MASK;

  if(LPS22HB_WriteReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/*
* @brief  Set ResetAutoZero Function bit
* @details REF_P reg (@0x015..17) set pressure reference to default value RPDS reg (0x18/19).
* @param  None
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_ResetAutoZeroFunction(void)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  /* Set the RESET_AZ bit*/
  /* RESET_AZ is self cleared*/
  tmp |= LPS22HB_RESET_AZ_MASK;

  if(LPS22HB_WriteReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;


  return LPS22HB_OK;
}


/**
* @brief  Enable/ Disable the computing of differential pressure output (Interrupt Generation)
* @param  LPS22HB_ENABLE,LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_InterruptDifferentialGeneration(LPS22HB_State_et diff_en)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(diff_en));

  if(LPS22HB_ReadReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_DIFF_EN_MASK;
  tmp |= ((uint8_t)diff_en)<<LPS22HB_DIFF_EN_BIT;

  if(LPS22HB_WriteReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief  Get the DIFF_EN bit value
* @param  buffer to empty with the read value of DIFF_EN bit
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_InterruptDifferentialGeneration(LPS22HB_State_et* diff_en)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  *diff_en= (LPS22HB_State_et)((tmp & LPS22HB_DIFF_EN_MASK)>>LPS22HB_DIFF_EN_BIT);

  return LPS22HB_OK;
}

/**
* @brief  Latch Interrupt request to the INT_SOURCE register.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_LatchInterruptRequest(LPS22HB_State_et status)
{
  uint8_t tmp;

 LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_LIR_MASK;
  tmp |= (((uint8_t)status)<<LPS22HB_LIR_BIT);

  if(LPS22HB_WriteReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}



  /**
* @brief  Enable\Disable Interrupt Generation on differential pressure Low event
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_PLE(LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_PLE_MASK;
  tmp |= (((uint8_t)status)<<LPS22HB_PLE_BIT);

  if(LPS22HB_WriteReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Enable\Disable Interrupt Generation on differential pressure High event
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_PHE(LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_PHE_MASK;
  tmp |= (((uint8_t)status)<<LPS22HB_PHE_BIT);

  if(LPS22HB_WriteReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Get the Interrupt Generation on differential pressure status event and the Boot Status.
* @detail  The INT_SOURCE register is cleared by reading it.
* @param   Status Event Flag: BOOT, PH,PL,IA
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_InterruptDifferentialEventStatus(LPS22HB_InterruptDiffStatus_st* interruptsource)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_INTERRUPT_SOURCE_REG, 1, &tmp))
    return LPS22HB_ERROR;

  interruptsource->PH = (uint8_t)(tmp & LPS22HB_PH_MASK);
  interruptsource->PL = (uint8_t)((tmp & LPS22HB_PL_MASK)>>LPS22HB_PL_BIT);
  interruptsource->IA = (uint8_t)((tmp & LPS22HB_IA_MASK)>>LPS22HB_IA_BIT);
  interruptsource->BOOT= (uint8_t)((tmp & LPS22HB_BOOT_STATUS_MASK)>>LPS22HB_BOOT_STATUS_BIT);

  return LPS22HB_OK;
}

/**
* @brief  Get the status of Pressure and Temperature data
* @param  Data Status Flag:  TempDataAvailable, TempDataOverrun, PressDataAvailable, PressDataOverrun
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_DataStatus(LPS22HB_DataStatus_st* datastatus)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_STATUS_REG, 1, &tmp))
    return LPS22HB_ERROR;

  datastatus->PressDataAvailable = (uint8_t)(tmp & LPS22HB_PDA_MASK);
  datastatus->TempDataAvailable = (uint8_t)((tmp & LPS22HB_TDA_MASK)>>LPS22HB_PDA_BIT);
  datastatus->TempDataOverrun = (uint8_t)((tmp & LPS22HB_TOR_MASK)>>LPS22HB_TOR_BIT);
  datastatus->PressDataOverrun = (uint8_t)((tmp & LPS22HB_POR_MASK)>>LPS22HB_POR_BIT);

  return LPS22HB_OK;
}



/**
* @brief  Get the LPS22HB raw presure value
* @detail   The data are expressed as PRESS_OUT_H/_L/_XL in 2�s complement.
            Pout(hPA)=PRESS_OUT / 4096
* @param  The buffer to empty with the pressure raw value
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/

static LPS22HB_Error_et LPS22HB_Get_RawPressure(int32_t *raw_press)
{
  uint8_t buffer[3];
  uint32_t tmp = 0;

  if(LPS22HB_ReadReg(LPS22HB_PRESS_OUT_XL_REG, 3, buffer))
    return LPS22HB_ERROR;

  /* Build the raw data */
  for(uint8_t i=0; i<3; i++)
    tmp |= (((uint32_t)buffer[i]) << (8*i));

  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if(tmp & 0x00800000)
    tmp |= 0xFF000000;

  *raw_press = ((int32_t)tmp);

  return LPS22HB_OK;
}

/**
* @brief    Get the LPS22HB Pressure value in hPA.
* @detail   The data are expressed as PRESS_OUT_H/_L/_XL in 2�s complement.
            Pout(hPA)=PRESS_OUT / 4096
* @param      The buffer to empty with the pressure value that must be divided by 100 to get the value in hPA
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Get_Pressure(int32_t* Pout)
{
  int32_t raw_press;

  if(LPS22HB_Get_RawPressure(&raw_press))
    return LPS22HB_ERROR;

  *Pout = (raw_press*100)/4096;

  return LPS22HB_OK;
}

/**
* @brief    Get the Raw Temperature value.
* @detail   Temperature data are expressed as TEMP_OUT_H&TEMP_OUT_L as 2�s complement number.
*            Tout(degC)=TEMP_OUT/100
* @param     Buffer to empty with the temperature raw tmp.
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Get_RawTemperature(int16_t* raw_data)
{
  uint8_t buffer[2];
  uint16_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_TEMP_OUT_L_REG, 2, buffer))
    return LPS22HB_ERROR;

  /* Build the raw tmp */
  tmp = (((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0];

  *raw_data = ((int16_t)tmp);

  return LPS22HB_OK;
}


/**
* @brief    Get the Temperature value in 1/100 of C.
* @detail   Temperature data are expressed as TEMP_OUT_H&TEMP_OUT_L as 2�s complement number.
* @param Tout to fill with the temperature value in 1/100 of C
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Get_Temperature(int16_t* Tout)
{
  int16_t raw_data;

  if(LPS22HB_Get_RawTemperature(&raw_data))
    return LPS22HB_ERROR;

  *Tout = raw_data;

  return LPS22HB_OK;
}

/**
* @brief    Get the threshold value used for pressure interrupt generation (hPA).
* @detail   THS_P=THS_P_H&THS_P_L and is expressed as unsigned number. P_ths(hPA)=(THS_P)/16.
* @param    Buffer to empty with the pressure threshold in hPA
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Get_PressureThreshold(int16_t* P_ths)
{
  uint8_t tempReg[2];

  if(LPS22HB_ReadReg(LPS22HB_THS_P_LOW_REG, 2, tempReg))
    return LPS22HB_ERROR;

  *P_ths= (((((uint16_t)tempReg[1])<<8) + tempReg[0])/16);

  return LPS22HB_OK;
}

/**
* @brief    Set the threshold value  used for pressure interrupt generation (hPA).
* @detail   THS_P=THS_P_H&THS_P_L and is expressed as unsigned number. P_ths(hPA)=(THS_P)/16.
* @param       Pressure threshold in hPA
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_PressureThreshold(int16_t P_ths)
{
  uint8_t buffer[2];

  buffer[0] = (uint8_t)(16 * P_ths);
  buffer[1] = (uint8_t)(((uint16_t)(16 * P_ths))>>8);

  if(LPS22HB_WriteReg(LPS22HB_THS_P_LOW_REG, 2, buffer))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Set Fifo Mode.
* @param  Fifo Mode struct
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FifoMode(LPS22HB_FifoMode_et fifomode)
{
  uint8_t tmp;

 LPS22HB_assert_param(IS_LPS22HB_FifoMode(fifomode));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_MODE_MASK;
  tmp |= (uint8_t)fifomode;

  if(LPS22HB_WriteReg(LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief    Get Fifo Mode
* @param   buffer to empty with fifo mode tmp
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoMode(LPS22HB_FifoMode_et* fifomode)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= LPS22HB_FIFO_MODE_MASK;
  *fifomode = (LPS22HB_FifoMode_et)tmp;

  return LPS22HB_OK;
}

/**
* @brief    Set Fifo Watermark Level.
* @param    Watermark level value [0 31]
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FifoWatermarkLevel(uint8_t wtmlevel)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_WtmLevel(wtmlevel));

  if(LPS22HB_ReadReg(LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_WTM_POINT_MASK;
  tmp |= wtmlevel;

  if(LPS22HB_WriteReg(LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Get FIFO Watermark Level
* @param   buffer to empty with watermak level[0,31] value read from sensor
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoWatermarkLevel(uint8_t *wtmlevel)
{
  if(LPS22HB_ReadReg(LPS22HB_CTRL_FIFO_REG, 1, wtmlevel))
    return LPS22HB_ERROR;

  *wtmlevel &= LPS22HB_WTM_POINT_MASK;

  return LPS22HB_OK;
}

/**
* @brief    Get the Fifo Status
* @param    Status Flag: FIFO_FTH,FIFO_EMPTY,FIFO_FULL,FIFO_OVR and level of the FIFO->FIFO_LEVEL
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoStatus(LPS22HB_FifoStatus_st* status)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_STATUS_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  status->FIFO_FTH = (uint8_t)((tmp & LPS22HB_FTH_FIFO_MASK)>>LPS22HB_FTH_FIFO_BIT);
  status->FIFO_OVR=(uint8_t)((tmp & LPS22HB_OVR_FIFO_MASK)>>LPS22HB_OVR_FIFO_BIT);
  status->FIFO_LEVEL = (uint8_t)(tmp & LPS22HB_LEVEL_FIFO_MASK);

  if(status->FIFO_LEVEL ==LPS22HB_FIFO_EMPTY)
    status->FIFO_EMPTY=0x01;
  else
    status->FIFO_EMPTY=0x00;

  if (status->FIFO_LEVEL ==LPS22HB_FIFO_FULL)
     status->FIFO_FULL=0x01;
  else
    status->FIFO_FULL=0x00;


  return LPS22HB_OK;
}

/**
* @brief  Get the reference pressure after soldering for computing differential pressure (hPA)
* @param buffer to empty with the he pressure value (hPA)
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_PressureOffsetValue(int16_t *pressoffset)
{
  uint8_t buffer[2];
  int16_t raw_press;

  if(LPS22HB_ReadReg(LPS22HB_RPDS_L_REG, 2, buffer))
    return LPS22HB_ERROR;

  raw_press = (int16_t)((((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0]);

  *pressoffset = (raw_press*100)/4096;

  return LPS22HB_OK;
}


/**
* @brief  Get the Reference Pressure value
* @detail  It is a 24-bit data added to the sensor output measurement to detect a measured pressure beyond programmed limits.
* @param  Buffer to empty with reference pressure value
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_ReferencePressure(int32_t* RefP)
{
  uint8_t buffer[3];
  uint32_t tempVal=0;
  int32_t raw_press;

  if(LPS22HB_ReadReg(LPS22HB_REF_P_XL_REG, 3, buffer))
    return LPS22HB_ERROR;

  /* Build the raw data */
  for(uint8_t i=0; i<3; i++)
    tempVal |= (((uint32_t)buffer[i]) << (8*i));

  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if(tempVal & 0x00800000)
    tempVal |= 0xFF000000;

  raw_press =((int32_t)tempVal);
  *RefP = (raw_press*100)/4096;


  return LPS22HB_OK;
}


/**
* @brief  Check if the single measurement has completed.
* @param  the returned value is set to 1, when the measurement is completed
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_IsMeasurementCompleted(uint8_t* Is_Measurement_Completed)
{
  uint8_t tmp;
  LPS22HB_DataStatus_st datastatus;

  if(LPS22HB_ReadReg(LPS22HB_STATUS_REG, 1, &tmp))
    return LPS22HB_ERROR;

  datastatus.TempDataAvailable=(uint8_t)((tmp&LPS22HB_TDA_MASK)>>LPS22HB_TDA_BIT);
  datastatus.PressDataAvailable= (uint8_t)(tmp&LPS22HB_PDA_MASK);

  *Is_Measurement_Completed=(uint8_t)((datastatus.PressDataAvailable) & (datastatus.TempDataAvailable));

  return LPS22HB_OK;
}

/**
* @brief  Get the values of the last single measurement.
* @param  Pressure and temperature tmp
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Get_Measurement(LPS22HB_MeasureTypeDef_st *Measurement_Value)
{
  int16_t Tout;
  int32_t Pout;

  if(LPS22HB_Get_Temperature(&Tout))
    return LPS22HB_ERROR;

  Measurement_Value->Tout=Tout;

  if(LPS22HB_Get_Pressure(&Pout))
    return LPS22HB_ERROR;

   Measurement_Value->Pout=Pout;

  return LPS22HB_OK;

}

/**
* @brief  Initialization function for LPS22HB.
*         This function make a memory boot.
*         Init the sensor with a standard basic confifuration.
*         Low Power, ODR 25 Hz, Low Pass Filter disabled; BDU enabled; I2C enabled;
*        NO FIFO; NO Interrupt Enabled.
* @param  None.
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Init(void)
{
  LPS22HB_ConfigTypeDef_st pLPS22HBInit;
  LPS22HB_FIFOTypeDef_st pLPS22HBFIFO;

  /* Make LPS22HB Reset and Reboot */
  if(LPS22HB_SwResetAndMemoryBoot())
  {
    return LPS22HB_ERROR;
  }

// initialise in low power mode
 pLPS22HBInit.PowerMode=LPS22HB_LowPower;    //LPS22HB_LowNoise;
 pLPS22HBInit.OutputDataRate=LPS22HB_ODR_ONE_SHOT;
 pLPS22HBInit.LowPassFilter=LPS22HB_DISABLE;
 pLPS22HBInit.LPF_Cutoff=LPS22HB_ODR_20;
 pLPS22HBInit.BDU=LPS22HB_BDU_CONTINUOUS_UPDATE;
 pLPS22HBInit.IfAddInc=LPS22HB_ENABLE; //default
 pLPS22HBInit.Sim= LPS22HB_SPI_4_WIRE;

 /* Set Generic Configuration*/
 if(LPS22HB_Set_GenericConfig(&pLPS22HBInit))
 {
   return LPS22HB_ERROR;
 }
 pLPS22HBFIFO.FIFO_MODE=LPS22HB_FIFO_STREAM_MODE;
 pLPS22HBFIFO.WTM_INT=LPS22HB_ENABLE;
 pLPS22HBFIFO.WTM_LEVEL=0x08; // FiFo coefficient filter = 8
 if(LPS22HB_Set_FifoConfig(&pLPS22HBFIFO))
 {
    return LPS22HB_ERROR;
 }

  return LPS22HB_OK;
}

/**
* @brief  De initialization function for LPS22HB.
*         This function make a memory boot and clear the data output flags.
* @param  None.
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_DeInit(void)
{
  LPS22HB_MeasureTypeDef_st Measurement_Value;

  /* Make LPS22HB Reset and Reboot */
  if(LPS22HB_SwResetAndMemoryBoot())
    return LPS22HB_ERROR;

  /* Dump of data output */
  if(LPS22HB_Get_Measurement(&Measurement_Value))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief   Set Generic Configuration
* @param   Struct to empty with the chosen values
* @retval  Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_GenericConfig(LPS22HB_ConfigTypeDef_st* pxLPS22HBInit)
{

    /* Enable Low Current Mode (low Power) or Low Noise Mode*/
    if(LPS22HB_Set_PowerMode(pxLPS22HBInit->PowerMode))
    {
        return LPS22HB_ERROR;
    }

  /* Init the Output Data Rate*/
  if(LPS22HB_Set_Odr(pxLPS22HBInit->OutputDataRate))
  {
      return LPS22HB_ERROR;
  }

  /* BDU bit is used to inhibit the output registers update between the reading of upper and
  lower register parts. In default mode (BDU = �0�), the lower and upper register parts are
  updated continuously. If it is not sure to read faster than output data rate, it is recommended
  to set BDU bit to �1�. In this way, after the reading of the lower (upper) register part, the
  content of that output registers is not updated until the upper (lower) part is read too.
  This feature avoids reading LSB and MSB related to different samples.*/

  if(LPS22HB_Set_Bdu(pxLPS22HBInit->BDU))
  {
    return LPS22HB_ERROR;
  }

  /*Enable/Disale low-pass filter on LPS22HB pressure data*/
  if(LPS22HB_Set_LowPassFilter(pxLPS22HBInit->LowPassFilter))
  {
    return LPS22HB_ERROR;
  }

   /* Set low-pass filter cutoff configuration*/
  if(LPS22HB_Set_LowPassFilterCutoff(pxLPS22HBInit->LPF_Cutoff))
  {
    return LPS22HB_ERROR;
  }

  /* SIM bit selects the SPI serial interface mode.*/
  /* This feature has effect only if SPI interface is used*/

    if(LPS22HB_Set_SpiInterface(pxLPS22HBInit->Sim))
    {
        return LPS22HB_ERROR;
    }

  /*Enable or Disable the Automatic increment register address during a multiple byte access with a serial interface (I2C or SPI)*/
  if(LPS22HB_Set_AutomaticIncrementRegAddress(pxLPS22HBInit->IfAddInc))
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
* @brief  Get Generic configuration
* @param  Struct to empty with configuration values
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_GenericConfig(LPS22HB_ConfigTypeDef_st* pxLPS22HBInit)
{

  uint8_t tmp;

  /*Read LPS22HB_RES_CONF_REG*/
 if(LPS22HB_Get_PowerMode(&pxLPS22HBInit->PowerMode))
   return LPS22HB_ERROR;

  /*Read LPS22HB_CTRL_REG1*/
  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  pxLPS22HBInit->OutputDataRate= (LPS22HB_Odr_et)(tmp & LPS22HB_ODR_MASK);
  pxLPS22HBInit->BDU=(LPS22HB_Bdu_et)(tmp & LPS22HB_BDU_MASK);
  pxLPS22HBInit->Sim=(LPS22HB_SPIMode_et)(tmp& LPS22HB_SIM_MASK);
  pxLPS22HBInit->LowPassFilter=(LPS22HB_State_et)((tmp& LPS22HB_LPFP_MASK)>>LPS22HB_LPFP_BIT);
  pxLPS22HBInit->LPF_Cutoff=(LPS22HB_LPF_Cutoff_et)(tmp& LPS22HB_LPFP_CUTOFF_MASK);

  /*Read LPS22HB_CTRL_REG2*/
  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  pxLPS22HBInit->IfAddInc=(LPS22HB_State_et)((tmp& LPS22HB_ADD_INC_MASK)>>LPS22HB_ADD_INC_BIT);

  return LPS22HB_OK;
}


/**
* @brief  Set Interrupt configuration
* @param  Struct holding the configuration values
* @retval  Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_InterruptConfig(LPS22HB_InterruptTypeDef_st* pLPS22HBInt)
{
  /* Enable Low Current Mode (low Power) or Low Noise Mode*/
   if(LPS22HB_Set_InterruptActiveLevel(pLPS22HBInt->INT_H_L))
    return LPS22HB_ERROR;

   /* Push-pull/open drain selection on interrupt pads.*/
   if(LPS22HB_Set_InterruptOutputType(pLPS22HBInt->PP_OD))
    return LPS22HB_ERROR;

   /* Set Data signal on INT pad control bits.*/
   if(LPS22HB_Set_InterruptControlConfig(pLPS22HBInt->OutputSignal_INT))
    return LPS22HB_ERROR;

   /* Enable/Disable Data-ready signal on INT_DRDY pin. */
   if(LPS22HB_Set_DRDYInterrupt(pLPS22HBInt->DRDY))
    return LPS22HB_ERROR;

    /* Enable/Disable FIFO overrun interrupt on INT_DRDY pin. */
   if(LPS22HB_Set_FIFO_OVR_Interrupt(pLPS22HBInt->FIFO_OVR))
    return LPS22HB_ERROR;

   /* Enable/Disable FIFO Treshold interrupt on INT_DRDY pin. */
   if(LPS22HB_Set_FIFO_FTH_Interrupt(pLPS22HBInt->FIFO_FTH))
    return LPS22HB_ERROR;

   /* Enable/Disable FIFO FULL interrupt on INT_DRDY pin. */
   if(LPS22HB_Set_FIFO_FULL_Interrupt(pLPS22HBInt->FIFO_FULL))
    return LPS22HB_ERROR;

  /* Latch Interrupt request to the INT_SOURCE register. */
    if(LPS22HB_LatchInterruptRequest(pLPS22HBInt->LatchIRQ))
      return LPS22HB_ERROR;

    /* Set the threshold value  used for pressure interrupt generation (hPA). */
   if(LPS22HB_Set_PressureThreshold(pLPS22HBInt->THS_threshold))
      return LPS22HB_ERROR;

   /*Enable/Disable  AutoRifP function */
  if(pLPS22HBInt->AutoRifP==LPS22HB_ENABLE){
    if(LPS22HB_Set_AutoRifP())
      return LPS22HB_ERROR;
  }
  else{
    if(LPS22HB_ResetAutoRifP())
      return LPS22HB_ERROR;
  }

  /*Enable/Disable AutoZero function*/
  if(pLPS22HBInt->AutoZero==LPS22HB_ENABLE){
    if(LPS22HB_Set_AutoZeroFunction())
      return LPS22HB_ERROR;
  }
  else{
    if(LPS22HB_ResetAutoZeroFunction())
      return LPS22HB_ERROR;
  }


   if(pLPS22HBInt->OutputSignal_INT==LPS22HB_P_HIGH)
   {
    /* Enable\Disable Interrupt Generation on differential pressure high event*/
      if(LPS22HB_Set_PHE(LPS22HB_ENABLE))
        return LPS22HB_ERROR;
       if(LPS22HB_Set_InterruptDifferentialGeneration(LPS22HB_ENABLE))
          return LPS22HB_ERROR;
   }
   else  if(pLPS22HBInt->OutputSignal_INT==LPS22HB_P_LOW)
      {
    /* Enable Interrupt Generation on differential pressure Loe event*/
      if(LPS22HB_Set_PLE(LPS22HB_ENABLE))
        return LPS22HB_ERROR;
       if(LPS22HB_Set_InterruptDifferentialGeneration(LPS22HB_ENABLE))
          return LPS22HB_ERROR;
   }
    else  if(pLPS22HBInt->OutputSignal_INT==LPS22HB_P_LOW_HIGH)
    {
      /* Enable Interrupt Generation on differential pressure high event*/
      if(LPS22HB_Set_PHE(LPS22HB_ENABLE))
        return LPS22HB_ERROR;
    /* Enable\Disable Interrupt Generation on differential pressure Loe event*/
      if(LPS22HB_Set_PLE(LPS22HB_ENABLE))
        return LPS22HB_ERROR;
       if(LPS22HB_Set_InterruptDifferentialGeneration(LPS22HB_ENABLE))
          return LPS22HB_ERROR;
   }
   else
   {
      if(LPS22HB_Set_InterruptDifferentialGeneration(LPS22HB_DISABLE))
          return LPS22HB_ERROR;
      /* Disable Interrupt Generation on differential pressure High event*/
      if(LPS22HB_Set_PHE(LPS22HB_DISABLE))
        return LPS22HB_ERROR;
     /* Disable Interrupt Generation on differential pressure Low event*/
      if(LPS22HB_Set_PLE(LPS22HB_DISABLE))
        return LPS22HB_ERROR;
   }

  return LPS22HB_OK;
}

/**
* @brief  LPS22HBGet_InterruptConfig
* @param  Struct to empty with configuration values
* @retval S Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_InterruptConfig(LPS22HB_InterruptTypeDef_st* pLPS22HBInt)
{
   uint8_t tmp;

  /*Read LPS22HB_CTRL_REG3*/
  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  pLPS22HBInt->INT_H_L= (LPS22HB_InterruptActiveLevel_et)(tmp & LPS22HB_INT_H_L_MASK);
  pLPS22HBInt->PP_OD=(LPS22HB_OutputType_et)(tmp & LPS22HB_PP_OD_MASK);
  pLPS22HBInt->OutputSignal_INT=(LPS22HB_OutputSignalConfig_et)(tmp& LPS22HB_INT_S12_MASK);
  pLPS22HBInt->DRDY=(LPS22HB_State_et)((tmp& LPS22HB_DRDY_MASK)>>LPS22HB_DRDY_BIT);
  pLPS22HBInt->FIFO_OVR=(LPS22HB_State_et)((tmp& LPS22HB_FIFO_OVR_MASK)>>LPS22HB_FIFO_OVR_BIT);
  pLPS22HBInt->FIFO_FTH=(LPS22HB_State_et)((tmp& LPS22HB_FIFO_FTH_MASK)>>LPS22HB_FIFO_FTH_BIT);
  pLPS22HBInt->FIFO_FULL=(LPS22HB_State_et)((tmp& LPS22HB_FIFO_FULL_MASK)>>LPS22HB_FIFO_FULL_BIT);

  /*Read LPS22HB_INTERRUPT_CFG_REG*/
  if(LPS22HB_ReadReg(LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  pLPS22HBInt->LatchIRQ= (LPS22HB_State_et)((tmp& LPS22HB_LIR_MASK)>>LPS22HB_LIR_BIT);

  if(LPS22HB_Get_PressureThreshold(&pLPS22HBInt->THS_threshold))
    return LPS22HB_ERROR;

  //AutoRifP and Autozero are self clear //
  pLPS22HBInt->AutoRifP=LPS22HB_DISABLE;
  pLPS22HBInt->AutoZero=LPS22HB_DISABLE;

  return LPS22HB_OK;
}

/**
* @brief  Set Fifo configuration
* @param  Struct holding the configuration values
* @retval  Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
static LPS22HB_Error_et LPS22HB_Set_FifoConfig(LPS22HB_FIFOTypeDef_st* pLPS22HBFIFO)
{

   if(pLPS22HBFIFO->FIFO_MODE == LPS22HB_FIFO_BYPASS_MODE) {
    /* FIFO Disable-> FIFO_EN bit=0 in CTRL_REG2*/
    if(LPS22HB_Set_FifoModeUse(LPS22HB_DISABLE))
      return LPS22HB_ERROR;
    /* Force->Disable FIFO Watermark Level Use*/
     if(LPS22HB_Set_FifoWatermarkLevelUse(LPS22HB_DISABLE))
          return LPS22HB_ERROR;

    /* Force->Disable FIFO Treshold interrupt on INT_DRDY pin. */
     if(LPS22HB_Set_FIFO_FTH_Interrupt(LPS22HB_DISABLE))
            return LPS22HB_ERROR;
  }
  else {
    /* FIFO Enable-> FIFO_EN bit=1 in CTRL_REG2*/
    if(LPS22HB_Set_FifoModeUse(LPS22HB_ENABLE))
    {
      return LPS22HB_ERROR;
    }

      if (pLPS22HBFIFO->WTM_INT){
        /* Enable FIFO Watermark Level Use*/
        if(LPS22HB_Set_FifoWatermarkLevelUse(LPS22HB_ENABLE))
        {
          return LPS22HB_ERROR;
        }
        /*Set Fifo Watermark Level*/
        if(LPS22HB_Set_FifoWatermarkLevel(pLPS22HBFIFO->WTM_LEVEL))
        {
          return LPS22HB_ERROR;
        }
        /* Force->Enable FIFO Treshold interrupt on INT_DRDY pin. */
        if(LPS22HB_Set_FIFO_FTH_Interrupt(LPS22HB_ENABLE))
        {
            return LPS22HB_ERROR;
        }
      }
  }

  if(LPS22HB_Set_FifoMode(pLPS22HBFIFO->FIFO_MODE))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Get Fifo configuration
* @param  Struct to empty with the configuration values
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoConfig(LPS22HB_FIFOTypeDef_st* pLPS22HBFIFO)
{
   uint8_t tmp;

  if(LPS22HB_ReadReg(LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  /*!< Fifo Mode Selection */
  pLPS22HBFIFO->FIFO_MODE= (LPS22HB_FifoMode_et)(tmp& LPS22HB_FIFO_MODE_MASK);

  /*!< FIFO threshold/Watermark level selection*/
  pLPS22HBFIFO->WTM_LEVEL= (uint8_t)(tmp& LPS22HB_WTM_POINT_MASK);

  if(LPS22HB_ReadReg(LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

   /*!< Enable/Disable the watermark interrupt*/
  pLPS22HBFIFO->WTM_INT= (LPS22HB_State_et)((tmp& LPS22HB_WTM_EN_MASK)>>LPS22HB_WTM_EN_BIT);


  return LPS22HB_OK;
}

// Timeout for I2C accesses in 'ticks'
#define I2C_ACCESS_TIMEOUT (100)
static LPS22HB_Error_et LPS22HB_ReadReg(uint8_t regAddr, uint32_t numByteToRead, uint8_t* data)
{
    // For read, first write the register address, then read the data
    struct hal_i2c_master_data mdata = {
      .address = ALTIMETER_I2C_ADDR,
      .buffer = &regAddr,
      .len = 1,
    };
    // Write reg address
    int rc = hal_i2c_master_write(ALTIMETER_I2C_CHAN, &mdata, I2C_ACCESS_TIMEOUT, 1);
    if (rc==0) {
      mdata.buffer = data;    // read the data now
      mdata.len = numByteToRead;
      rc = hal_i2c_master_read(ALTIMETER_I2C_CHAN, &mdata, I2C_ACCESS_TIMEOUT, 1);
    }
    if (rc==0)
    {
      return LPS22HB_OK;
    } 
    else
    {
      log_warn("i2c read to altimeter fails:%d", rc);
      return LPS22HB_ERROR;
    }
}

static LPS22HB_Error_et LPS22HB_WriteReg(uint8_t regAddr, uint32_t numByteToWrite, uint8_t* data)
{
    //If more than 2 bytes are needed to be written, this function has to be updated because for now,
    //can only write 2 bytes at regAddr (developed like that for more simplicity)

    // For write, you write directly in struct the internal address (register) and the data
    int rc = -1;
    if (numByteToWrite >= 3)
    {
        //Max 2 bytes with this driver
        return LPS22HB_ERROR;
    }
    else if (numByteToWrite == 2)
    {
        uint8_t i2c_data[3] = {regAddr, data[0], data[1]};
        struct hal_i2c_master_data mdata = 
        {
            .address = ALTIMETER_I2C_ADDR,
            .buffer = i2c_data,
            .len = 3,
        };
        rc = hal_i2c_master_write(ALTIMETER_I2C_CHAN, &mdata, I2C_ACCESS_TIMEOUT, 1);
    }
    else
    {
        uint8_t i2c_data[2] = {regAddr, *data};
        struct hal_i2c_master_data mdata = 
        {
            .address = ALTIMETER_I2C_ADDR,
            .buffer = i2c_data,
            .len = 2,
        };
        rc = hal_i2c_master_write(ALTIMETER_I2C_CHAN, &mdata, I2C_ACCESS_TIMEOUT, 1);
    }
    if (rc==0) {
        return LPS22HB_OK;
    } 
    else 
    {
      log_warn("i2c write to altimeter fails:%d", rc);
      return LPS22HB_ERROR;
    }
}

//****************************************************************//
//**********************API IMPLEMENTATION************************//
//****************************************************************//

ALTI_Error_t ALTI_init(void)
{
  if(LPS22HB_Init() == LPS22HB_OK)
    {
        return ALTI_SUCCESS;
    }

    return ALTI_ERROR;
}

ALTI_Error_t ALTI_activate(void)
{
    //Set altimeter mode to Low Noise
    if (LPS22HB_Set_PowerMode(LPS22HB_LowNoise) != LPS22HB_OK)
    {
      return ALTI_ERROR;
    }
    //Set ODR to 75Hz
    if (LPS22HB_Set_Odr(LPS22HB_ODR_75HZ) != LPS22HB_OK)
    {
      return ALTI_ERROR;
    }
    return ALTI_SUCCESS;
}

ALTI_Error_t ALTI_sleep(void)
{
    //Set ODR to one shot mode to prevent wakeup from sleep
    if (LPS22HB_Set_Odr(LPS22HB_ODR_ONE_SHOT) != LPS22HB_OK)
    {
      return ALTI_ERROR;
    }
    //Set altimeter mode to Low Power mode
    if (LPS22HB_Set_PowerMode(LPS22HB_LowPower) != LPS22HB_OK)
    {
      return ALTI_ERROR;
    }
    return ALTI_SUCCESS;
}

ALTI_Error_t ALTI_calibratePressure(int16_t referencePressure)
{
    uint8_t buffer[2] = {0};
    
    buffer[0] = referencePressure;
    buffer[1] = (referencePressure >> 8);
    if(LPS22HB_WriteReg(LPS22HB_RPDS_L_REG, 2, buffer) != LPS22HB_OK)
    {
        return ALTI_ERROR;
    }
    return ALTI_SUCCESS;
}

ALTI_Error_t ALTI_readAllData(int32_t *pressure, int16_t *temperature)
{
    if (ALTI_readPressure(pressure) != ALTI_SUCCESS)
    {
      return ALTI_ERROR;
    }
    if (ALTI_readTemperature(temperature) != ALTI_SUCCESS) 
    {
        return ALTI_ERROR;
    }
    return ALTI_SUCCESS;
}

ALTI_Error_t ALTI_readPressure(int32_t *pressure)
{
    if (LPS22HB_Get_Pressure(pressure)!= LPS22HB_OK)
    {
      return ALTI_ERROR;
    }
    return ALTI_SUCCESS;
}

ALTI_Error_t ALTI_readTemperature(int16_t *temperature)
{
    if (LPS22HB_Get_Temperature(temperature) != LPS22HB_OK)
    {
      return ALTI_ERROR;
    }
    return ALTI_SUCCESS;
}
#endif /* ALTI_LPS22HB */