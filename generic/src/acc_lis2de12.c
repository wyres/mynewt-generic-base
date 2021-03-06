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
 * basic accelero interface implementation for LIS2DE12
 */
#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_i2c.h"

#include "wyres-generic/wutils.h"
#include "wyres-generic/acc_basic.h"

/* Only include this implementation of the accelero api if the bsp says we have one of these */
#ifdef ACC_LIS2DE12

// Timeout for I2C accesses in 'ticks'
#define I2C_ACCESS_TIMEOUT (100)
// ACT_DUR masks
#define LIS2DE_DEFAULT_ADDRESS 	0x18
// I2C address
#define LIS2DE_I2C_ADDRESS      0x4C
// LID2DE12 Registers
#define LIS2DE_ID             	0x33            //0x0D
// Registers
#define LIS2DE_STATUS_REG_AUX 	0x07
#define LIS2DE_OUT_TEMP_L 		0x0C
#define LIS2DE_OUT_TEMP_H 		0x0D
#define LIS2DE_WHO_AM_I 		0x0F
#define LIS2DE_TEMP_CFG_REG 	0x1F
#define LIS2DE_CTRL_REG1 		0x20
#define LIS2DE_CTRL_REG2 		0x21
#define LIS2DE_CTRL_REG3 		0x22
#define LIS2DE_CTRL_REG4 		0x23
#define LIS2DE_CTRL_REG5 		0x24
#define LIS2DE_CTRL_REG6 		0x25
#define LIS2DE_REFERENCE 		0x26
#define LIS2DE_STATUS_REG2 		0x27
#define LIS2DE_OUT_X 			0x29
#define LIS2DE_OUT_Y 			0x2B
#define LIS2DE_OUT_Z 			0x2D
#define LIS2DE_FIFO_CTRL_REG 	0x2E
#define LIS2DE_FIFO_SRC_REG 	0x2F
#define LIS2DE_INT1_CFG 		0x30
#define LIS2DE_INT1_SOURCE 		0x31
#define LIS2DE_INT1_THS 		0x32
#define LIS2DE_INT1_DURATION 	0x33
#define LIS2DE_INT2_CFG 		0x34
#define LIS2DE_INT2_SOURCE 		0x35
#define LIS2DE_INT2_THS 		0x36
#define LIS2DE_INT2_DURATION 	0x37
#define LIS2DE_CLICK_CFG 		0x38
#define LIS2DE_CLICK_SRC 		0x39
#define LIS2DE_CLICK_THS 		0x3A
#define LIS2DE_TIME_LIMIT 		0x3B
#define LIS2DE_TIME_LATENCY 	0x3C
#define LIS2DE_TIME_WINDOW 		0x3D
#define LIS2DE_ACT_THS 			0x3E
#define LIS2DE_ACT_DUR 			0x3F

// Register Masks
// STATUS_AUX_REG masks
#define LIS2DE_TOR_MASK 		0x40
#define LIS2DE_TDA_MASK 		0x04
// WHO_AM_I masks
#define LIS2DE_I_AM_MASK 		0x33
// TEMP_CFG_REG masks
#define LIS2DE_TEMP_EN_MASK 	0xC0
// CTRL_REG1 masks
#define LIS2DE_LPEN_MASK            0x08
#define LIS2DE_ODR_MASK 			0xF0
#define LIS2DE_Z_EN_MASK 			0x04
#define LIS2DE_Y_EN_MASK 			0x02
#define LIS2DE_X_EN_MASK			0x01
#define LIS2DE_XYZ_EN_MASK			0x07
#define LIS2DE_POWER_DOWN_MASK		0x00
#define LIS2DE_ONE_HZ_MASK			0x10
#define LIS2DE_TEN_HZ_MASK			0x20
#define LIS2DE_TWENTY_FIVE_HZ_MASK	0x30
#define LIS2DE_FIFTY_HZ_MASK		0x40
#define LIS2DE_ONE_HUNDRED_HZ_MASK	0x50
#define LIS2DE_TWO_HUNDRED_HZ_MASK	0x60
#define LIS2DE_FOUR_HUNDRED_HZ_MASK	0x70
// CTRL_REG2 masks
#define LIS2DE_HPM_MASK 		0xC0
#define LIS2DE_HPCF_MASK 		0x30
#define LIS2DE_FDS_MASK 		0x08
#define LIS2DE_HPCLICK_MASK 	0x04
#define LIS2DE_HPIS2_MASK 		0x02
#define LIS2DE_HPIS1_MASK 		0x01
// CTRL_REG3 masks
#define LIS2DE_I1_CLICK 		0x80
#define LIS2DE_I1_AOI1 			0x40
#define LIS2DE_I1_AOI2 			0x20
#define LIS2DE_I1_DRDY 			0x18
#define LIS2DE_I1_WTM 			0x04
#define LIS2DE_I1_OVERRUN 		0x02
// CTRL_REG4 masks
#define LIS2DE_BDU_MASK 		0x80
#define LIS2DE_FS_MASK 			0x30
#define LIS2DE_HR_MASK 			0x08
#define LIS2DE_ST_MASK 			0x06
#define LIS2DE_SIM_MASK 		0x01
// CTRL_REG5 masks
#define LIS2DE_BOOT_MASK 		0x80
#define LIS2DE_FIFO_EN_MASK 	0x40
#define LIS2DE_LIR_INT1_MASK 	0x08
#define LIS2DE_D4D_INT1_MASK 	0x04
#define LIS2DE_LIR_INT2_MASK	0x02
#define LIS2DE_D4D_INT2_MASK	0x01
// CTRL_REG6 masks
#define LIS2DE_I2C_CCK_EN_MASK	0x80
#define LIS2DE_I2C_INT1_MASK 	0x40
#define LIS2DE_I2C_INT2_MASK 	0x20
#define LIS2DE_BOOT_I2_MASK 	0x10
#define LIS2DE_P2_ACT_MASK 		0x08
#define LIS2DE_H_LACTIVE_MASK 	0x02
// STATUS_REG masks
#define LIS2DE_ZYXOR_MASK 		0x80
#define LIS2DE_ZOR_MASK 		0x40
#define LIS2DE_YOR_MASK 		0x20
#define LIS2DE_XOR_MASK 		0x10
#define LIS2DE_ZYXDA_MASK 		0x08
#define LIS2DE_ZDA_MASK 		0x04
#define LIS2DE_YDA_MASK 		0x02
#define LIS2DE_XDA_MASK 		0x01
// FIFO_CTRL_REG masks
#define LIS2DE_FM_MASK 			0xC0
#define LIS2DE_TR_MASK 			0x20
#define LIS2DE_FTH_MASK 		0x1F
// FIFO_SRC_REG masks
#define LIS2DE_WTM_MASK 		0x80
#define LIS2DE_OVRN_FIFO_MASK 	0x40
#define LIS2DE_EMPTY_MASK 		0x20
#define LIS2DE_FSS_MASK 		0x1F
// INT1/2_CFG masks
#define LIS2DE_AOI_MASK 		0x80
#define LIS2DE_6D_MASK 			0x40
#define LIS2DE_ZHIE_MASK 		0x20
#define LIS2DE_ZLIE_MASK 		0x10
#define LIS2DE_YHIE_MASK 		0x08
#define LIS2DE_YLIE_MASK 		0x04
#define LIS2DE_XHIE_MASK 		0x02
#define LIS2DE_XLIE_MASK 		0x01
// INT1/2_SRC masks
#define LIS2DE_INT_IA_MASK 		0x40
#define LIS2DE_ZH_MASK 			0x20
#define LIS2DE_ZL_MASK 			0x10
#define LIS2DE_YH_MASK 			0x08
#define LIS2DE_YL_MASK 			0x04
#define LIS2DE_XH_MASK 			0x02
#define LIS2DE_XL_MASK 			0x01
// INT1/2_THS masks
#define LIS2DE_THS_MASK 		0x4F
// INT1/2_DURATION masks
#define LIS2DE_D_MASK 			0x4F
// CLICK_CFG masks
#define LIS2DE_ZD_MASK 			0x20
#define LIS2DE_ZS_MASK 			0x10
#define LIS2DE_YD_MASK 			0x08
#define LIS2DE_YS_MASK 			0x04
#define LIS2DE_XD_MASK 			0x02
#define LIS2DE_XS_MASK 			0x01
// CLICK_SRC masks
#define LIS2DE_CLK_IA_MASK 		0x40
#define LIS2DE_DCLICK_MASK 		0x20
#define LIS2DE_SCLICK_MASK 		0x10
#define LIS2DE_SIGN_MASK 		0x08
#define LIS2DE_Z_CLICK_MASK 	0x04
#define LIS2DE_Y_CLICK_MASK 	0x02
#define LIS2DE_X_CLICK_MASK 	0x01
// CLICK_THS masks
#define LIS2DE_CLK_THS_MASK 	0x7F
// TIME_LIMIT masks
#define LIS2DE_TLI_MASK 		0x7F
// ACT_THS masks
#define LIS2DE_ACTH_MASK 		0x7F

typedef enum
{
    LIS2DE12_OK,
    LIS2DE12_ERR,
}
LIS2DE12_Error_t;

// Private internals: should be using device driver sensor but instead directly access a lis2de12 
/*!
 * @brief     Accelerometer rewrited register
 * @param[IN] Register address
 * @param[IN] Data to be written to internal register
 * @return    LIS2DE12_OK, LIS2DE12_ERR
 */
static LIS2DE12_Error_t LIS2DE12_WriteReg( uint8_t addr, uint8_t data )
{
    // For write, first the register address and then the actual value
    uint8_t i2c_data[2] = { addr, data};
    struct hal_i2c_master_data mdata = {
        .address = ACCELERO_I2C_ADDR,
        .buffer = i2c_data,
        .len = 2,
    };
    int rc = hal_i2c_master_write(ACCELERO_I2C_CHAN, &mdata, I2C_ACCESS_TIMEOUT, 1);
    if (rc==0) 
    {
        return LIS2DE12_OK;
    } 
    else
    {
        log_warn("i2c write to accelero fails:%d", rc);
        return LIS2DE12_ERR;
    }
}
/*!
 * @brief      Accelerometer read register
 * @param[IN]  Register address
 * @param[OUT] Data to be written
 * @return     LIS2DE12_OK, LIS2DE12_ERR
 */
static LIS2DE12_Error_t LIS2DE12_ReadReg( uint8_t addr, uint8_t *data )
{
    // For read, first write the register address, then read the data
    struct hal_i2c_master_data mdata = {
        .address = ACCELERO_I2C_ADDR,
        .buffer = &addr,
        .len = 1,
    };
    // Write reg address
    int rc = hal_i2c_master_write(ACCELERO_I2C_CHAN, &mdata, I2C_ACCESS_TIMEOUT, 1);
    if (rc==0) 
    {
        mdata.buffer = data;    // read the data now
        rc = hal_i2c_master_read(ACCELERO_I2C_CHAN, &mdata, I2C_ACCESS_TIMEOUT, 1);
    }
    if (rc==0) 
    {
        return LIS2DE12_OK;
    } 
    else 
    {
        log_warn("i2c read to accelero fails:%d", rc);
        return LIS2DE12_ERR;
    }

}

// interface to accelero - rewrite to use sensor device driver?
ACC_Error_t ACC_init() {
    uint8_t data = 0;
    uint8_t Rx = 0;

    // Read its id to check its the right device
    if (LIS2DE12_ReadReg(LIS2DE_WHO_AM_I, &Rx) != LIS2DE12_OK) 
    {
        log_debug("Fail, rx is %d", Rx);
        return ACC_ERROR;
    }
    if (Rx != LIS2DE_ID) 
    {
        log_error("accelero has bad id %d", Rx);
        return ACC_ERROR;
    }

    data = (LIS2DE_XYZ_EN_MASK | LIS2DE_LPEN_MASK | LIS2DE_TEN_HZ_MASK);
    if (LIS2DE12_WriteReg(LIS2DE_CTRL_REG1, data) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }

    data = (LIS2DE_HPIS1_MASK | LIS2DE_HPIS2_MASK);
    if(LIS2DE12_WriteReg(LIS2DE_CTRL_REG2, data) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }

    data = LIS2DE_I1_AOI1;
    if(LIS2DE12_WriteReg(LIS2DE_CTRL_REG3, data) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }

    data = 0x00;        // continuous update, full scale +/-2g, self test disabled, SPI 4 wire
    if(LIS2DE12_WriteReg(LIS2DE_CTRL_REG4, data) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }

    data = (LIS2DE_LIR_INT1_MASK | LIS2DE_LIR_INT2_MASK);
    if(LIS2DE12_WriteReg(LIS2DE_CTRL_REG5, data) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }

    data = (LIS2DE_I2C_INT2_MASK | LIS2DE_P2_ACT_MASK);
    if(LIS2DE12_WriteReg(LIS2DE_CTRL_REG6, data) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }

    // MOTION DETECTION SETUP
    data = 0x02; // 0x02
    if(LIS2DE12_WriteReg(LIS2DE_INT1_THS, data) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }

    data = 0x02; // 0x03
    if(LIS2DE12_WriteReg(LIS2DE_INT1_DURATION, data) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }

    data = (LIS2DE_ZHIE_MASK | LIS2DE_YHIE_MASK | LIS2DE_XHIE_MASK);
    if(LIS2DE12_WriteReg(LIS2DE_INT1_CFG, data) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }

    // Clear IT sources by reading them
    if (LIS2DE12_ReadReg(LIS2DE_INT1_SOURCE, &Rx) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }
    if (LIS2DE12_ReadReg(LIS2DE_INT2_SOURCE, &Rx) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }
    return ACC_SUCCESS;
}

ACC_Error_t ACC_activate() {
    //XYZ enabled, 10Hz frequency, low power mode enabled
    if (LIS2DE12_WriteReg(LIS2DE_CTRL_REG1, (LIS2DE_XYZ_EN_MASK | LIS2DE_TEN_HZ_MASK)) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }
    return ACC_SUCCESS;
}

ACC_Error_t ACC_sleep() {
    //Leave XYZ enabled, 10Hz frequency
    if (LIS2DE12_WriteReg(LIS2DE_CTRL_REG1, (LIS2DE_XYZ_EN_MASK | LIS2DE_ONE_HZ_MASK)) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }
    return ACC_SUCCESS;
}
ACC_Error_t ACC_readXYZ(int8_t* xp, int8_t* yp, int8_t* zp) {
    ACC_Error_t res = ACC_SUCCESS;
    // X axis
    if (LIS2DE12_ReadReg(LIS2DE_OUT_X, (uint8_t*)xp) != LIS2DE12_OK)
    {
        res = ACC_ERROR;
    }
    // Y axis
    if (LIS2DE12_ReadReg(LIS2DE_OUT_Y, (uint8_t*)yp) != LIS2DE12_OK)
    {
        res = ACC_ERROR;
    }
    // Z axis
    if (LIS2DE12_ReadReg(LIS2DE_OUT_Z, (uint8_t*)zp) != LIS2DE12_OK)
    {
        res = ACC_ERROR;
    }

    return res;
}
/*!
 * @brief       Check Pin state to know if board has moved
 * @param[OUT]  Pointer to where data will be written
 * @return      ACC_SUCCESS or ACC_ERROR
 */
ACC_Error_t ACC_HasDetectedMoved(bool *hasDetectedMove)
{
    uint8_t Rx = 0;
    *hasDetectedMove = false;
    
    if (LIS2DE12_ReadReg(LIS2DE_INT1_SOURCE, &Rx) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }
    // read of reg clears it
    //If IA register of INT1_SRC is SET, means that one or more detection events have occured since last check
    if ((Rx & LIS2DE_INT_IA_MASK) == LIS2DE_INT_IA_MASK)
    {
        *hasDetectedMove = true;
    }
    return ACC_SUCCESS;
}
/*!
 * @brief       Check Pin state to know if board has fall
 * @param[OUT]  Pointer to where data will be written
 * @return      ACC_SUCCESS or ACC_ERROR
 */
ACC_Error_t ACC_HasDetectedFreeFallOrShock(bool *hasDetectedMove)
{
    uint8_t Rx = 0;
    *hasDetectedMove = false;

    if (LIS2DE12_ReadReg(LIS2DE_INT2_SOURCE, &Rx) != LIS2DE12_OK)
    {
        return ACC_ERROR;
    }
    // read of reg clears it
    //If IA register of INT2_SRC is SET, means that one or more detection events have occured since last check
    if((Rx & LIS2DE_INT_IA_MASK) == LIS2DE_INT_IA_MASK)
    {
        *hasDetectedMove = true;
    }
    return ACC_SUCCESS;
}

/*!
 * @brief     Check Pin state to know if board has fall
 * @param[IN] Threshold used to detect a shock or free fall (in terms of acceleration)
 * @param[IN] Duration configuration of accelero detection
 * @return    ACC_SUCCESS or ACC_ERROR
 */
ACC_Error_t ACC_setDetectionMode(ACC_DetectionMode_t detectionMode, uint8_t threshold, uint8_t duration)
{
    switch(detectionMode)
    {
        case ACC_ShockDetection:
        {
            if(LIS2DE12_WriteReg(LIS2DE_INT2_CFG, (LIS2DE_ZHIE_MASK | LIS2DE_YHIE_MASK | LIS2DE_XHIE_MASK)) != LIS2DE12_OK)
            {
                return ACC_ERROR;
            }
            break;
        }
        case ACC_FreeFallDetection:
        {
            if(LIS2DE12_WriteReg(LIS2DE_INT2_CFG, (LIS2DE_AOI_MASK | LIS2DE_ZLIE_MASK | LIS2DE_YLIE_MASK | LIS2DE_XLIE_MASK)) != LIS2DE12_OK)
            {
                return ACC_ERROR;
            }
            break;
        }
        case ACC_DetectionOff:
        default:
        {
            if(LIS2DE12_WriteReg(LIS2DE_INT2_CFG, 0) != LIS2DE12_OK)
            {
                return ACC_ERROR;
            }
            break;
        }
    }

    if (detectionMode != ACC_FreeFallDetection && detectionMode != ACC_ShockDetection)
    {
        //Basic threshold can be 100
        //100 x 16mg = 1.5g
        if(LIS2DE12_WriteReg(LIS2DE_INT2_THS, threshold) != LIS2DE12_OK)
        {
            return ACC_ERROR;
        }

        //Duration basic value should be 6
        if(LIS2DE12_WriteReg(LIS2DE_INT2_DURATION, duration) != LIS2DE12_OK)
        {
            return ACC_ERROR;
        }
    }

    return ACC_SUCCESS;    
}

#endif /* ACC_LIS2DE12 */
