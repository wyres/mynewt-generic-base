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
 * basic accelero interface implementation for MPL3115A2
 */
#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_i2c.h"

#include "wyres-generic/wutils.h"
#include "wyres-generic/acc_basic.h"

/* Only include this implementation of the accelero api if the bsp says we have one of these */
#ifdef ACC_MMA7660FCR1

// interface to accelero - rewrite to use sensor device driver?
ACC_Error_t ACC_init() {
    return ACC_SUCCESS;
}

ACC_Error_t ACC_activate() {
    return ACC_SUCCESS;
}

ACC_Error_t ACC_sleep() {
    return ACC_SUCCESS;
}
ACC_Error_t ACC_readXYZ(int8_t* xp, int8_t* yp, int8_t* zp) {
    ACC_Error_t res = ACC_SUCCESS;
    *xp = 0;
    *yp = 0;
    *zp = 0;
    return res;
}
/*!
 * @brief       Check Pin state to know if board has moved
 * @param[OUT]  Pointer to where data will be written
 * @return      ACC_SUCCESS or ACC_ERROR
 */
ACC_Error_t ACC_HasDetectedMoved(bool *hasDetectedMove)
{
    *hasDetectedMove = true;
    return ACC_SUCCESS;
}
/*!
 * @brief       Check Pin state to know if board has fall
 * @param[OUT]  Pointer to where data will be written
 * @return      ACC_SUCCESS or ACC_ERROR
 */
ACC_Error_t ACC_HasDetectedFreeFallOrShock(bool *hasDetectedMove)
{
    *hasDetectedMove = false;
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
    return ACC_SUCCESS;    
}

#endif /* ACC_MMA7660FCR1 */
