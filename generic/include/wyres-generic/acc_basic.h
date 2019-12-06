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
#ifndef H_ACC_BASIC_H
#define H_ACC_BASIC_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

// Detection modes of accelero
typedef enum 
{
    ACC_DetectionOff =      0,
    ACC_ShockDetection =    1,
    ACC_FreeFallDetection = 2,
} ACC_DetectionMode_t;

typedef enum
{
    ACC_SUCCESS,
    ACC_ERROR,
} ACC_Error_t;

// interface to accelero - rewrite to use sensor device driver?

/*!
 * @brief   Initialize accelerometer (depends on implementation, typically full scale = 2g & frequency rate = 10Hz)
 * @param   void
 * @return  true if success
 */
ACC_Error_t ACC_init();

/*!
 * @brief   Activate the accelerometer
 * @param   void
 * @return  true if success
 */
ACC_Error_t ACC_activate();

/*!
 * @brief   Put accelerometer in sleep mode
 * @param   void
 * @return  true if success
 */
ACC_Error_t ACC_sleep();

/*!
 * @brief     Poll data from accelerometer
 * @param[IN] Pointer to where data of x-axis will be written
 * @param[IN] Pointer to where data of y-axis will be written
 * @param[IN] Pointer to where data of z-axis will be written
 * @return    true if success
 */
ACC_Error_t ACC_readXYZ(int8_t* xp, int8_t* yp, int8_t* zp);

/*!
 * @brief       Check Pin state to know if board has moved
 * @param[OUT]  Pointer to where data will be written
 * @return      ACC_SUCCESS or ACC_ERROR
 */
ACC_Error_t ACC_HasDetectedMoved(bool *hasDetectedMove)

/*!
 * @brief       Check Pin state to know if board has fall
 * @param[OUT]  Pointer to where data will be written
 * @return      ACC_SUCCESS or ACC_ERROR
 */
ACC_Error_t ACC_HasDetectedFreeFallOrShock(bool *hasDetectedMove)

/*!
 * @brief     Set detection mode of accelerometer
 * @param[IN] Threshold used to detect a shock or free fall, units are 16mg (because full scale is 2g, depends on implementation)
 * @param[IN] Duration configuration of accelero detection
 * @return    true if success
 */
ACC_Error_t ACC_setDetectionMode(ACC_DetectionMode_t detectionMode, uint8_t threshold, uint8_t duration);

#ifdef __cplusplus
}
#endif

#endif  /* H_ACC_BASIC_H */
