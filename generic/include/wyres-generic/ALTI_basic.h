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
 * @author Nicolas Hell
 * @date   05/12/2019
 * @brief  Basic API to configure and use an altimeter
*/
#ifndef H_ALTI_BASIC_H
#define H_ALTI_BASIC_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

//****************************************************************//
//**************************API ENUM******************************//
//****************************************************************//
typedef enum
{
    ALTI_SUCCESS,
    ALTI_ERROR,
} ALTI_Error_t;


//****************************************************************//
//***********************API PROTOTYPES***************************//
//****************************************************************//

/*!
 * @brief   Initialize altimeter
 * @param   void
 * @return  ALTI_SUCCESS on success, ALTI_ERROR otherwise
 */
ALTI_Error_t ALTI_init(void);

/*!
 * @brief   Set the altimeter in active mode
 * @param   void
 * @return  ALTI_SUCCESS on success, ALTI_ERROR otherwise
 */
ALTI_Error_t ALTI_activate(void);
/*!
 * @brief  Set altimeter in low power mode
 * @param  void
 * @return ALTI_SUCCESS on success, ALTI_ERROR otherwise
 */
ALTI_Error_t ALTI_sleep(void);

/*!
 * @brief      Calibrate altimeter pressure
 * @param[IN]  The reference pressure
 * @return     ALTI_SUCCESS on success, ALTI_ERROR otherwise
 */
ALTI_Error_t ALTI_calibratePressure(int16_t referencePressure);

/*!
 * @brief       Read all data in one shot
 * @param[OUT]  Pointer to where pressure will be written
 * @param[OUT]  Pointer to where temperature will be written
 * @return      ALTI_SUCCESS on success, ALTI_ERROR otherwise
 */
ALTI_Error_t ALTI_readAllData(int32_t *pressure, int32_t deltaPressure, int16_t *temperature);

/*!
 * @brief       Read pressure
 * @param[OUT]  Pointer to where pressure will be written
 * @return      ALTI_SUCCESS on success, ALTI_ERROR otherwise
 */
ALTI_Error_t ALTI_readPressure(int32_t *pressure, int32_t deltaPressure);
/*!
 * @brief       Read temperature
 * @param[OUT]  Pointer to where temperature will be written
 * @return      ALTI_SUCCESS on success, ALTI_ERROR otherwise
 */
ALTI_Error_t ALTI_readTemperature(int16_t *temperature);


#ifdef __cplusplus
}
#endif

#endif //H_ALTI_BASIC_H
