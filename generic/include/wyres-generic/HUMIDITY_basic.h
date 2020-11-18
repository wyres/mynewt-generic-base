/**
 * Copyright 2020 Joël Wiki
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at
 *    http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, 
 * software distributed under the License is distributed on 
 * an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
 * either express or implied. See the License for the specific 
 * language governing permissions and limitations under the License.
 * @author Joël Wiki
 * @date   06/09/2020
 * @brief  Basic API to configure and use an Humidity sensor
*/
#ifndef H_HUMIDITY_BASIC_H
#define H_HUMIDITY_BASIC_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

//****************************************************************//
//**************************API ENUM******************************//
//****************************************************************//
typedef enum
{
    HUMIDITY_SUCCESS,
     HUMIDITY_ERROR,
}  HUMIDITY_Error_t;


//****************************************************************//
//***********************API PROTOTYPES***************************//
//****************************************************************//
// Return true if a humidy sensor present
bool HUMIDITY_present();

/*!
 * @brief   Initialize HUMIDITY sensor
 * @param   void
 * @return  HUMIDITY_SUCCESS on success, HUMIDITY_ERROR otherwise
 */
HUMIDITY_Error_t HUMIDITY_init(void);
HUMIDITY_Error_t HUMIDITY_reset();
HUMIDITY_Error_t  HUMIDITY_activate();
HUMIDITY_Error_t HUMIDITY_sleep();

/*!
 * @brief       Read all data in one shot
 * @param[OUT]  Pointer to where pressure will be written
 * @param[OUT]  Pointer to where temperature will be written
 * @return      HUMIDITY_SUCCESS on success, HUMIDITY_ERROR otherwise
 */
HUMIDITY_Error_t HUMIDITY_readAllData(int8_t *humidity, int16_t *temperature);

/*!
 * @brief       Read pressure
 * @param[OUT]  Pointer to where pressure will be written
 * @return      HUMIDITY_SUCCESS on success, HUMIDITY_ERROR otherwise
 */
HUMIDITY_Error_t HUMIDITY_readHumidity(int8_t *humidity);
/*!
 * @brief       Read temperature
 * @param[OUT]  Pointer to where temperature will be written
 * @return      HUMIDITY_SUCCESS on success, HUMIDITY_ERROR otherwise
 */
HUMIDITY_Error_t HUMIDITY_readTemperature(int16_t *temperature);

HUMIDITY_Error_t HUMIDITY_calibrate();

#ifdef __cplusplus
}
#endif

#endif //H_HUMIDITY_BASIC_H
