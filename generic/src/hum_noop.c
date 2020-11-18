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
*/
/**
 * HUMIDITY  API implementation case where no sensor present
 */
#include <stdint.h>

#include "os/os.h"
#include "bsp/bsp.h"

#include "wyres-generic/wutils.h"
#include "wyres-generic/HUMIDITY_basic.h"

/* Only include this implementation of the HUMIDITY  api no other one included */
#ifndef HUM_AHT10
//****************************************************************//
//**********************API IMPLEMENTATION************************//
//****************************************************************//
// Return true if a humidy sensor present
bool HUMIDITY_present() {
  return false;
}

HUMIDITY_Error_t HUMIDITY_init(void)
{
  return HUMIDITY_ERROR;
}

HUMIDITY_Error_t HUMIDITY_reset() 
{
  return HUMIDITY_ERROR;
}
HUMIDITY_Error_t HUMIDITY_readAllData(int8_t *humidity, int16_t *temperature)
{
  return HUMIDITY_ERROR;
}


HUMIDITY_Error_t HUMIDITY_readHumidity(int8_t *humidity)
{
  return HUMIDITY_ERROR;
}

HUMIDITY_Error_t HUMIDITY_readTemperature(int16_t *temperature)
{
  return HUMIDITY_ERROR;
}


HUMIDITY_Error_t HUMIDITY_calibrate()
{
  return HUMIDITY_ERROR;
}

HUMIDITY_Error_t  HUMIDITY_activate() {
    return HUMIDITY_ERROR;
}
HUMIDITY_Error_t HUMIDITY_sleep() {
    return HUMIDITY_ERROR;
}


#endif 