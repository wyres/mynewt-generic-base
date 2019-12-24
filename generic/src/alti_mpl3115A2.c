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
#ifdef ALTI_MPL3115A2

//****************************************************************//
//**********************API IMPLEMENTATION************************//
//****************************************************************//

ALTI_Error_t ALTI_init(void)
{
  return ALTI_SUCCESS;
}

ALTI_Error_t ALTI_activate(void)
{
    return ALTI_SUCCESS;
}

ALTI_Error_t ALTI_sleep(void)
{
    return ALTI_SUCCESS;
}

ALTI_Error_t ALTI_calibratePressure(int16_t referencePressure)
{
    return ALTI_SUCCESS;
}

ALTI_Error_t ALTI_readAllData(int32_t *pressure, int16_t *temperature)
{
  *pressure = 0;
  *temperature = 0;
  return ALTI_SUCCESS;
}

ALTI_Error_t ALTI_readPressure(int32_t *pressure)
{
  *pressure = 0;
    return ALTI_SUCCESS;
}

ALTI_Error_t ALTI_readTemperature(int16_t *temperature)
{
  *temperature = 0;
    return ALTI_SUCCESS;
}
#endif /* ALTI_MPL3115A2 */