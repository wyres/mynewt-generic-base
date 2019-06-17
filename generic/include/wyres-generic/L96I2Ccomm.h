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
#ifndef H_L96I2CCOMM_H
#define H_L96I2CCOMM_H

#include <inttypes.h>
#include <mcu/mcu.h>

#include "os/os_eventq.h"
#include "wskt_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// Create a L96 via I2C access device
bool L96_I2C_comm_create(char* dname, const char* i2cdev, uint8_t i2caddr, int i2cpowerGPIO);

#ifdef __cplusplus
}
#endif

#endif  /* H_L96I2CCOMM_H */
