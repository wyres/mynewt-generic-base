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
 * Sensor manager. 
 * Works on a start/stop basis, records the values which can be read at any time
 */
#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_i2c.h"

#include "wyres-generic/wutils.h"

#include "wyres-generic/sensorMgr.h"


// Timeout for I2C accesses in 'ticks'
#define I2C_ACCESS_TIMEOUT (100)

void SRMgr_start() {

}

void SRMgr_stop() {
    
}
uint32_t SRMgr_getTemp() {
    return 2000;        // value *100
}
uint32_t SRMgr_getPressure() {
    return 93000;       // in Pa
}
uint32_t SRMgr_getBattery() {
    return 3000;         // in mV
}
uint32_t SRMgr_getLight() {
    return 0;
}
uint32_t SRMgr_getButtonMask() {
    return 0;
}


