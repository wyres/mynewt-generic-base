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
 * HUMIDITY  API implementation for AHT10 (aka DHT10)
 */
#include <stdint.h>

#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_i2c.h"

#include "wyres-generic/wutils.h"
#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/HUMIDITY_basic.h"

/* Only include this implementation of the HUMIDITY  api if the bsp says we have one of these */
#ifdef HUM_AHT10
#define HUMIDITY_I2C_CHAN (0)
#define HUMIDITY_I2C_ADDR (0x38)

#define I2C_ACCESS_TIMEOUT (100)

//****************************************************************//
//**********************API IMPLEMENTATION************************//
//****************************************************************//
// Return true if a humidy sensor present
bool HUMIDITY_present() {
  // TODO probe i2c to check if hw present at address of ht10 sensor
  return true;
}

HUMIDITY_Error_t HUMIDITY_init(void)
{
  // Note 1 is OFF so start with it off
  GPIO_define_out("ext-i2c-pwr", EXT_I2C_PWR, 1, LP_DEEPSLEEP, PULL_UP);
  GPIO_write(EXT_I2C_PWR, 0);     // yup pull DOWN for ON
  if (HUMIDITY_reset()) {
    /* create struct who contains data to calibrate */
    return HUMIDITY_calibrate();
  }
  return HUMIDITY_ERROR;
}


HUMIDITY_Error_t HUMIDITY_readAllData(int8_t *humidity, int16_t *temperature)
{
  uint8_t readData[3]=  {0xAC,0x33,0x00}; ///< aquisition command : read from register 0xAC
  uint8_t data[6];
  struct hal_i2c_master_data mdata = {
        .address = HUMIDITY_I2C_ADDR,
        .buffer = &readData[0],
        .len = sizeof(readData),
    };

 int rc =  hal_i2c_master_write(HUMIDITY_I2C_CHAN, &mdata, I2C_ACCESS_TIMEOUT, 1); //just write these data

  if(rc==0){
    mdata.buffer = &data[0];
    mdata.len = sizeof(data);
    rc = hal_i2c_master_read(HUMIDITY_I2C_CHAN, &mdata, I2C_ACCESS_TIMEOUT, 1);
    if (rc==0) {
    // need to check for status busy ie 1 byte read with bit7=1
      if ((data[0] & 0x80) == 0) {
        // Extract values form byte array (20 bits for each value in BE order)
        uint32_t hval = (uint32_t)((((data[3] & 0xF0)>>4) + (data[2]<<8) + (data[1]<<16)) >> 4); 
        uint32_t tval = (uint32_t)(((data[3] & 0x0F) << 16) + (data[4]<<8) + data[5]);
        if (humidity!=NULL) {
          *humidity = (hval*100) / (1024*1024);    // should explicit the LE access
        }
        if (temperature!=NULL) {
          *temperature = ((tval*200) / (1024*1024))-50;
        }
        return HUMIDITY_SUCCESS;
      } else {
        // device is budy, sorry
        return HUMIDITY_ERROR;  
      }
    }
  }
  return HUMIDITY_ERROR;
}


HUMIDITY_Error_t HUMIDITY_readHumidity(int8_t *humidity)
{
  return HUMIDITY_readAllData(humidity, NULL);
}

HUMIDITY_Error_t HUMIDITY_readTemperature(int16_t *temperature)
{
  return HUMIDITY_readAllData(NULL, temperature);
}


HUMIDITY_Error_t HUMIDITY_reset()
{
  uint8_t reset[3] = {0xBA};

  // Init and calibrate by writing 0x08/0x00 to register 0xE1
  struct hal_i2c_master_data mdata = {
        .address = HUMIDITY_I2C_ADDR,
        .buffer = &reset[0],
        .len = sizeof(reset),
    };

 int rc =  hal_i2c_master_write(HUMIDITY_I2C_CHAN, &mdata, I2C_ACCESS_TIMEOUT, 1); //just write these data

  if(rc==0){
    return HUMIDITY_SUCCESS;
  }else{
    return HUMIDITY_ERROR;
  }
}
HUMIDITY_Error_t HUMIDITY_calibrate()
{
  uint8_t calibrate[3] = {0xE1,0x08,0x00};///< Calibration command

  // Init and calibrate by writing 0x08/0x00 to register 0xE1
  struct hal_i2c_master_data mdata = {
        .address = HUMIDITY_I2C_ADDR,
        .buffer = &calibrate[0],
        .len = sizeof(calibrate),
    };

 int rc =  hal_i2c_master_write(HUMIDITY_I2C_CHAN, &mdata, I2C_ACCESS_TIMEOUT, 1); //just write these data

  if(rc==0){
    return HUMIDITY_SUCCESS;
  }else{
    return HUMIDITY_ERROR;
  }
}

HUMIDITY_Error_t  HUMIDITY_activate() {
  GPIO_write(EXT_I2C_PWR, 0);     // yup pull DOWN for ON
    return HUMIDITY_SUCCESS;
}
HUMIDITY_Error_t HUMIDITY_sleep() {
  GPIO_write(EXT_I2C_PWR, 1);     // yup pull UP for OFF
  return HUMIDITY_SUCCESS;
}


#endif /* HUM_AHT10 */