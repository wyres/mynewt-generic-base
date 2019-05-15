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
