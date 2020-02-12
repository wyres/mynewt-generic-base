# Wyres generic modules Package Definition

This is the package containing the generic firmware modules used in Wyres objects.

There are several 'manager' blocks that provide APIs for different functionality, as well as some basic utility code. These are briefly described below : see their header files for detailed API definitions.

wutils : utility methods, in particular logging and assert handling. The logging can be directed to UART or mynewt console output. It also provides a 'function log' circular buffer stored in PROM at reboot, handy for diagnostics. Log levels can be dynamically set, or removed entirly for product builds (based on BUILD_RELEASE syscfg flag, rather than NDEBUG, as assert() is kept for production builds)

timemgr : basic api to wrap time get/set and ability to set a 'now' to get absolute times.

configmgr : provides a key/length/opaque value api to store and retrieve config values from non-volatile storage. The implementation requires a byte level accessible storage such as a EEPROM. This must be implemented by the BSP.

rebootmgr : utility api for reboot management : stores reboot reasons/assert details etc in non-volatile storage (provided by configmgr) to allow diagnostic of object reboots.

gpiomgr : wrapper round hal level GPIO accesses which hooks the lowpowermgr api to provide automatic init/deinit of GPIO pins when the lowpower state changes.

uartselector/uartlinemgr/wsktmgr : async UART multi-access handling for 'line' based exchanges

gpsmgr/minema : handling of GPS module via UART connection, including NEMA decode and error handling.

movementmgr/acc_xxx : high level api for accelero functions and low level drivers for specific devices (currently for the ST lis2de12 device via I2C).

sensormgr/alti_xxx : high level api for sensors such as battery level, light, altimeter, etc and the low level drivers as required (currently for the ST LPS22HB via I2C)

ledmgr : non-blocking led control task to flash/light leds in various patterns as required. Integrates with the lowpower mgr api to disable LEDS in sleep modes.

lowpowermgr : api to allow drivers/app code to be notified when low power sleep modes are entered/exited (to enable/disable hardware) and to set the required level of sleep whenever the scheduler is idle (to signal which hw may be on or off). This hooks a system level implementation of the os_tick_idle() method used by MyNewt to indicate scheduling idleness.

wconsole : basic console handling which allows AT command set type interactions and deals with line parsing etc. The actual AT command sets are defined by the applicatin code as 'command/fn callback' pairs.

sm_exec : FSM (state machine) framework allowing the definition of multiple table based state machines, driven by events and serially executed by a single task. Note that use of this framework REQUIRES a NON-BLOCKING, ASYNCHRONOUS and EVENT DRIVEN architecture....

cirbuf : circular byte buffer utility implementation : thanks to Siddharth Chandrasekaran from Embed journal!

cborxxx : CBOR encoding methods : thanks to Intel Corp.

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

