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
#ifndef H_WCONSOLE_H
#define H_WCONSOLE_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { ATCMD_OK, ATCMD_GENERR, ATCMD_BADARG } ATRESULT;
typedef ATRESULT (*ATCMD_CBFN_t)(uint8_t nargs, char* argv[]);
typedef struct {
    const char* cmd;
    const char* desc;
    ATCMD_CBFN_t fn;
} ATCMD_DEF_t;

void wconsole_mgr_init(const char* dname, uint32_t baudrate, int8_t uartSelect);

// Start console with specific set of at cmds
void wconsole_start(uint8_t ncmds, ATCMD_DEF_t* cmds, uint32_t idleTimeoutS);
void wconsole_stop();
bool wconsole_isInit();
bool wconsole_isActive();
bool wconsole_println(const char* l, ...);

#ifdef __cplusplus
}
#endif

#endif  /* H_WCONSOLE_H */
