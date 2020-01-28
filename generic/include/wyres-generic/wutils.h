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
#ifndef H_WUTILS_H
#define H_WUTILS_H

#ifdef __cplusplus
extern "C" {
#endif
#undef assert

enum LOGS_LEVEL { LOGS_DEBUG, LOGS_INFO, LOGS_RUN, LOGS_OFF };

// We signal production code with explicit define, not NDEBUG. This is coz NDEBUG
// removes asserts everywhere (including mynewt etc), and these asserts are generally
// not just 'test/debug' cases, but also 'badness reboot recovery' cases... 
// Release code therefore just removes logs and other debug/test code surrounded by #ifndef RELEASE_BUILD/#endif
#if MYNEWT_VAL(BUILD_RELEASE)
#ifndef RELEASE_BUILD 
#define RELEASE_BUILD (1)
#endif

#define log_debug(__s, ...) {do{}while(0);}
#define log_info(__s, ...) {do{}while(0);}
#define log_noout(__s, ...) {do{}while(0);}
#define log_blocking(__s, ...) {do{}while(0);}

#else /* BUILD_RELEASE */
// Ensure define is not defined when not in release
#ifdef RELEASE_BUILD 
#undef RELEASE_BUILD
#endif

#define log_debug log_debug_fn
#define log_info log_info_fn
#define log_noout log_noout_fn
#define log_blocking log_blocking_fn

#endif /* BUILD_RELEASE */

// Always have 'run' level operation : assert, warn/error and function logging
// NOTE : global assert defined in libc/assert.h, defined to call OS_CRASH() defined in os/os_fault.h which is the __assert_func()
// defined in the MCU specific os_fault.c (os/src/arch/src/cortex_m3/os_fault.c for example)
// This can be set to call the os_assert_cb() function by defined OS_ASSERT_CB: 1, which is defined in wutils.c to call the same wassert_fn() as here...
// fun times... 
#ifdef NDEBUG
// When removing assert, beware of 'unused variable' warnings. This 'void' of assert avoids that but at the cost of requiring the vars
// in the test to exist... which isn't quite C standard... note lack of ; at end...
#define assert(__e) {do{ (void)sizeof((__e)); }while(0);}
#else /* NDEBUG */ 
#if MYNEWT_VAL(OS_CRASH_FILE_LINE)
    #define assert(__e) ((__e) ? (void)0 : wassert_fn(__FILE__, __LINE__))
#else
    #define assert(__e) ((__e) ? (void)0 : wassert_fn(NULL, 0))
#endif
#endif /* NDEBUG */
#define log_warn log_warn_fn
#define log_error log_error_fn
#define log_fn log_fn_fn

// Utility functions tried and tested
// sw test assert failure
void wassert_fn(const char* file, int lnum);
// hw test assert failure
void wassert_hw_fault();
// logging
void log_init_console(bool enable);
void log_init_dbg(uint8_t u);
void log_config_uart(const char* dev, uint32_t baud, int8_t uartSelect);
int log_init_uart();
void log_deinit_uart();
bool log_check_uart_active();
void log_debug_fn(const char* sl, ...);
void log_info_fn(const char* sl, ...);
void log_warn_fn(const char* sl, ...);
void log_error_fn(const char* sl, ...);
void log_noout_fn(const char* sl, ...);
void log_blocking_fn(int u, const char* sl, ...);
void log_fn_fn();
uint8_t get_log_level();
const char* get_log_level_str();
void set_log_level(uint8_t lev);
// More utility functions
/*
 * Write a 32 bit unsigned int as LE format into a buffer at specified offset. 
 */
void Util_writeLE_uint32_t(uint8_t* b, uint8_t offset, uint32_t v);
/*
 * Write a 32 bit signed int as LE format into a buffer at specified offset. 
 */
void Util_writeLE_int32_t(uint8_t* b, uint8_t offset, int32_t v); 
/*
 * Write a 16 bit unsigned int as LE format into a buffer at specified offset. 
 */
void Util_writeLE_uint16_t(uint8_t* b, uint8_t offset, uint16_t v);
/*
 * Write a 16 bit signed int as LE format into a buffer at specified offset. 
 */
void Util_writeLE_int16_t(uint8_t* b, uint8_t offset, int16_t v); 
/*
 * helper to read 16 bit LE from buffer (may be 0 stripped)
 */
uint16_t Util_readLE_uint16_t(uint8_t* b, uint8_t l);
/*
 * helper to read 32 bit LE from buffer (may be 0 stripped)
 */
uint32_t Util_readLE_uint32_t(uint8_t* b, uint8_t l);

// Unittest support
#if MYNEWT_VAL(UNITTEST) 
#define UNITTEST (1)
bool unittest(const char* tn, bool res);

// add your unittest fns here
bool unittest_gps();
bool unittest_cfg();
#endif 

#ifdef __cplusplus
}
#endif

#endif  /* H_WUTILS_H */
