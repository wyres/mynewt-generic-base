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

#ifdef NDEBUG
// TODO check if this is ok to remove the calls completely from the binary
#define assert(__e) {do{}while(0);}
#define log_debug(__s) {do{}while(0);}
#define log_info(__s) {do{}while(0);}
#define log_warn(__s) {do{}while(0);}
#define log_error(__s) {do{}while(0);}
#define log_noout(__s) {do{}while(0);}
#define log_blocking(__s) {do{}while(0);}
#define log_fn(__s) {do{}while(0);}

#else /* NDEBUG */

#if MYNEWT_VAL(OS_CRASH_FILE_LINE)
    #define assert(__e) ((__e) ? (void)0 : wassert_fn(__FILE__, __LINE__))
#else
    #define assert(__e) ((__e) ? (void)0 : wassert_fn(NULL, 0))
#endif
#define log_debug log_debug_fn
#define log_info log_debug_fn
#define log_warn log_warn_fn
#define log_error log_error_fn
#define log_noout log_noout_fn
#define log_blocking log_blocking_fn
#define log_fn log_fn_fn

#endif /* NDEBUG */


// Utility functions tried and tested
void wassert_fn(const char* file, int lnum);
// logging
void log_init_console(bool enable);
void log_init_dbg(uint8_t u);
int log_init_uart(const char* dev, uint32_t baud, int8_t uartSelect);
void log_debug_fn(const char* sl, ...);
void log_warn_fn(const char* sl, ...);
void log_error_fn(const char* sl, ...);
void log_noout_fn(const char* sl, ...);
void log_blocking_fn(int u, const char* sl, ...);
void log_fn_fn();


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
