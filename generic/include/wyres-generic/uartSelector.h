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
#ifndef H_UARTSELECTOR_H
#define H_UARTSELECTOR_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

void uart_selector_init(int8_t gpio0, int8_t gpio1);
// Select UART if using uart switch
// id = -1 -> ignored, 0-3 written to gpios if defined
int8_t uart_select(int8_t id);

#ifdef __cplusplus
}
#endif

#endif  /* H_UARTSELECTOR_H */
