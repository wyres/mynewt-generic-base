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
#ifndef H_SM_EXEC_H
#define H_SM_EXEC_H

#include <stdint.h>
#include <mcu/mcu.h>

#include "os/os.h"

#ifdef __cplusplus
extern "C" {
#endif

// Standard events list. Each SM should add its own enum of event ids, with its initial element starting at 0 (and not more than 9999!)
// eg 
// enum MyEvents { ME_GPS_FIX, ME_BLE_SCAN_DONE, ME_LORA_RESULT };
typedef enum { SM_ENTER=10000, SM_EXIT, SM_TIMEOUT } SM_EVENT_TYPE_t;

//State ids are in the entry in the table of states. Each value must be unique.
// eg : 
// enum MyStates { MS_IDLE, MS_GETTING_GPS, MS_GETTING_BLE, MS_LAST };
typedef int8_t SM_STATE_ID_t;
// Special value to return to stay in same state
#define SM_STATE_CURRENT (-1)

// Each state's function must return the id of the next state to transition, or the value SM_STATE_CURRENT to stay in the same one
// Note that the event 'e' is an int as the event list (SM_EVENT_TYEP_t) can be extended for each SM
// ctxarg is the value as supplied to the sm_init() 
// NOTE : for ENTER and EXIT events, no state change is allowed (returned value is ignored).
typedef SM_STATE_ID_t (*SM_STATE_FN_t)(void* ctxarg, int e, void* data);
typedef struct {
    SM_STATE_ID_t id;         // this is the enum
    const char* name;           // for debug logs
    SM_STATE_FN_t fn;           // the function that deals with events in this state
} SM_STATE_t;

typedef void* SM_ID_t;      // Id is actually pointer to internal struct

/* Caller is responsible for building the array of state fns as a static structure. (MUST BE STATIC FOR LIFETIME OF EXECUTION)
 * The order is not important, only the mapping between the .id and the .fn.
 * eg:
SM_STATE_t _mySM[MS_LAST] = {
    {.id=MS_IDLE, .name="Idle", .fn=State_Idle},
    {.id=MS_GETTING_GPS, .name="GettingGPS", .fn=State_GettingGPS},
    {.id=MS_GETTING_BLE, .name="GettingBLE", .fn=State_GettingBLE},    
    {.id=MS_SENDING_DM, .name="SendingDM", .fn=State_SendingDM},    
};
 * Call sm_init() with with this table, its size, and the initial state, and the ctx arg passed to state functions (can be NULL)
 */
SM_ID_t sm_init(const char* name, const SM_STATE_t* stateTable, uint8_t sz, SM_STATE_ID_t initialState, void* ctxarg);
/*
 * To start your state machine do 
sm_start(id):
 * which calls the function associated with the 'initialState' id with an ENTER event.
 */
bool sm_start(SM_ID_t id);
// Send an event to the given state machine. Note that e is an int as the event list can be extended
bool sm_sendEvent(SM_ID_t id, int e, void* data);
// Start a timer for tms milliseconds. This will generate a SM_TIMEOUT event.
void sm_timer_start(SM_ID_t id, uint32_t tms);
// Stop the timer. Note this is automatically done for you when state changes
void sm_timer_stop(SM_ID_t id);
// Start timer for tms ms from now that will send event e to SM id when it pops. If there is already a timer for event e, the
// timeout is reset to tms ms from now. If the current state of the SM changes, these timers ARE NOT STOPPED (to allow cross-state timer)
// If a timer is already running for this SM with this event, it is reset (stop/start) even if it has already popped but not yet been executed.
void sm_timer_startE(SM_ID_t id, uint32_t tms, int e);
// Stop the timer that is sending event e
void sm_timer_stopE(SM_ID_t id, int e);

// What state is this machine currently in?
SM_STATE_ID_t sm_getCurrentState(SM_ID_t id);

#ifdef __cplusplus
}
#endif

#endif  /* H_SM_EXEC_H */
