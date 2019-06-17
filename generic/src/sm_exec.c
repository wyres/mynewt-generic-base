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
 * Core mechanism for building and executing state machines
 * Multiple state machines can be defined, each with an id
 * Their execution is run on a single task, hence they will be effectivement single threaded and non-re-entrant
 */

#include "os/os.h"
#include "wyres-generic/wutils.h"
#include "wyres-generic/sm_exec.h"

#define SM_TASK_PRIO       MYNEWT_VAL(SM_TASK_PRIO)
#define SM_TASK_STACK_SZ   OS_STACK_ALIGN(512)
#define SM_MAX_EVENTS      MYNEWT_VAL(SM_MAX_EVENTS)
#define SM_MAX_SMS         MYNEWT_VAL(SM_MAX_SMS)

// this is the list of events waiting to be executed for all state machines
typedef struct sm_event {
    SM_ID_t sm_id;
    int e;
    void* data;
} SM_EVENT_t;
typedef struct {
    SM_STATE_t* sm_table;
    uint8_t sz;
    void* ctxarg;
    SM_STATE_t* currentState;
    struct os_callout timer;
} SM_t;

static os_stack_t _sm_task_stack[SM_TASK_STACK_SZ];
static struct os_task sm_mgr_task_str;

static SM_t _smTable[SM_MAX_SMS];
static uint8_t _smIdx = 0;

SM_EVENT_t _sm_event_list[SM_MAX_EVENTS];
uint8_t _sm_event_list_head=0;
uint8_t _sm_event_list_tail=0;

static struct os_event _sm_schedule_event;
static struct os_eventq _sm_EQ;

// predeclare privates
static void sm_mgr_task(void* arg);
static void sm_timer_cb(struct os_event* ev);
static void sm_nextevent_cb(struct os_event* ev);

static uint8_t circListNext(uint8_t* idx, uint8_t sz);
static bool circListFull(uint8_t head, uint8_t tail, uint8_t sz);
static bool circListEmpty(uint8_t head, uint8_t tail, uint8_t sz);
static SM_STATE_t* findStateFromId(SM_STATE_ID_t id, SM_STATE_t* table, uint8_t sz);

// Called from sysinit via reference in pkg.yml
void init_sm_exec(void) {
    _smIdx = 0;
    // setup event lists, task
    // Event list has no entries
    _sm_event_list_head = _sm_event_list_tail = 0;
    _sm_schedule_event.ev_cb = sm_nextevent_cb;
    _sm_schedule_event.ev_arg = _sm_event_list;
    os_eventq_init(&_sm_EQ);
    // Create task 
    os_task_init(&sm_mgr_task_str, "SM_task", sm_mgr_task, NULL, SM_TASK_PRIO,
               OS_WAIT_FOREVER, _sm_task_stack, SM_TASK_STACK_SZ);

}

// PUBLIC : Create an SM and return its reference
SM_ID_t sm_init(const char* name, SM_STATE_t* states, uint8_t sz, SM_STATE_ID_t initialState, void* ctxarg) {
    // Sanity check the table
    assert(sz>0 && sz<255);
    // alloc a space
    SM_t* sm = &_smTable[_smIdx++];
    assert(_smIdx<SM_MAX_SMS);
    sm->sm_table = states;
    sm->sz = sz;
    sm->ctxarg = ctxarg;
    // Set current state to initial one
    sm->currentState = findStateFromId(initialState, sm->sm_table, sm->sz);
    os_callout_init(&(sm->timer), &_sm_EQ, sm_timer_cb, sm);
    return sm;
}

// PULBIC : start a start machine
bool sm_start(SM_ID_t id) {
    return sm_sendEvent(id, SM_ENTER, NULL);    
}

// PUBLIC : send an event to a state machine (called from ext or int)
bool sm_sendEvent(SM_ID_t id, int e, void* data) {
    if (circListFull(_sm_event_list_head, _sm_event_list_tail, SM_MAX_EVENTS)) {
        return false;
    }
    uint8_t idx = circListNext(&_sm_event_list_tail, SM_MAX_EVENTS);
    _sm_event_list[idx].sm_id = id;
    _sm_event_list[idx].e = e;
    _sm_event_list[idx].data = data;
    os_eventq_put(&_sm_EQ, &_sm_schedule_event);
    return true;
}
void sm_timer_start(SM_ID_t id, uint32_t tms) {
    SM_t* sm = (SM_t*)id;
    os_time_t ticks;
    os_time_ms_to_ticks(tms, &ticks);
    // TODO STOP OLD TIMER?
    os_callout_reset(&(sm->timer), ticks);
}
void sm_timer_stop(SM_ID_t id) {
    SM_t* sm = (SM_t*)id;
    // TODO REMOVE ANY TIMEOUT EVENTS FOR THIS SM FROM EVENT Q (in case it popped but is now cancelled)
    os_callout_stop(&(sm->timer));
}

// Callouts
static void sm_timer_cb(struct os_event* e) {
    sm_sendEvent((SM_ID_t)(e->ev_arg), SM_TIMEOUT, NULL);
}

static void sm_nextevent_cb(struct os_event* e) {
    // pop events until empty
    while(!circListEmpty(_sm_event_list_head, _sm_event_list_tail, SM_MAX_EVENTS)) {
        // call SM with event
        SM_EVENT_t* evt = &_sm_event_list[circListNext(&_sm_event_list_head, SM_MAX_EVENTS)];
        SM_t* sm = (SM_t*)(evt->sm_id);
        SM_STATE_ID_t nextState = (sm->currentState->fn)(sm->ctxarg, evt->e, evt->data);
        // Check if change of state
        if (nextState!=SM_STATE_CURRENT) {
            // ensure timer is stopped before entering next state
            sm_timer_stop(sm);
            SM_STATE_t* next = findStateFromId(nextState, sm->sm_table, sm->sz);
            if (next==NULL) {
                // oops
                log_error("SM tries to change to unknown state[%d] from current [%s] on event [%d]", nextState, sm->currentState->name, evt->e);
                assert(0);      // stop right here boys
            }
            // exit previous - you are NOT allowed to change the destination state 
            (sm->currentState->fn)(sm->ctxarg, SM_EXIT, NULL);
            // change to next one
            sm->currentState = next;
            // enter next - allowed to change to a new one? no - in this case you can send yourself your own event...
            (sm->currentState->fn)(sm->ctxarg, SM_ENTER, NULL);
        }
    }
}

// task just runs the calls into SMs using events
static void sm_mgr_task(void* arg) {
    while(1) {
        os_eventq_run(&_sm_EQ);
    }
}

static SM_STATE_t* findStateFromId(SM_STATE_ID_t id, SM_STATE_t* table, uint8_t sz) {
    // start search at the 'idth' element iff id<sz, in case table elements are in the enum order... optimisation... TODO
    for(int i=0;i<sz;i++) {
        if (table[i].id==id) {
            return &table[i];
        }
    }
    return NULL;        // This would be bad
}


static uint8_t circListNext(uint8_t* idx, uint8_t sz) {
    uint8_t ret = *idx;
    *idx = ((*idx)+1)%sz;
    return ret;
}
static bool circListFull(uint8_t head, uint8_t tail, uint8_t sz) {
    if (head==((tail+1)%sz)) {
        return true;
    }
    return false;
}
static bool circListEmpty(uint8_t head, uint8_t tail, uint8_t sz) {
    if (head==tail) {
        return true;
    }
    return false;
}