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
// how many per-event timers can you have?
#define MAX_PER_EVT_TIMERS MYNEWT_VAL(SM_MAX_EVENT_TIMERS)

// this is the list of events waiting to be executed for all state machines
typedef struct sm_event {
    SM_ID_t sm_id;
    int e;
    void* data;
} SM_EVENT_t;
typedef struct {
    const SM_STATE_t* sm_table;
    uint8_t sz;
    void* ctxarg;
    const SM_STATE_t* currentState;
    struct os_callout timer;
    struct sm_evttimers {
        int e;
        struct os_callout t;
        SM_ID_t s;                // must hold our own SM pointer to be able to recover it in the timer cb
    } evttimers[MAX_PER_EVT_TIMERS];
} SM_t;

static os_stack_t _sm_task_stack[SM_TASK_STACK_SZ];
static struct os_task sm_mgr_task_str;

static SM_t _smTable[SM_MAX_SMS];
static uint8_t _smIdx = 0;

static SM_EVENT_t _sm_event_list[SM_MAX_EVENTS];
static uint8_t _sm_event_list_head=0;
static uint8_t _sm_event_list_tail=0;

static struct os_event _sm_schedule_event;
static struct os_eventq _sm_EQ;

// predeclare privates
static void sm_mgr_task(void* arg);
static void sm_timer_cb(struct os_event* ev);
static void sm_timerE_cb(struct os_event* ev);
static void sm_nextevent_cb(struct os_event* ev);

static uint8_t circListNext(uint8_t* idx, uint8_t sz);
static bool circListFull(uint8_t head, uint8_t tail, uint8_t sz);
static bool circListEmpty(uint8_t head, uint8_t tail, uint8_t sz);
static const SM_STATE_t* findStateFromId(SM_STATE_ID_t id, const SM_STATE_t* table, uint8_t sz);

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
SM_ID_t sm_init(const char* name, const SM_STATE_t* states, uint8_t sz, SM_STATE_ID_t initialState, void* ctxarg) {
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
    // TODO mutex protect list accesses
    if (circListFull(_sm_event_list_head, _sm_event_list_tail, SM_MAX_EVENTS)) {
        log_warn("!!!aie!!!");
        return false;
    }
    uint8_t idx = circListNext(&_sm_event_list_tail, SM_MAX_EVENTS);
    _sm_event_list[idx].sm_id = id;
    _sm_event_list[idx].e = e;
    _sm_event_list[idx].data = data;
    // Schedule event handler if not already waiting to run (does all SM events on list)
    os_eventq_put(&_sm_EQ, &_sm_schedule_event);
    return true;
}
void sm_timer_start(SM_ID_t id, uint32_t tms) {
    SM_t* sm = (SM_t*)id;
    os_time_t ticks;
    os_time_ms_to_ticks(tms, &ticks);
    // Not required to explicitly stop timer if it was running, reset stops it
    os_callout_reset(&(sm->timer), ticks);
}
void sm_timer_stop(SM_ID_t id) {
    SM_t* sm = (SM_t*)id;
    os_callout_stop(&(sm->timer));
    uint8_t head = _sm_event_list_head;
    // REMOVE ANY TIMEOUT EVENTS FOR THIS SM FROM EVENT Q (in case it popped but is now cancelled)
    // TODO mutex protect list accesses
    while (!circListEmpty(head, _sm_event_list_tail, SM_MAX_EVENTS)) {
        // get events 
        SM_EVENT_t* evt = &_sm_event_list[circListNext(&head, SM_MAX_EVENTS)];
        // If its a timeout event for this SM, invalidate it
        if (evt->sm_id==id && evt->e==SM_TIMEOUT) {
            evt->sm_id = NULL;      // so it gets ignored by sm_nextevent_sb processing
        }
    }
}
// Start timer for tms ms from now that will send event e to SM id when it pops. If there is already a timer for event e, the
// timeout is reset to tms ms from now. If the current state of the SM changes, these timers ARE NOT STOPPED.
void sm_timer_startE(SM_ID_t id, uint32_t tms, int e) {
    // need a list of individual timers per event value...
    // try to stop it first to get any associated events off list
    sm_timer_stopE(id, e);
    // Now start it
    SM_t* sm = (SM_t*)id;
    // Find an empty timer for this event (shouldn't exist already as the stop removes it...)
    for(int i=0;i<MAX_PER_EVT_TIMERS;i++) {
        if (sm->evttimers[i].s==NULL)  {
            sm->evttimers[i].e = e;
            sm->evttimers[i].s = id;
            os_time_t ticks;
            os_time_ms_to_ticks(tms, &ticks);
            os_callout_init(&(sm->evttimers[i].t), &_sm_EQ, sm_timerE_cb, &(sm->evttimers[i]));
            os_callout_reset(&(sm->evttimers[i].t), ticks);      
            return;
        }
    }
    // oops too many running per-event timers...
    log_error("SM: per-event timers overflow for sm %x", sm->sm_table);
    assert(0);
    return;
}
// Stop the timer that is sending event e
void sm_timer_stopE(SM_ID_t id, int e) {
    SM_t* sm = (SM_t*)id;
    // find the appropriate timer associated with this event and cancel it
    for(int i=0;i<MAX_PER_EVT_TIMERS;i++) {
        if (sm->evttimers[i].s!=NULL && sm->evttimers[i].e==e) {
            // gotcha
            os_callout_stop(&(sm->evttimers[i].t));
            sm->evttimers[i].s = NULL;      // its gone
            sm->evttimers[i].e = -1;      // its gone
        }
    }
    // Not an issue if we don't find it...
    // find any event in the pending events list with this event and remove it (timer has already popped but not executed)
    // TODO mutex protect list accesses
    uint8_t head = _sm_event_list_head;
    while (!circListEmpty(head, _sm_event_list_tail, SM_MAX_EVENTS)) {
        // get events 
        SM_EVENT_t* evt = &_sm_event_list[circListNext(&head, SM_MAX_EVENTS)];
        // If its a timeout event for this SM, invalidate it
        if (evt->sm_id==id && evt->e==e) {
            evt->sm_id = NULL;      // so it gets ignored by sm_nextevent_sb processing
        }
    }
}

// Get current state
SM_STATE_ID_t sm_getCurrentState(SM_ID_t id) {
    SM_t* sm = (SM_t*)id;
    return sm->currentState->id;
}

/** default log for unhandled event in a state to make debugging easier and centralised */
void sm_default_event_log(SM_ID_t id, const char* log, int e) {
    log_debug("SM:%s:[%s] ignored %d", log, ((SM_t*)id)->currentState->name, e);
}

// Callouts
static void sm_timer_cb(struct os_event* e) {
    sm_sendEvent((SM_ID_t)(e->ev_arg), SM_TIMEOUT, NULL);
}
// for case with specific event
static void sm_timerE_cb(struct os_event* e) {
    // not the sm id, its the per-event structure
    struct sm_evttimers* et = (struct sm_evttimers *)(e->ev_arg);
    // send the event
    sm_sendEvent(et->s, et->e, NULL);
    // free the slot
    et->s = NULL;
    et->e = -1;
}

static void sm_nextevent_cb(struct os_event* e) {
    // pop events until empty
    while(!circListEmpty(_sm_event_list_head, _sm_event_list_tail, SM_MAX_EVENTS)) {
        // call SM with event
        SM_EVENT_t* evt = &_sm_event_list[circListNext(&_sm_event_list_head, SM_MAX_EVENTS)];
        if (evt->sm_id!=NULL) {
            SM_t* sm = (SM_t*)(evt->sm_id);
            SM_STATE_ID_t nextState = (sm->currentState->fn)(sm->ctxarg, evt->e, evt->data);
            // Check if change of state
            if (nextState!=SM_STATE_CURRENT) {
                // ensure timer is stopped before entering next state
                sm_timer_stop(sm);
                const SM_STATE_t* next = findStateFromId(nextState, sm->sm_table, sm->sz);
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
        } // else this event was cancelled whilest in the q, ignore it
    }
}

// task just sends the events on the global list into SMs using event to run the task
static void sm_mgr_task(void* arg) {
    while(1) {
        os_eventq_run(&_sm_EQ);
    }
}

static const SM_STATE_t* findStateFromId(SM_STATE_ID_t id, const SM_STATE_t* table, uint8_t sz) {
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