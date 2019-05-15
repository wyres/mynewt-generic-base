#ifndef H_SM_EXEC_H
#define H_SM_EXEC_H

#include <stdint.h>
#include <mcu/mcu.h>

#include "os/os.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initial events list. Each SM can create its own enum of events, with its initial element starting at (SM_EXT+1)
// eg enum myEvents { TOTO=(SM_EXT+1), TITI, TATA };
typedef enum { SM_ENTER, SM_EXIT, SM_TIMEOUT, SM_EXT } SM_EVENT_TYPE_t;
//State ids are in the entry in the table of states. Each value must be unique.
// enum MyStates { MS_IDLE, MS_GETTING_GPS, MS_GETTING_BLE, MS_LAST };
typedef int8_t SM_STATE_ID_t;
// Special value to return to stay in same state
#define SM_STATE_CURRENT (-1)
// Each state's function must return the id of the next state to transition, or the value SM_STATE_CURRENT to stay in the same one
// Note that the event 'e' is an int as the event list (SM_EVENT_TYEP_t) can be extended for each SM
// NOTE : for ENTER and EXIT events, no state change is allowed (returned value is ignored).
typedef SM_STATE_ID_t (*SM_STATE_FN_t)(int e, void* data);
typedef struct {
    SM_STATE_ID_t id;         // this is the enum
    const char* name;           // for debug logs
    SM_STATE_FN_t fn;           // the function that deals with events in this state
} SM_STATE_t;

typedef void* SM_ID_t;      // Id is actually pointer to internal struct

/* Caller is responsible for building the array of state fns as a static structure. The id for a state is its index in the table.
 * Start your state machine by doing 
 * sm_sendEvent(id, SM_ENTER, NULL):
 * which calls the initialState function with ENTER event
 */
SM_ID_t sm_init(const char* name, SM_STATE_t* stateTable, uint8_t sz, SM_STATE_ID_t initialState);

// Send an event to the given state machine. Note that e is an int as the event list can be extended
bool sm_sendEvent(SM_ID_t id, int e, void* data);
// Start a timer for tms milliseconds. This will generate a SM_TIMEOUT event.
void sm_timer_start(SM_ID_t id, uint32_t tms);
// Stop the timer. Note this is automatically done for you when state changes
void sm_timer_stop(SM_ID_t id);

#ifdef __cplusplus
}
#endif

#endif  /* H_SM_EXEC_H */
