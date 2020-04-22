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

/** Tone player : plays pwm (tone or whatever) sequences using PWM output on one or more IO pins 
 * Uses default event q for callout timers which may impact the precision of the tone lengths
*/

#include <string.h>

#include "sysinit/sysinit.h"
#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#include <math.h>

#include "wyres-generic/wutils.h"
#include "wyres-generic/gpiomgr.h"
#include "wyres-generic/pwmplayer.h"

#define MAX_PWMS    MYNEWT_VAL(MAX_PWMS)

#define QUEUE_SZ    MYNEWT_VAL(PWM_QUEUE_SZ)
// do modulo mapping on a index in q to loop inside QUEUE_SZ
#define MODQ(x) ((x+QUEUE_SZ)%QUEUE_SZ)

#define CROTCHET_NOMINAL_DUR (16)       // Allows to divide down to hemi-demi-semi-quavers

// internals
// structure per gpio output
typedef struct {
    const char* name;       // also used to idicate if entry is in use
    int8_t pin;
    bool playing;
    struct {
        uint8_t head;
        uint8_t tail;
        struct {
            uint16_t freq;
            uint16_t durMS;
            uint8_t duty;
        } tone[QUEUE_SZ];
    } q;
    struct os_callout durTimer;       // For the duration timer for this specific gpio
} PWMGPIO;

static struct {
    PWMGPIO pins[MAX_PWMS];
} _ctx;

// predefine private fns
static bool isPlaying(PWMGPIO* p);
static bool playNextPWMEntry(PWMGPIO* p);
static int findDur(const char dur);
static float findNote(const char note, const char modif);
static PWMGPIO* checkPin(int8_t gpio);
static void tone_ev_cb(struct os_event *ev);

// Public API
// Define a PWM gpio to use - ignored if already defined
bool PWM_define(const char* name, int8_t gpio, int tim) {
    int free = -1;
    for(int i=0;i<MAX_PWMS;i++) {
        if (_ctx.pins[i].name!=NULL && _ctx.pins[i].pin==gpio) {
            return true;
        }
        if (_ctx.pins[i].name==NULL) {
            free = i;       // in case we need it
        }
    }
    if (free<0) {
        return false;        // no spaces
    }
    // Set up entry (compiler set everything to 0 by itself)
    _ctx.pins[free].playing = false;
    _ctx.pins[free].name = name;
    _ctx.pins[free].pin = gpio;
    // Use timer 2???
    GPIO_define_pwm(name, gpio, tim, LP_DOZE, HIGH_Z);

    // Each entry has its own timer, where the arg in the event for the timer callback is the gpio value...
    os_callout_init(&(_ctx.pins[free].durTimer), os_eventq_dflt_get(),
                    &tone_ev_cb, (void*)(&_ctx.pins[free]));
    return true;

}
// Request for the given PWM 'gpio' to add the playing given sequence to the pin. The pattern is <note><duration> where 
// note is a letter from A-G, a-g (C=middle C 440kHz) and duration is m(minim), c(crotchet), q(quaver), d(demi quaver), h.
// As there is no 'blank' between notes of the same frequency, other durations can be built from these eg 
// CcCq : dotted crotchet C
// CcCc : semi-breve C
// The beat is given in crotchets per minute (beats)
bool PWM_play(int8_t gpio, const char* pattern, int beat) {
    PWMGPIO* p = checkPin(gpio);
    if (p==NULL) {
        log_warn("PWM[%d] not found", gpio);
        return false;
    }
    assert(pattern!=NULL);
    // will pattern fit?
    int plen = strnlen(pattern, QUEUE_SZ*3)/3;
    if (plen > (MODQ((p->q.head-1) - p->q.tail))) {
        log_warn("PWM[%d] play too long %d", gpio, plen);
        return false;
    }
    // Add each note to queue
    for(int i=0;i<plen; i++) {
        PWM_addNote(gpio, pattern[i*3], pattern[i*3+1], pattern[i*3+2], beat);
    }
    return true;
}
// Add a note tone (as used above for the pattern) to the queue for the given pin. Returns ture if ok, false if the queue is full
bool PWM_addNote(int8_t gpio, char note, char modif, char dur, int beat) {
    PWMGPIO* p = checkPin(gpio);
    if (p==NULL) {
        return false;
    }
    float freq = findNote(note, modif);
    int durMS = (((findDur(dur) * 1000)/CROTCHET_NOMINAL_DUR) * 60)/beat;      // logically ((note dur in secs if crotchet=1s) / (beat/60)) *1000
    return PWM_addPWM(gpio, (int)freq, 50, durMS);
}
// Add a pwm tone to the queue for the given pin. Returns ture if ok, false if the queue is full
bool PWM_addPWM(int8_t gpio, int freq, int duty, int durationMS) {
    PWMGPIO* p = checkPin(gpio);
    if (p==NULL) {
        return false;
    }
    if (p->q.head == (MODQ(p->q.tail-1))) {
        // full
        return false;
    }
    // Add to queue
    p->q.tone[p->q.head].freq = (uint16_t)freq;
    p->q.tone[p->q.head].duty = (uint8_t)duty;
    p->q.tone[p->q.head].durMS = (uint16_t)durationMS;
    p->q.head = MODQ(p->q.head+1);
    // If it wasn't playing, start
    if (!isPlaying(p)) {
        playNextPWMEntry(p);
    }
    return true;
}
// flush current queue for the given pin and stop any note playing
void PWM_flush(int8_t gpio) {
    PWMGPIO* p = checkPin(gpio);
    if (p==NULL) {
        return;
    }
    // flush q by setting head to tail to 0
    p->q.head = 0;
    p->q.tail = 0;

    // If currently playing (ie timer running), stop callout and the pwm
    if (isPlaying(p)) {
        os_callout_stop(&p->durTimer);
        GPIO_writePWM(p->pin, 0, 0);
        p->playing = false;
    }
}

// privates
static bool isPlaying(PWMGPIO* p) {
    return (p->playing);
}
// Start pwm based on entry at head of queue, and set timer for its duration. 
// If queue is empty, then don't set timer for next time.
static bool playNextPWMEntry(PWMGPIO* p) {
    // Anything on queue?    
    if (p->q.head==p->q.tail) {
        // Stop last note
        GPIO_writePWM(p->pin, 0, 0);
        p->playing=false;
//        log_info("PWM[%d]:play end", p->pin);
        return false;       // nothing to play as tail caught up to head
    }
    p->playing=true;        // now we're playing something (note this applies even if playing silence)
//    log_info("PWM[%d]:@%dHz for %d ms", p->pin, p->q.tone[p->q.tail].freq, p->q.tone[p->q.tail].durMS);

    // Start PWM
    GPIO_writePWM(p->pin, p->q.tone[p->q.tail].freq, p->q.tone[p->q.tail].duty);

    // start timer to move to next entry when this one done
    os_callout_reset(&p->durTimer, (p->q.tone[p->q.tail].durMS * OS_TICKS_PER_SEC)/1000);

    // update queue to next one ready for when this done
    p->q.tail = MODQ(p->q.tail+1);
    return true;
}

static int findDur(const char dur) {
    switch(dur) {
        case 'm': return CROTCHET_NOMINAL_DUR*2;
        case 'c': return CROTCHET_NOMINAL_DUR;
        case 'q': return CROTCHET_NOMINAL_DUR/2;
        case 's': return CROTCHET_NOMINAL_DUR/4;
        case 'd': return CROTCHET_NOMINAL_DUR/8;
        case 'h': return CROTCHET_NOMINAL_DUR/16;
        default : return CROTCHET_NOMINAL_DUR;
    }
}
// Mapping of note names to frequencies. As the intervals are not constant (what?), accidentals are a duplicate of the note below with the correct frequency. Hence
// if you find your note name, then the sharped value is entry+1, and the flattened is entry-1.
// Starts with the 'silent note' at f=0, then a dummy note to handle A_ (A flat).
static struct {
    char n;
    float f;
} NOTE2FREQ[] = {
    {.n='S', .f=0},{.n='Z', .f=207.65},{.n='A', .f=220.0},{.n='A', .f=238.08},{.n='B', .f=246.94},{.n='C', .f=261.63},{.n='C', .f=277.18},{.n='D', .f=293.66},{.n='D', .f=311.13},{.n='E', .f=329.63},{.n='F', .f=349.23},{.n='F', .f=369.99},{.n='G', .f=392.0},{.n='G', .f=415.30},
    {.n='a', .f=440},{.n='a', .f=466.16},{.n='b', .f=493.88},{.n='c', .f=523.25},{.n='c', .f=554.37},{.n='d', .f=587.33},{.n='d', .f=622.25},{.n='e', .f=659.25},{.n='f', .f=698.46},{.n='f', .f=739.99},{.n='g', .f=783.99},{.n='g', .f=830.61},
};
// Get frequency in Hz based on note name and modifier
static float findNote(const char note, const char modif) {
    for(int i=0;i<(sizeof(NOTE2FREQ)/sizeof(NOTE2FREQ[0]));i++) {
        if (NOTE2FREQ[i].n==note) {
            switch(modif) {
                case '#': return NOTE2FREQ[i+1].f;
                case '_':return NOTE2FREQ[i-1].f;
                default: return NOTE2FREQ[i].f;
            }
        }
    }  
    return 440.0;
}

// Find or allocate gpio slot
// Note we use name pointer to indicate used slots, as this way we don't need any init code (auto set to 0 at boot)
static PWMGPIO* checkPin(int8_t gpio) {
    for(int i=0;i<MAX_PWMS;i++) {
        if (_ctx.pins[i].name!=NULL && _ctx.pins[i].pin==gpio) {
            return &_ctx.pins[i];
        }
    }
    return NULL;        // not found
}

// Timer callback : arg is the pin entry
static void tone_ev_cb(struct os_event *ev) {
    PWMGPIO* p = (PWMGPIO*)(ev->ev_arg);
    if (p!=NULL) {
        // Attempt to play next note in queue : deals with empty queues
        playNextPWMEntry(p);
    } else {
        log_warn("PWM event no ev_arg");
    }
}

