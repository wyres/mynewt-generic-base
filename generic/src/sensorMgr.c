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
 * Sensor manager. 
 * Works on a start/stop basis, records the values which can be read at any time
 */
#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_i2c.h"

#include "app-core/app_core.h"

#include "wyres-generic/wutils.h"
#include "wyres-generic/timemgr.h"
#include "wyres-generic/gpiomgr.h"

#include "wyres-generic/lowpowermgr.h"
#include "wyres-generic/sensormgr.h"
#include "wyres-generic/configmgr.h"
#include "wyres-generic/ALTI_basic.h"

// debug : must disable ext-button reading if using it for debug output
#if MYNEWT_VAL(UART_DBG)
#undef EXT_BUTTON
#define EXT_BUTTON (-1)
#endif

#define GPIO_ADC1   (-1)        // or EXT_IO is possible
#define CHAN_ADC1   (-1)
#define GPIO_ADC2   (-1)
#define CHAN_ADC2   (-1)

#define MAX_CBS (4)
#define MAX_BUTTONS (2)

#define ADC_MAX_VALUE                               4095    // 12 bits max value
// Should read the factory calibrated vref from the eerom at 0x1FF8 00F8/9
#define ADC_VREF_BANDGAP                            1224    // vRef in mV for ADC

#define LIGHT_MAX				3950
#define LIGHT_MIN				20

// store values in between checks
static struct {
    LP_ID_t lpUserId;
    struct button_cfg {
        int8_t io;
        uint32_t lastButtonPressTS;         // in ms since boot
        uint32_t lastButtonReleaseTS;       // in ms since boot
        uint8_t currButtonState;
        uint8_t lastButtonState;
        uint8_t lastButtonPressType;
        uint8_t lastDebounceButtonState;
        struct os_callout buttonDebounceTimer;
    } buttons[MAX_BUTTONS];
    struct {
        int8_t io;
        SR_BUTTON_CBFN_t fn;
        void* ctx;
    } buttonCBs[MAX_CBS];
    struct {
        SR_NOISE_CBFN_t fn;
        void* ctx;
    } noiseCBs[MAX_CBS];
    uint32_t lastReadTS;                // in seconds since boot
    bool isActive;
    int16_t currTempcC;
    uint16_t currBattmV;
    int32_t currPressurePa;
    uint8_t currLight;
    int16_t lastTempcC;
    uint16_t lastBattmV;
    uint32_t lastPressurePa;
    uint8_t lastLight;
    uint32_t lastNoiseTS;   // in seconds since boot
    uint8_t noiseFreqkHz;
    uint8_t noiseLeveldB;
    uint16_t currADC1mV;
    uint16_t currADC2mV;
    uint16_t lastADC1mV;
    uint16_t lastADC2mV;
} _ctx;    // memset'd all to 0 in init()

// Timeout for I2C accesses in 'ticks'
#define I2C_ACCESS_TIMEOUT (100)
static struct button_cfg* findButton(int io);
static void buttonCB(void* arg);
static bool config();
    // read stuff into current values
static bool readEnv();
static void deconfig();
static uint32_t delta(int a, int b);
static uint8_t deltaPercent(int a, int b);
static void buttonCheckDebounced(struct os_event* e);
static uint8_t mapButton(int in);
static LIGHT_STATE_t mapLight(uint8_t reading);

// Called from sysinit
void SRMgr_init(void) 
{
    memset(&_ctx, 0, sizeof(_ctx));
    for(int i=0;i<MAX_BUTTONS;i++) {
        _ctx.buttons[i].io = -1;        // no buttons defined yet
    }

    // config alti on i2c
    if (ALTI_init() != ALTI_SUCCESS)
    {
        log_warn("SM:Alti_init fail");
        // badness we stop here
        wassert_hw_fault();
    }

    // Register for to set desired low power mode. No need for callback to change setup
    _ctx.lpUserId = LPMgr_register(NULL);
    // We are currently ok with deepsleep during idle as not "started"
    LPMgr_setLPMode(_ctx.lpUserId, LP_DEEPSLEEP);

}

bool SRMgr_start() 
{
    bool ret = true;
    // While active sensing dont turn off periphs please
    LPMgr_setLPMode(_ctx.lpUserId, LP_DOZE);
    // configure GPIOs / I2C for periphs
    ret &= config();
    // read stuff into current values
    readEnv();
    return ret;
}

void SRMgr_stop() 
{
    // read stuff into current values before stopping
    readEnv();
    // deconfigure GPIOS/I2C
    deconfig();
    // can go into deep sleep now as we have deconfigured the periphs
    LPMgr_setLPMode(_ctx.lpUserId, LP_DEEPSLEEP);
}

/** Define a gpio as a button, ie with debounce and IRQ execution */
bool SRMgr_defineButton(int8_t gpio) {
    if (gpio>=0)
    {
        // Find free slot, or if already defined
        int dispo = -1;
        for(int i=0;i<MAX_BUTTONS;i++) {
            if (_ctx.buttons[i].io==gpio) {
                return true;     // its defined already
            }
            if (_ctx.buttons[i].io==-1) {
                dispo = i;
            }
        }
        if (dispo<0) {
            return false;       // not defined already and no free slot
        }
        _ctx.buttons[dispo].io = gpio;
        // Event used for button debounces. Note that N button inputs may be used, defined by specific modules by calls to defineButton().
        os_callout_init(&(_ctx.buttons[dispo].buttonDebounceTimer), os_eventq_dflt_get(), buttonCheckDebounced, (void*)((int)gpio));

        // if a button is enabled, its IRQ etc runs all the time even during deepsleep
        // Same callback used for all buttons with arg being the gpio for the button
        GPIO_define_irq("button", gpio, buttonCB, (void*)((int)gpio), HAL_GPIO_TRIG_BOTH, HAL_GPIO_PULL_UP, LP_DEEPSLEEP, PULL_UP);
        return true;
    }
    return false;
}

/** Register callback for a button identified by its GPIO */
bool SRMgr_registerButtonCB(int8_t butio, SR_BUTTON_CBFN_t cb, void* c) 
{
    // Check button io has been defined and define if possible
    if (SRMgr_defineButton(butio)) {
        for(int i=0;i<MAX_CBS;i++) {
            if (_ctx.buttonCBs[i].fn==NULL) {
                _ctx.buttonCBs[i].io = butio;
                _ctx.buttonCBs[i].fn = cb;
                _ctx.buttonCBs[i].ctx = c;
                return true;
            }
        }
    }
    return false;
}
void SRMgr_unregisterButtonCB(SR_BUTTON_CBFN_t cb) 
{
    for(int i=0;i<MAX_CBS;i++) 
    {
        if (_ctx.buttonCBs[i].fn==cb) 
        {
            _ctx.buttonCBs[i].fn = NULL;
        }
    }
}

// Register callback to be notified when noise is detected (also means micro is active during deep sleep)
bool SRMgr_registerNoiseCB(SR_NOISE_CBFN_t cb, void* c) 
{
    // Activate microphone stuff TODO
    for(int i=0;i<MAX_CBS;i++) 
    {
        if (_ctx.noiseCBs[i].fn==NULL) 
        {
            _ctx.noiseCBs[i].fn = cb;
            _ctx.noiseCBs[i].ctx = c;
            return true;
        }
    }
    return false;       // no space
}
// Remove registration - if noone is registered then micro input only checked at UL time..
void SRMgr_unregisterNoiseCB(SR_NOISE_CBFN_t cb) 
{
    bool atLeastOneCB = false;
    for(int i=0;i<MAX_CBS;i++) {
        if (_ctx.noiseCBs[i].fn==cb) 
        {
            _ctx.noiseCBs[i].fn = NULL;
        }
        if (_ctx.noiseCBs[i].fn!=NULL) 
        {
            atLeastOneCB = true;
        }
    }
    // If no CBs disable noise check task
    if (atLeastOneCB) 
    {
        // TODO
    }
}

bool SRMgr_hasButtonChanged(int8_t io) 
{
    struct button_cfg* bcfg = findButton((int)io);
    if (bcfg!=NULL) {
        return (bcfg->currButtonState != bcfg->lastButtonState);
    } 
    return false;
}
uint8_t SRMgr_getButton(int8_t io) 
{
    readEnv();
    struct button_cfg* bcfg = findButton((int)io);
    if (bcfg!=NULL) {
        return bcfg->currButtonState;
    } 
    return SR_BUTTON_RELEASED;
}
void SRMgr_updateButton(int8_t io) 
{
    readEnv();
    struct button_cfg* bcfg = findButton((int)io);
    if (bcfg!=NULL) {
        bcfg->lastButtonState = bcfg->currButtonState;
    } 
}
uint8_t SRMgr_getLastButtonPressType(int8_t io) 
{
    struct button_cfg* bcfg = findButton((int)io);
    if (bcfg!=NULL) {
        return bcfg->lastButtonPressType;
    } 
    return SR_BUTTON_SHORT;
}
uint32_t SRMgr_getLastButtonPressTS(int8_t io) 
{
    struct button_cfg* bcfg = findButton((int)io);
    if (bcfg!=NULL) {
        return bcfg->lastButtonPressTS;
    } 
    return 0;
}
uint32_t SRMgr_getLastButtonReleaseTS(int8_t io) 
{
    struct button_cfg* bcfg = findButton((int)io);
    if (bcfg!=NULL) {
        return bcfg->lastButtonReleaseTS;
    } 
    return 0;
}

bool SRMgr_hasTempChanged() 
{
    readEnv();      // ensure uptodate
    // significant if > 0,5 degC [values are in 1/100th deg]
    return (delta(_ctx.currTempcC, _ctx.lastTempcC)>50);
}
int16_t SRMgr_getTempcC() 
{
    readEnv();
    return _ctx.currTempcC;        // value in 1/100 C
}
void SRMgr_updateTemp() 
{
    readEnv();
    _ctx.lastTempcC = _ctx.currTempcC;
}

bool SRMgr_hasPressureChanged() 
{
    readEnv();      // ensure uptodate
    // significatif if > 1hPa. Values are in Pa
    return (delta(_ctx.currPressurePa, _ctx.lastPressurePa)>100);
}
int32_t SRMgr_getPressurePa() 
{
    readEnv();
    return _ctx.currPressurePa;       // in Pa
}
void SRMgr_updatePressure() 
{
    readEnv();
    _ctx.lastPressurePa = _ctx.currPressurePa;
}

bool SRMgr_hasBattChanged() 
{
    readEnv();      // ensure uptodate
    // change of >100mV is significatif (values are in mV)
    return (delta(_ctx.currBattmV, _ctx.lastBattmV)>100);
}
uint16_t SRMgr_getBatterymV() 
{
    readEnv();
    return _ctx.currBattmV;         // in mV
}
void SRMgr_updateBatt() 
{
    readEnv();
    _ctx.lastBattmV = _ctx.currBattmV;
}

bool SRMgr_hasLightChanged() 
{
    readEnv();      // ensure uptodate
    // Light level changes are very variable in daylight. Avoid generating too many UL...
    // Map to 'dark', 'interier', 'daylight' and only signal as change if changes state
    LIGHT_STATE_t cs = mapLight(_ctx.currLight);
    LIGHT_STATE_t ls = mapLight(_ctx.lastLight);
//    return (deltaPercent(_ctx.currLight, _ctx.lastLight) > 20);
    return cs != ls;
}
uint8_t SRMgr_getLight() 
{
    readEnv();
    return _ctx.currLight;
}

LIGHT_STATE_t SRMgr_getLightState() {
    return mapLight(SRMgr_getLight());
}

void SRMgr_updateLight() 
{
    readEnv();
    _ctx.lastLight = _ctx.currLight;
}

uint32_t SRMgr_getLastNoiseTimeSecs() 
{
    readEnv();      // ensure uptodate
    return _ctx.lastNoiseTS;
}
uint8_t SRMgr_getNoiseFreqkHz() 
{
    readEnv();      // ensure uptodate
    return _ctx.noiseFreqkHz;
}
uint8_t SRMgr_getNoiseLeveldB() 
{
    readEnv();      // ensure uptodate
    return _ctx.noiseLeveldB;
}

bool SRMgr_hasADC1Changed() 
{
    readEnv();      // ensure uptodate
    // Significatif if >1%
    return (deltaPercent(_ctx.currADC1mV, _ctx.lastADC1mV)>5);
}
uint16_t SRMgr_getADC1mV() 
{
    readEnv();
    return _ctx.currADC1mV;
}
void SRMgr_updateADC1() 
{
    readEnv();
    _ctx.lastADC1mV = _ctx.currADC1mV;
}
bool SRMgr_hasADC2Changed() 
{
    readEnv();      // ensure uptodate
    return (deltaPercent(_ctx.currADC2mV, _ctx.lastADC2mV)>5);
}
uint16_t SRMgr_getADC2mV() 
{
    readEnv();
    return _ctx.currADC2mV;
}
void SRMgr_updateADC2() 
{
    readEnv();
    _ctx.lastADC2mV = _ctx.currADC2mV;
}

// internals
//buttons
static struct button_cfg* findButton(int io) {
    for(int i=0;i<MAX_BUTTONS;i++) {
        if (_ctx.buttons[i].io == io) {
            return &_ctx.buttons[i];
        }
    }
    return NULL;      // not found
}
static SR_BUTTON_PRESS_TYPE_t calcButtonPressType(uint32_t durms) 
{
    if (durms<2000) 
    {
        return SR_BUTTON_SHORT;
    }
    if (durms<5000) 
    {
        return SR_BUTTON_MED;
    }
    if (durms<10000) 
    {
        return SR_BUTTON_LONG;
    }
    return SR_BUTTON_VLONG;
}
static uint8_t mapButton(int in) 
{
    // gpio input is 0 when pressed, 1 when released; map to 0 for release, 1 for pressed
    return (in>0 ? SR_BUTTON_RELEASED : SR_BUTTON_PRESSED);
}

// CB called each time button changes state
// Arg is the io of the gpio
static void buttonCB(void* arg) 
{
    struct button_cfg* bcfg = findButton((int)arg);
    if (bcfg!=NULL) {
        // we just schedule a timeout for in 100ms to check new state (debounce) IFF its not already scheduled
        if (os_callout_queued(&bcfg->buttonDebounceTimer) == false) 
        {
            bcfg->lastDebounceButtonState = bcfg->currButtonState;        // record the button state before first transition
            os_callout_reset(&bcfg->buttonDebounceTimer, OS_TICKS_PER_SEC/10);
        } // else ignore
    }
}

// Called 100ms after first change of button state. Check the state now that it has settled
static void buttonCheckDebounced(struct os_event* e) 
{
    // Check which button we're dealing with ('arg' is the gpio of the button for thie event)
    struct button_cfg* bcfg = findButton((int)(e->ev_arg));
    if (bcfg!=NULL) {
        // Read the button state NOW
        bcfg->currButtonState = mapButton(GPIO_read(bcfg->io));
        // And compare to when it first changed - only if its still different will we deal with it
        if (bcfg->currButtonState != bcfg->lastDebounceButtonState) 
        {
            // Manage "short press", "2s press", "5s press", "10s press" etc
            if (bcfg->currButtonState==SR_BUTTON_PRESSED) 
            {
                // Pressed
                bcfg->lastButtonPressTS = TMMgr_getRelTimeMS();
                // TODO start timer to do on-going button press length checks (ie signal 'long press' while not yet released)

            } 
            else 
            {
                // released
                bcfg->lastButtonReleaseTS = TMMgr_getRelTimeMS();
                // Manage "short press", "2s press", "5s press", "10s press" etc
                bcfg->lastButtonPressType = calcButtonPressType(bcfg->lastButtonReleaseTS - bcfg->lastButtonPressTS);
            }
            // call callbacks
            for(int i=0;i<MAX_CBS;i++) 
            {
                // If callback defined and its for the io that caused event, then call it
                if (_ctx.buttonCBs[i].fn!=NULL && _ctx.buttonCBs[i].io==bcfg->io) 
                {
                    (*(_ctx.buttonCBs[i].fn))(_ctx.buttonCBs[i].ctx, bcfg->currButtonState, 
                            calcButtonPressType(TMMgr_getRelTimeMS() - bcfg->lastButtonPressTS));
                }
            }
        } // else it toggled but settled into same state as before -> no transition
    }
}
static bool config() 
{
    if (!_ctx.isActive) {
        _ctx.isActive = true;
        // config GPIOS
        // Note the ADC ones will work but return 0 on read if ADC not enabled
        if (LIGHT_SENSOR>=0) 
        {
            // Must activate GPIO that provides power to it
            GPIO_define_out("micropower", SENSOR_PWR, 1, LP_DOZE, OUT_0);
            GPIO_define_adc("light", LIGHT_SENSOR, LIGHT_SENSOR_ADCCHAN, LP_DOZE, HIGH_Z);
    //        log_debug("SM light");
        }
        if (GPIO_ADC1>=0) 
        {
            GPIO_define_adc("adc1", GPIO_ADC1, CHAN_ADC1, LP_DOZE, HIGH_Z);
        }
        if (GPIO_ADC2>=0) 
        {
            GPIO_define_adc("adc2", GPIO_ADC2, CHAN_ADC2, LP_DOZE, HIGH_Z);
        }
        if (BATTERY_GPIO>=0) 
        {
            GPIO_define_adc("battery", BATTERY_GPIO, BATTERY_ADCCHAN, LP_DOZE, HIGH_Z); //TODO : double check "HIGH_Z"
    //        log_debug("SM:batt");
        }
        if (ALTI_activate() != ALTI_SUCCESS)
        {
            log_warn("SM:Erractivate alti");
            return false;
        }

        // config noise detector on micro
        // TODO
    }
    return true;
}


// read stuff into current values
// TODO add ability to average/smooth certain values?
static bool readEnv() 
{
    bool ret=true;
    if (_ctx.isActive) 
    {
        _ctx.lastReadTS = TMMgr_getRelTimeSecs();

        for(int i=0;i<MAX_BUTTONS;i++) {
            if (_ctx.buttons[i].io>=0) {
                _ctx.buttons[i].currButtonState = mapButton(GPIO_read(_ctx.buttons[i].io));
            }
        }
        if (BATTERY_GPIO>=0) 
        {
            int newvalue = GPIO_readADC(BATTERY_GPIO);       
            // We don't use the VREF from calibValues here, rather a constant calculation
            // calculate the Voltage in millivolt
            if (newvalue > 0) {
                newvalue = ( uint32_t )ADC_VREF_BANDGAP * ( uint32_t )ADC_MAX_VALUE / ( uint32_t )newvalue;
            }
            _ctx.currBattmV = (newvalue + _ctx.currBattmV)/2;   // Average it a little as battery value changes slowly over time
//            log_debug("S bat %d", _ctx.currBattmV);
        }
        if (LIGHT_SENSOR>=0) 
        {
            uint16_t rawLightLevel = 0;
            uint16_t formatedLightLevel = 0;
            rawLightLevel = GPIO_readADC(LIGHT_SENSOR);  
            if (rawLightLevel > LIGHT_MAX)
            {
                formatedLightLevel = 0xFF;
            }
            else if(rawLightLevel < LIGHT_MIN)
            {
                formatedLightLevel = 0;
            }
            else
            {
                formatedLightLevel = ((rawLightLevel * 0xFF) / LIGHT_MAX);
            }
            _ctx.currLight = (uint8_t)formatedLightLevel;
//            log_debug("S lum %d", _ctx.currLight);
        }
        if (GPIO_ADC1>=0) 
        {
            _ctx.currADC1mV = GPIO_readADC(GPIO_ADC1);
        }
        if (GPIO_ADC2>=0) 
        {
            _ctx.currADC2mV = GPIO_readADC(GPIO_ADC2);
        }
        if (ALTI_readAllData(&_ctx.currPressurePa, &_ctx.currTempcC) != ALTI_SUCCESS)
        {
            log_warn("SM:Err read alti");
            ret = false;
        } else {
            //            log_debug("SM:temp %d", _ctx.currTempdC);
            //            log_debug("SM:press %d", _ctx.currPressurePa);
        }
    }
    return ret;
}


static void deconfig() 
{
    if (_ctx.isActive) {
        _ctx.isActive = false;

        // remove config GPIOS
        if (LIGHT_SENSOR>=0) 
        {
            GPIO_release(LIGHT_SENSOR);
            GPIO_release(SENSOR_PWR);
        }
        if (GPIO_ADC1>=0) 
        {
            GPIO_release(GPIO_ADC1);
        }
        if (GPIO_ADC2>=0) 
        {
            GPIO_release(GPIO_ADC2);
        }
        if (BATTERY_GPIO>=0) 
        {
            GPIO_release(BATTERY_GPIO);
        }
        // accelero power state controlled by MovementMgr, no need for us to tell him

        // sleep the alti on i2c
        if (ALTI_sleep() != ALTI_SUCCESS)
        {
            log_warn("SM:Err sleep alti");
        }

        // TODO
        // config noise detector on micro
        // TODO
    }
}

// map light level to a state
static LIGHT_STATE_t mapLight(uint8_t reading) {
    if (reading < 2) {
        return DARK;
    } else if (reading <25) {
        return INTERIER;
    } else {
        return DAYLIGHT;
    }
}
// Change in percent between two values
static uint8_t deltaPercent(int a, int b) {
    int r = max(max(a,b),1);
    return ((delta(a,b)*100)/r);
}

static uint32_t delta(int a, int b) 
{
    if (a>b) 
    {
        return (uint32_t)(a-b);
    }
    return (uint32_t)(b-a);
}

