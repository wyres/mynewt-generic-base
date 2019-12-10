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

#define ADC_MAX_VALUE                               4095    // 12 bits max value
// Should read the factory calibrated vref from the eerom at 0x1FF8 00F8/9
#define ADC_VREF_BANDGAP                            1224    // vRef in mV for ADC

#define LIGHT_MAX				3950
#define LIGHT_MIN				20

// store values in between checks
static struct {
    struct {
        SR_BUTTON_CBFN_t fn;
        void* ctx;
    } buttonCBs[MAX_CBS];
    struct {
        SR_NOISE_CBFN_t fn;
        void* ctx;
    } noiseCBs[MAX_CBS];
    uint32_t lastReadTS;
    uint32_t lastSignificantChangeTS;
    uint32_t lastButtonPressTS;
    uint32_t lastButtonReleaseTS;
    uint8_t currButtonState;
    uint8_t lastButtonState;
    uint8_t lastButtonPressType;
    bool isActive;
    int16_t currTempdC;
    uint16_t currBattmV;
    int32_t currPressurePa;
    uint8_t currLight;
    int16_t lastTempdC;
    uint16_t lastBattmV;
    uint32_t lastPressurePa;
    uint8_t lastLight;
    uint32_t lastNoiseTS;
    uint8_t noiseFreqkHz;
    uint8_t noiseLeveldB;
    uint16_t currADC1mV;
    uint16_t currADC2mV;
    uint16_t lastADC1mV;
    uint16_t lastADC2mV;
    uint8_t lastDebounceButtonState;
    struct os_callout buttonDebounceTimer;
} _ctx = {
    // memset'd all to 0 in init()
};

// Timeout for I2C accesses in 'ticks'
#define I2C_ACCESS_TIMEOUT (100)

static void buttonCB(void* arg);
static void config();
    // read stuff into current values
static void readEnv();
static void deconfig();
static uint32_t delta(int a, int b);
static void buttonCheckDebounced(struct os_event* e);
static uint8_t mapButton(int in);

// Called from sysinit
void SRMgr_init(void) 
{
    memset(&_ctx, 0, sizeof(_ctx));
    os_callout_init(&(_ctx.buttonDebounceTimer), os_eventq_dflt_get(), buttonCheckDebounced, &_ctx);
    if (EXT_BUTTON>=0)
    {
        // if ext button is enabled, its IRQ etc runs all the time even during deepsleep
        GPIO_define_irq("button", EXT_BUTTON, buttonCB, &_ctx, HAL_GPIO_TRIG_BOTH, HAL_GPIO_PULL_UP, LP_DEEPSLEEP);
    }

}

void SRMgr_start() 
{
    // configure GPIOs / I2C for periphs
    config();
    _ctx.isActive = true;    
    // read stuff into current values
    readEnv();
}

void SRMgr_stop() 
{
    // read stuff into current values before stopping
    readEnv();
    _ctx.isActive = false;    
    // deconfigure GPIOS/I2C
    deconfig();
}

bool SRMgr_registerButtonCB(SR_BUTTON_CBFN_t cb, void* c) 
{
    if (EXT_BUTTON>=0) {
        for(int i=0;i<MAX_CBS;i++) {
            if (_ctx.buttonCBs[i].fn==NULL) {
                _ctx.buttonCBs[i].fn = cb;
                _ctx.buttonCBs[i].ctx = c;
                return true;
            }
        }
    }
    return false;       // no space or no button defined
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

uint32_t SRMgr_getLastEnvChangeTime() 
{
    return _ctx.lastSignificantChangeTS;
}
bool SRMgr_hasButtonChanged() 
{
    return (_ctx.currButtonState != _ctx.lastButtonState);
}
uint8_t SRMgr_getButton() 
{
    readEnv();
    return _ctx.currButtonState;
}
uint8_t SRMgr_getLastButtonPressType() 
{
    return _ctx.lastButtonPressType;
}
uint32_t SRMgr_getLastButtonPressTS() 
{
    return _ctx.lastButtonPressTS;
}
uint32_t SRMgr_getLastButtonReleaseTS() 
{
    return _ctx.lastButtonReleaseTS;
}
bool SRMgr_hasTempChanged() 
{
    return (delta(_ctx.currTempdC, _ctx.lastTempdC)>2);
}
int16_t SRMgr_getTempdC() 
{
    readEnv();
    return _ctx.currTempdC;        // value in 1/10 C
}
bool SRMgr_hasPressureChanged() 
{
    return (delta(_ctx.currPressurePa, _ctx.lastPressurePa)>10);
}
int32_t SRMgr_getPressurePa() 
{
    readEnv();
    return _ctx.currPressurePa;       // in Pa
}
bool SRMgr_hasBattChanged() 
{
    return (delta(_ctx.currBattmV, _ctx.lastBattmV)>50);
}
uint16_t SRMgr_getBatterymV() 
{
    readEnv();
    return _ctx.currBattmV;         // in mV
}
bool SRMgr_hasLightChanged() 
{
    return (delta(_ctx.currLight, _ctx.lastLight)>2);
}
uint8_t SRMgr_getLight() 
{
    readEnv();
    return _ctx.currLight;
}
uint32_t SRMgr_getLastNoiseTime() 
{
    return _ctx.lastNoiseTS;
}
uint8_t SRMgr_getNoiseFreqkHz() 
{
    return _ctx.noiseFreqkHz;
}
uint8_t SRMgr_getNoiseLeveldB() 
{
    return _ctx.noiseLeveldB;
}

bool SRMgr_hasADC1Changed() 
{
    return (delta(_ctx.currADC1mV, _ctx.lastADC1mV)>50);
}
uint16_t SRMgr_getADC1mV() 
{
    readEnv();
    return _ctx.currADC1mV;
}
bool SRMgr_hasADC2Changed() 
{
    return (delta(_ctx.currADC2mV, _ctx.lastADC2mV)>50);
}
uint16_t SRMgr_getADC2mV() 
{
    readEnv();
    return _ctx.currADC2mV;
}
// Any value that has changed 'significantly' has the last value updated to the current 
// app layer can decide to do this after having read and sent values that had changed
bool SRMgr_updateEnvs() 
{
    bool changed = false;
    // only update those that have changed (otherwise slowly changing values will never be seen as changed)
    if (SRMgr_hasBattChanged()) 
    {
        _ctx.lastBattmV = _ctx.currBattmV;
        changed = true;
    }
    if (SRMgr_hasLightChanged()) 
    {
        _ctx.lastLight = _ctx.currLight;
        changed = true;
    }
    if (SRMgr_hasTempChanged()) 
    {
        _ctx.lastTempdC = _ctx.currTempdC;
        changed = true;
    }
    if (SRMgr_hasPressureChanged()) 
    {
        _ctx.lastPressurePa = _ctx.currPressurePa;
        changed = true;
    }
    if (SRMgr_hasADC1Changed()) 
    {
        _ctx.lastADC1mV = _ctx.currADC1mV;
        changed = true;
    }
    if (SRMgr_hasADC2Changed()) 
    {
        _ctx.lastADC2mV = _ctx.currADC2mV;
        changed = true;
    }
    if (SRMgr_hasButtonChanged()) 
    {
        _ctx.lastButtonState = _ctx.currButtonState;
        changed = true;
    }
    if (changed) 
    {
        _ctx.lastSignificantChangeTS = TMMgr_getRelTime();
    }
    return changed;
}
// internals
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
// CB called each time button changes state
static void buttonCB(void* arg) 
{
    // we just schedule a timeout for in 100ms to check new state (debounce) IFF its not already scheduled
    if (os_callout_queued(&_ctx.buttonDebounceTimer) == false) 
    {
        _ctx.lastDebounceButtonState = _ctx.currButtonState;        // record the button state before first transition
        os_callout_reset(&_ctx.buttonDebounceTimer, OS_TICKS_PER_SEC/10);
    } // else ignore
}

// Called 100ms after first change of button state. Check the state now that it has settled
static void buttonCheckDebounced(struct os_event* e) 
{
    // Read the button state NOW
    _ctx.currButtonState = mapButton(GPIO_read(EXT_BUTTON));
    // And compare to when it first changed - only if its still different will we deal with it
    if (_ctx.currButtonState != _ctx.lastDebounceButtonState) 
    {
        // Manage "short press", "2s press", "5s press", "10s press" etc
        if (_ctx.currButtonState==SR_BUTTON_PRESSED) 
        {
            // Pressed
            _ctx.lastButtonPressTS = TMMgr_getRelTime();
            // TODO start timer to do on-going button press length checks (ie signal 'long press' while not yet released)

        } 
        else 
        {
            // released
            _ctx.lastButtonReleaseTS = TMMgr_getRelTime();
            // Manage "short press", "2s press", "5s press", "10s press" etc
            _ctx.lastButtonPressType = calcButtonPressType(_ctx.lastButtonReleaseTS - _ctx.lastButtonPressTS);
        }
        // call callbacks
        for(int i=0;i<MAX_CBS;i++) 
        {
            if (_ctx.buttonCBs[i].fn!=NULL) 
            {
                (*(_ctx.buttonCBs[i].fn))(_ctx.buttonCBs[i].ctx, _ctx.currButtonState, calcButtonPressType(TMMgr_getRelTime() - _ctx.lastButtonPressTS));
            }
        }
    } // else it toggled but settled into same state as before -> no transition
}
static void config() 
{
    // config GPIOS
    // Note the ADC ones will work but return 0 on read if ADC not enabled
    if (LIGHT_SENSOR>=0) 
    {
        // Must activate GPIO that provides power to it
        GPIO_define_out("micropower", SENSOR_PWR, 1, LP_DOZE);
        GPIO_define_adc("light", LIGHT_SENSOR, LIGHT_SENSOR_ADCCHAN, LP_DOZE);
        log_debug("S adc-light");
    }
    if (GPIO_ADC1>=0) 
    {
        GPIO_define_adc("adc1", GPIO_ADC1, CHAN_ADC1, LP_DOZE);
    }
    if (GPIO_ADC2>=0) 
    {
        GPIO_define_adc("adc2", GPIO_ADC2, CHAN_ADC2, LP_DOZE);
    }
    if (BATTERY_GPIO>=0) 
    {
        GPIO_define_adc("battery", BATTERY_GPIO, BATTERY_ADCCHAN, LP_DOZE);
        log_debug("S adc-batt");
    }
    // config alti on i2c
    // TODO
    if (ALTI_init() != ALTI_SUCCESS)
    {
        log_debug("Failed to activate altimeter");
    }
    // config noise detector on micro
    // TODO
}


// read stuff into current values
static void readEnv() 
{
    if (_ctx.isActive) 
    {
        _ctx.lastReadTS = TMMgr_getRelTime();

        if (EXT_BUTTON>=0) 
        {
            _ctx.currButtonState = mapButton(GPIO_read(EXT_BUTTON));
        }
        if (BATTERY_GPIO>=0) 
        {
            _ctx.currBattmV = GPIO_readADC(BATTERY_GPIO);
            int ref_voltage = ( uint32_t )ADC_VREF_BANDGAP * ( uint32_t )ADC_MAX_VALUE;
            // We don't use the VREF from calibValues here.
            // calculate the Voltage in millivolt
            if (_ctx.currBattmV > 0) {
                _ctx.currBattmV = ref_voltage / ( uint32_t )_ctx.currBattmV;
            }
            log_debug("S bat %d", _ctx.currBattmV);
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
            log_debug("S lum %d", _ctx.currLight);
        }
        if (GPIO_ADC1>=0) 
        {
            _ctx.currADC1mV = GPIO_readADC(GPIO_ADC1);
        }
        if (GPIO_ADC2>=0) 
        {
            _ctx.currADC2mV = GPIO_readADC(GPIO_ADC2);
        }
        if (ALTI_activate() == ALTI_SUCCESS)
        {
            if (ALTI_readAllData(&_ctx.currPressurePa, &_ctx.currTempdC) != ALTI_SUCCESS)
            {
                log_debug("Error while reading altimeter data");
            }
            if (ALTI_sleep() != ALTI_SUCCESS)
            {
                log_debug("Error while putting altimeter in sleep mode");
            }
            log_debug("S temperature %d centi C°", _ctx.currTempdC);
            log_debug("S pressure %d Pa", _ctx.currPressurePa);
        }
        else
        {
            log_debug("Error while activating altimeter");
        }
    }
}


static void deconfig() 
{
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

    // config alti on i2c
    // TODO
    // config noise detector on micro
    // TODO
}

static uint32_t delta(int a, int b) 
{
    if (a>b) 
    {
        return (uint32_t)(a-b);
    }
    return (uint32_t)(b-a);
}

static uint8_t mapButton(int in) 
{
    // gpio input is 0 when pressed, 1 when released; map to 0 for release, 1 for pressed
    return (in>0 ? SR_BUTTON_RELEASED : SR_BUTTON_PRESSED);
}
