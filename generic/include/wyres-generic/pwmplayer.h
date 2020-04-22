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
#ifndef H_PWMPLAYER_H
#define H_PWMPLAYER_H

#include <inttypes.h>
#include <mcu/mcu.h>

#ifdef __cplusplus
extern "C" {
#endif

// Define a gpio to use for pwm playing and which timer to use for that. name must not be null.
// This must be called before using other methods on this pin
bool PWM_define(const char* name, int8_t gpio, int tim);
// Request for the given PWM 'gpio' to add the playing given sequence to the pin. The pattern is <note><qualifier><duration> where 
// note is a letter from A-G, a-g (a=A4 above middle C 440Hz) OR 'S' for silence, qualifier is '-' (natural) '#'(sharped) or '_' (flattened)
// and duration is 8(minim), 4(crotchet), 2(quaver), 1(demi quaver).
// As there is no 'blank' between notes of the same frequency, other durations can be built from these eg 
// C4C2 : dotted crotchet
// C8C8 : semi-breve
// The beat is given in crotchets per minute (beats)
bool PWM_play(int8_t gpio, const char* pattern, int beat);
// Add a note tone (as used above for the pattern) to the queue for the given pin. Returns ture if ok, false if the queue is full
bool PWM_addNote(int8_t gpio, char note, char modif, char dur, int beat);
// Add a pwm tone to the queue for the given pin. Returns ture if ok, false if the queue is full
bool PWM_addPWM(int8_t gpio, int freq, int duty, int durationMS);
// flush current queue for the given pin
void PWM_flush(int8_t gpio);

#ifdef __cplusplus
}
#endif

#endif  /* H_PWMPLAYER_H */
