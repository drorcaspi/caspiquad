//=============================================================================
//
// Status Indicators
//
//=============================================================================

/*-----------------------------------------------------------------------------
  CaspiQuad 1
  Copyright (c) 2009 Dror Caspi.  All rights reserved.

  Based on AeroQuad (http://www.AeroQuad.info)
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
-----------------------------------------------------------------------------*/

#include "quad.h"
#include "indicator.h"


//=============================================================================
//
// Indicator Definitions
//
//=============================================================================

#define PATTERN_REPEAT ((uint8_t)-1)
#define PATTERN_ONCE   ((uint8_t)-2)
#define PATTERN_END    ((uint8_t) 0)


//=============================================================================
//
// Static Variables
//
//=============================================================================

Indicator::Status Indicator::status        = Indicator::NONE;
Indicator::Status Indicator::temp_status   = Indicator::NONE;
uint8_t           Indicator::led_cycle_counter;
uint8_t           Indicator::led_pattern_counter;

const uint8_t     Indicator::none_led_pattern[]        = {PATTERN_ONCE, PATTERN_END};
const uint8_t     Indicator::setup_led_pattern[]       = {PATTERN_REPEAT,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END};
const uint8_t     Indicator::setup_next_led_pattern[]  = {PATTERN_ONCE,
                                                          5, 5,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END};
const uint8_t     Indicator::setup_err_led_pattern[]   = {PATTERN_ONCE,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END};
const uint8_t     Indicator::arming_led_pattern[]      = {PATTERN_ONCE,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          PATTERN_END};
const uint8_t     Indicator::flight_led_pattern[]      = {PATTERN_REPEAT,
                                                          50, 50,
                                                          PATTERN_END};
const uint8_t     Indicator::bat_warning_led_pattern[] = {PATTERN_REPEAT,
                                                          25, 25,
                                                          PATTERN_END};
const uint8_t     Indicator::bat_empty_led_pattern[]   = {PATTERN_REPEAT,
                                                          5, 25,
                                                          PATTERN_END};
const uint8_t     Indicator::hw_err_led_pattern[]      = {PATTERN_REPEAT,
                                                          5, 5,
                                                          PATTERN_END};
const uint8_t     Indicator::sw_warn_led_pattern[]     = {PATTERN_ONCE,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END};
const uint8_t     Indicator::sw_err_led_pattern[]      = {PATTERN_REPEAT,
                                                          10, 10,
                                                          PATTERN_END};

const uint8_t *const Indicator::p_patterns[] =
                     {
                       Indicator::none_led_pattern,
                       Indicator::setup_led_pattern,
                       Indicator::setup_next_led_pattern,
                       Indicator::setup_err_led_pattern,
                       Indicator::arming_led_pattern,
                       Indicator::flight_led_pattern,
                       Indicator::bat_warning_led_pattern,
                       Indicator::bat_empty_led_pattern,
                       Indicator::hw_err_led_pattern,
                       Indicator::sw_warn_led_pattern,
                       Indicator::sw_err_led_pattern
                     };


//=============================== init() ====================================
//
// Initialize the statues indicators

void Indicator::init(void)

{
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  digitalWrite(LED_PIN, 0);
  digitalWrite(BUZZER_PIN, 0);
};


//============================== update() ===================================
//
// Called periodically

Indicator::Status         // Ret: Indicared status indicated.
Indicator::update(void)   // In:  Status to indicate

{
  Status         current_status;
  const uint8_t *p_pattern;

  
  if (temp_status != NONE)
    current_status = temp_status;

  else
    current_status = status;

#if 0    
  Serial.print(status, DEC);
  Serial.print("\t");
  Serial.print(temp_status, DEC);
  Serial.print("\t");
  Serial.print(current_status, DEC);
  Serial.print("\t");
  Serial.print(led_cycle_counter, DEC);
  Serial.print("\t");
  Serial.println(led_pattern_counter, DEC);
#endif  

  if (current_status != NONE)
  {
    p_pattern = p_patterns[current_status];

    if (--led_cycle_counter == 0)
    {
      if (p_pattern[++led_pattern_counter] == PATTERN_END)
      {
        // End of pattern.

        temp_status = NONE;  // Even if we we're displaying temp

        indicate(status);
      }

      else
      {
        // Get next item in pattern
        
        led_cycle_counter = p_pattern[led_pattern_counter];
        //Serial.println(led_cycle_counter, DEC);
        digitalWrite(LED_PIN, led_pattern_counter & 1);
      }
    }
  }

  return current_status;
};


//============================= indicate() ==================================

void
Indicator::indicate(Status status_in)   // In:  Status to indicate

{
  const uint8_t *p_pattern;


  if (status_in == NONE)
  {
    status = NONE;
    temp_status = NONE;
  }

  else
  {
    p_pattern = p_patterns[status_in];

    if (p_pattern[0] == PATTERN_ONCE)
      temp_status = status_in;

    else
      status = status_in;
#if 0
    Serial.print(p_pattern[0], DEC);
    Serial.print("\t");
    Serial.print(status, DEC);
    Serial.print("\t");
    Serial.println(temp_status, DEC);
#endif
    led_pattern_counter = 1;
    led_cycle_counter = p_pattern[1];
    //Serial.println(led_cycle_counter, DEC);
    digitalWrite(LED_PIN, 1);
  }
};

