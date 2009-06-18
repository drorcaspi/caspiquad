#ifndef __INDICATOR_H__
#define __INDICATOR_H__

//=============================================================================
//
// Status Indicators API
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


//=============================================================================
//
// Indicator class is an abstraction of the LED and Buzzer indicators
//
//=============================================================================

class Indicator

{
public:
  enum Status
  {
    NONE,
    SETUP,       // Normal mode after power up
    SETUP_NEXT,  // Prompt to do next step of setup
    SETUP_ERR,   // Some error in setup, e.g., gyros not stable
    ARMING,      // Warning - arming motors
    FLIGHT,      // Everything ready & armed, motors running
    BAT_WARN,    // Battery warning
    BAT_EMPTY,   // Battery empty
    HW_ERR,      // Hardware error, e.g., can't read accelerometers 
    SW_WARN,     // Software warning (e.g., didn't complete main cycle on time)
    SW_ERR       // Software error
  };
  
  //=============================== init() ====================================
  //
  // Initialize the statues indicators
  
  static void init(void);
  
  //============================== update() ===================================
  //
  // Called periodically
  
  static
  Status                    // Ret: Indicared status indicated.
  update(void);             // In:  Status to indicate

  //============================= indicate() ==================================
  
  static
  void
  indicate(Status status);  // In:  Status to indicate

private:
  static Status         status;
  static Status         temp_status;
  static uint8_t        led_cycle_counter;
  static uint8_t        led_pattern_counter;

  static const uint8_t  none_led_pattern[];
  static const uint8_t  setup_led_pattern[];
  static const uint8_t  setup_next_led_pattern[];
  static const uint8_t  setup_err_led_pattern[];
  static const uint8_t  arming_led_pattern[];
  static const uint8_t  flight_led_pattern[];
  static const uint8_t  bat_warning_led_pattern[];
  static const uint8_t  bat_empty_led_pattern[];
  static const uint8_t  hw_err_led_pattern[];
  static const uint8_t  sw_warn_led_pattern[];
  static const uint8_t  sw_err_led_pattern[];

  static const uint8_t *const p_patterns[];
};


#endif
