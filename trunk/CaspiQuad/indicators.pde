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
#include "indicators.h"


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
// Indicator class is an abstraction of the LED and Buzzer indicators
//
//=============================================================================

class Indicator

{
private:
  const uint8_t         pin;
  IndicatorStatus       status;
  IndicatorStatus       temp_status;
  uint8_t               cycle_counter;
  uint8_t               pattern_counter;
  const uint8_t *const *p_patterns;

public:
  
  //============================ Constructor ==================================
  
  Indicator(uint8_t              pin_in,  // In:  Arduino pin
            const uint8_t *const p_patterns_in[]);
                                          // In:  Pointer to an array of patterns
  
  //=============================== init() ====================================
  //
  // Initialize the statues indicator
  
  void init(void);
  
  //============================== update() ===================================
  //
  // Called periodically
  
  void
  update(void);             // In:  Status to indicate

  //============================= set() =======================================
  
  void
  set(IndicatorStatus status);  // In:  Status to indicate
};


//=============================================================================
//
// Static Variables
//
//=============================================================================

//--------------
// LED Patterns
//--------------

static const uint8_t        none_led_pattern[]        = {PATTERN_END};
static const uint8_t        setup_led_pattern[]       = {
                                                          PATTERN_REPEAT,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END
                                                        };
static const uint8_t        setup_next_led_pattern[]  = {
                                                          PATTERN_ONCE,
                                                          5, 5,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END
                                                        };
static const uint8_t        setup_err_led_pattern[]   = {
                                                          PATTERN_ONCE,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END
                                                        };
static const uint8_t        arming_led_pattern[]      = {
                                                          PATTERN_REPEAT,
                                                          5, 5,
                                                          PATTERN_END
                                                        };
static const uint8_t        flight_led_pattern[]      = {
                                                          PATTERN_REPEAT,
                                                          50, 50,
                                                          PATTERN_END
                                                        };
static const uint8_t        bat_warn_led_pattern[]    = {
                                                          PATTERN_REPEAT,
                                                          25, 25,
                                                          PATTERN_END
                                                        };
static const uint8_t        bat_low_led_pattern[]     = {
                                                          PATTERN_REPEAT,
                                                          5, 15,
                                                          PATTERN_END
                                                        };
static const uint8_t        hw_err_led_pattern[]      = {
                                                          PATTERN_REPEAT,
                                                          5, 5,
                                                          PATTERN_END
                                                        };
static const uint8_t        sw_warn_led_pattern[]     = {
                                                          PATTERN_ONCE,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END
                                                        };
static const uint8_t        sw_err_led_pattern[]      = {
                                                          PATTERN_REPEAT,
                                                          10, 10,
                                                          PATTERN_END
                                                        };

static const uint8_t *const p_led_patterns[] = {
                                                 none_led_pattern,
                                                 setup_led_pattern,
                                                 setup_next_led_pattern,
                                                 setup_err_led_pattern,
                                                 arming_led_pattern,
                                                 flight_led_pattern,
                                                 bat_warn_led_pattern,
                                                 bat_low_led_pattern,
                                                 hw_err_led_pattern,
                                                 sw_warn_led_pattern,
                                                 sw_err_led_pattern
                                               };

//-----------------
// Buzzer Patterns
//-----------------

static const uint8_t        none_buz_pattern[]        = {PATTERN_END};
static const uint8_t        setup_buz_pattern[]       = {
                                                          PATTERN_REPEAT,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END
                                                        };
static const uint8_t        setup_next_buz_pattern[]  = {
                                                          PATTERN_ONCE,
                                                          5, 5,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END
                                                        };
static const uint8_t        setup_err_buz_pattern[]   = {
                                                          PATTERN_ONCE,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END
                                                        };
static const uint8_t        arming_buz_pattern[]      = {
                                                          PATTERN_REPEAT,
                                                          5, 5,
                                                          PATTERN_END
                                                        };
static const uint8_t        flight_buz_pattern[]      = {PATTERN_END};
static const uint8_t        bat_warn_buz_pattern[]    = {
                                                          PATTERN_REPEAT,
                                                          5, 45,
                                                          PATTERN_END
                                                        };
static const uint8_t        bat_low_buz_pattern[]     = {
                                                          PATTERN_REPEAT,
                                                          5, 15,
                                                          PATTERN_END
                                                        };
static const uint8_t        hw_err_buz_pattern[]      = {
                                                          PATTERN_REPEAT,
                                                          5, 5,
                                                          PATTERN_END
                                                        };
static const uint8_t        sw_warn_buz_pattern[]     = {
                                                          PATTERN_ONCE,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END
                                                        };
static const uint8_t        sw_err_buz_pattern[]      = {
                                                          PATTERN_REPEAT,
                                                          10, 10,
                                                          PATTERN_END
                                                        };

static const uint8_t *const p_buz_patterns[] = {
                                                 none_buz_pattern,
                                                 setup_buz_pattern,
                                                 setup_next_buz_pattern,
                                                 setup_err_buz_pattern,
                                                 arming_buz_pattern,
                                                 flight_buz_pattern,
                                                 bat_warn_buz_pattern,
                                                 bat_low_buz_pattern,
                                                 hw_err_buz_pattern,
                                                 sw_warn_buz_pattern,
                                                 sw_err_buz_pattern
                                               };

//-------------------
// Indicator Objects
//-------------------

static Indicator led_indicator(LED_PIN, p_led_patterns);
static Indicator buz_indicator(BUZZER_PIN, p_buz_patterns);


//=============================================================================
//
// Class Indicator Implementation
//
//=============================================================================

//============================ Constructor ==================================
  
Indicator::Indicator(
  uint8_t              pin_in,          // In:  Arduino pin
  const uint8_t *const p_patterns_in[]  // In:  Pointer to an array of patterns
) : pin(pin_in), p_patterns(p_patterns_in)

{
  status = IND_NONE;
  temp_status = IND_NONE;
  pattern_counter = 0;
  cycle_counter = 0;
}


//=============================== init() ====================================
//
// Initialize the statues indicators

void Indicator::init(void)

{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, 0);
};


//============================== update() ===================================
//
// Called periodically

void
Indicator::update(void)   // In:  Status to indicate

{
  IndicatorStatus  current_status;
  const uint8_t   *p_pattern;


  if (pattern_counter != 0)
  {
    // We have a pattern to display
    
    if (temp_status != IND_NONE)
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
    Serial.print(cycle_counter, DEC);
    Serial.print("\t");
    Serial.println(pattern_counter, DEC);
#endif  

    if (current_status != IND_NONE)
    {
      p_pattern = p_patterns[current_status];

      if (--cycle_counter == 0)
      {
        if (p_pattern[++pattern_counter] == PATTERN_END)
        {
          // End of pattern.

          temp_status = IND_NONE;  // Even if we we're displaying temp

          set(status);
        }

        else
        {
          // Get next item in pattern
          
          cycle_counter = p_pattern[pattern_counter];
          
          //Serial.println(cycle_counter, DEC);
          
          digitalWrite(pin, pattern_counter & 1);
        }
      }
    }
  }
};


//================================= set() =====================================

void
Indicator::set(IndicatorStatus status_in)   // In:  Status to indicate

{
  const uint8_t *p_pattern;


  if (status_in == IND_NONE)
  {
    status = IND_NONE;
    temp_status = IND_NONE;
  }

  else
  {
    p_pattern = p_patterns[status_in];

    if (p_pattern[0] == PATTERN_END)
    {
      // PATTERN_END at the start of the pattern means no status indication

      pattern_counter = 0;
      cycle_counter = 0;
      
      digitalWrite(pin, 0);
    }

    else
    {
      // Set the beginning of a new pattern (either temporary or not)
      
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

      // The actual pattern starts at entry #1
      
      pattern_counter = 1;
      cycle_counter = p_pattern[1];

      //Serial.println(cycle_counter, DEC);

      digitalWrite(pin, 1);
    }
  }
};


//=============================================================================
//
// Public Functions
//
//=============================================================================

//======================== indicators_init() ==================================
//
// Initialize the status indicators
  
void
indicators_init(void)

{
  led_indicator.init();
  buz_indicator.init();
};
  

//======================== indicators_update() ================================
//
// Called periodically
  
void
indicators_update(void)

{
  led_indicator.update();
  buz_indicator.update();
};


//========================= indicators_set() ==================================
  
void
indicators_set(IndicatorStatus status)  // In:  Status to indicate

{
  led_indicator.set(status);
  buz_indicator.set(status);
};

