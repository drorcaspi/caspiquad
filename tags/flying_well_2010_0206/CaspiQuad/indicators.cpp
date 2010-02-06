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
#include <avr/pgmspace.h>
#include "indicators.h"


//=============================================================================
//
// Indicator Definitions
//
//=============================================================================

#define INDICATOR_TICK_MSEC  100

// Indicator pattern headers

#define PATTERN_PERMANENT    ((uint8_t)-5)   // Permanent repeated pattern
#define PATTERN_REPEAT       ((uint8_t)-4)   // Repeated pattern
#define PATTERN_ONCE         ((uint8_t)-3)   // One-time pattern
#define PATTERN_ON           ((uint8_t)-2)   // Indicator on
#define PATTERN_OFF          ((uint8_t)-1)   // Indicator off
#define PATTERN_END          ((uint8_t) 0)

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
  set(IndicatorStatus status_in,   // In:  Status to indicate
      boolean         is_force);   // In:  Force update
};


//=============================================================================
//
// Static Variables
//
//=============================================================================

static const uint8_t off_pattern[]             PROGMEM = {PATTERN_OFF};
static const uint8_t on_pattern[]              PROGMEM = {PATTERN_ON};

//--------------
// LED Patterns
//--------------

static const uint8_t setup_led_pattern[]       PROGMEM = {
                                                           PATTERN_REPEAT,
                                                            7, 1,
                                                            1, 1,
                                                           PATTERN_END
                                                         };
static const uint8_t setup_next1_led_pattern[] PROGMEM = {
                                                           PATTERN_ONCE,
                                                           15, 1,
                                                           10,
                                                           PATTERN_END
                                                         };
static const uint8_t setup_next2_led_pattern[] PROGMEM = {
                                                           PATTERN_ONCE,
                                                           15, 1,
                                                            1, 1,
                                                           10,
                                                           PATTERN_END
                                                         };
static const uint8_t setup_next3_led_pattern[] PROGMEM = {
                                                           PATTERN_ONCE,
                                                           15, 1,
                                                            1, 1,
                                                            1, 1,
                                                           10,
                                                           PATTERN_END
                                                         };
static const uint8_t setup_roll_center_led_pattern[] PROGMEM =
                                                         {
                                                           PATTERN_ONCE,
                                                           15, 1,
                                                            1, 1,
                                                            1, 5,
                                                            1, 1,
                                                           10,
                                                           PATTERN_END
                                                         };
static const uint8_t setup_pitch_center_led_pattern[] PROGMEM =
                                                         {
                                                           PATTERN_ONCE,
                                                           15, 1,
                                                            1, 1,
                                                            1, 5,
                                                            1, 5,
                                                            1, 1,
                                                           10,
                                                           PATTERN_END
                                                         };
static const uint8_t rot_positive_led_pattern[] PROGMEM =
                                                         {
                                                           PATTERN_REPEAT,
                                                            5, 1,
                                                            1, 5,
                                                            1, 5,
                                                           PATTERN_END
                                                         };
static const uint8_t rot_negative_led_pattern[] PROGMEM =
                                                         {
                                                           PATTERN_REPEAT,
                                                            5, 1,
                                                            1, 1,
                                                            5, 5,
                                                           PATTERN_END
                                                         };
static const uint8_t setup_err_led_pattern[]   PROGMEM = {
                                                           PATTERN_ONCE,
                                                           15, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                           10,
                                                           PATTERN_END
                                                         };
static const uint8_t arming_led_pattern[]      PROGMEM = {
                                                           PATTERN_ONCE,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                           PATTERN_END
                                                         };
static const uint8_t flight_led_pattern[]      PROGMEM = {
                                                           PATTERN_REPEAT,
                                                            5, 5,
                                                           PATTERN_END
                                                         };
static const uint8_t bat_warn_led_pattern[]    PROGMEM = {
                                                           PATTERN_PERMANENT,
                                                            9, 1,
                                                           PATTERN_END
                                                         };
static const uint8_t bat_low_led_pattern[]     PROGMEM = {
                                                           PATTERN_PERMANENT,
                                                            1, 1,
                                                            3, 1,
                                                           PATTERN_END
                                                         };
static const uint8_t hw_err_led_pattern[]      PROGMEM = {
                                                           PATTERN_PERMANENT,
                                                            1, 1,
                                                           PATTERN_END
                                                         };
static const uint8_t sw_warn_led_pattern[]     PROGMEM = {
                                                           PATTERN_ONCE,
                                                           15, 2,
                                                            2, 2,
                                                            2, 2,
                                                            2, 2,
                                                           10,
                                                           PATTERN_END
                                                         };
static const uint8_t sw_err_led_pattern[]      PROGMEM = {
                                                           PATTERN_PERMANENT,
                                                            2, 2,
                                                           PATTERN_END
                                                         };

static const prog_uint8_t *const p_led_patterns[IND_NUM] PROGMEM = 
{
  off_pattern,                    // IND_NONE
  setup_led_pattern,
  setup_next1_led_pattern,
  setup_next2_led_pattern,
  setup_next3_led_pattern,
  setup_err_led_pattern,          // IND_SETUP_ERR_SENSORS_SETUP_TIMEOUT
  setup_err_led_pattern,          // IND_SETUP_ERR_THROTTLE_MIN_TIMEOUT
  setup_roll_center_led_pattern,  // IND_SETUP_ROLL_CENTER
  setup_pitch_center_led_pattern, // IND_SETUP_PITCH_CENTER
  rot_positive_led_pattern,       // IND_ROT_POSITIVE
  rot_positive_led_pattern,       // IND_ROT_NEGATIVE
  arming_led_pattern,
  flight_led_pattern,             // IND_FLIGHT
  on_pattern,                     // IND_FLIGHT_WITH_ACCEL
  off_pattern,                    // IND_FLIGHT_WITHOUT_ACCEL
  bat_warn_led_pattern,
  bat_low_led_pattern,
  hw_err_led_pattern,             // IND_HW_ERR_ACCEL_INIT
  hw_err_led_pattern,             // IND_HW_ERR_BARO_INIT
  sw_warn_led_pattern,            // IND_SW_WARN_LOOP_CYCLE
  sw_err_led_pattern
};

//-----------------
// Buzzer Patterns
//-----------------

static const uint8_t setup_next1_buz_pattern[] PROGMEM = {
                                                           PATTERN_ONCE,
                                                            1, 1,
                                                            1,
                                                           PATTERN_END
                                                         };
static const uint8_t setup_next2_buz_pattern[] PROGMEM = {
                                                           PATTERN_ONCE,
                                                            1, 1,
                                                            1, 1,
                                                            1,
                                                           PATTERN_END
                                                         };
static const uint8_t setup_next3_buz_pattern[] PROGMEM = {
                                                           PATTERN_ONCE,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1,
                                                           PATTERN_END
                                                         };
static const uint8_t setup_roll_center_buz_pattern[] PROGMEM =
                                                         {
                                                           PATTERN_ONCE,
                                                            1, 1,
                                                            1, 1,
                                                            1, 5,
                                                            1, 1,
                                                           10,
                                                           PATTERN_END
                                                         };
static const uint8_t setup_pitch_center_buz_pattern[] PROGMEM =
                                                         {
                                                           PATTERN_ONCE,
                                                            1, 1,
                                                            1, 1,
                                                            1, 5,
                                                            1, 5,
                                                            1, 1,
                                                           10,
                                                           PATTERN_END
                                                         };
static const uint8_t rot_positive_buz_pattern[] PROGMEM =
                                                         {
                                                           PATTERN_REPEAT,
                                                            5, 1,
                                                            1, 5,
                                                            1, 5,
                                                           PATTERN_END
                                                         };
static const uint8_t rot_negative_buz_pattern[] PROGMEM =
                                                         {
                                                           PATTERN_REPEAT,
                                                            5, 1,
                                                            1, 1,
                                                            5, 5,
                                                           PATTERN_END
                                                         };
static const uint8_t setup_err_buz_pattern[]   PROGMEM = {
                                                           PATTERN_ONCE,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            7,
                                                           PATTERN_END
                                                         };
static const uint8_t arming_buz_pattern[]      PROGMEM = {
                                                           PATTERN_ONCE,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                            1, 1,
                                                           PATTERN_END
                                                         };
static const uint8_t bat_warn_buz_pattern[]    PROGMEM = {
                                                           PATTERN_PERMANENT,
                                                            9, 1,
                                                           PATTERN_END
                                                         };
static const uint8_t bat_low_buz_pattern[]     PROGMEM = {
                                                           PATTERN_PERMANENT,
                                                            1, 1,
                                                            3, 1,
                                                           PATTERN_END
                                                         };
static const uint8_t hw_err_buz_pattern[]      PROGMEM = {
                                                           PATTERN_PERMANENT,
                                                            1, 1,
                                                           PATTERN_END
                                                         };
static const uint8_t sw_warn_buz_pattern[]     PROGMEM = {
                                                           PATTERN_ONCE,
                                                            2, 2,
                                                            2, 2,
                                                            2, 2,
                                                            2, 2,
                                                            6,
                                                           PATTERN_END
                                                         };
static const uint8_t sw_err_buz_pattern[]      PROGMEM = {
                                                           PATTERN_PERMANENT,
                                                            2, 2,
                                                           PATTERN_END
                                                         };

static const prog_uint8_t *const p_buz_patterns[IND_NUM] PROGMEM =
{
  off_pattern,                    // IND_NONE
  off_pattern,                    // IND_SETUP
  setup_next1_buz_pattern,
  setup_next2_buz_pattern,
  setup_next3_buz_pattern,
  setup_err_buz_pattern,          // IND_SETUP_ERR_SENSORS_SETUP_TIMEOUT
  setup_err_buz_pattern,          // IND_SETUP_ERR_THROTTLE_MIN_TIMEOUT
  setup_roll_center_buz_pattern,  // IND_SETUP_ROLL_CENTER
  setup_pitch_center_buz_pattern, // IND_SETUP_PITCH_CENTER
  rot_positive_buz_pattern,       // IND_ROT_POSITIVE
  rot_positive_buz_pattern,       // IND_ROT_NEGATIVE
  arming_buz_pattern,
  off_pattern,                    // IND_FLIGHT
  off_pattern,                    // IND_FLIGHT_WITH_ACCEL
  off_pattern,                    // IND_FLIGHT_WITHOUT_ACCEL
  bat_warn_buz_pattern,
  bat_low_buz_pattern,
  hw_err_buz_pattern,             // IND_HW_ERR_ACCEL_INIT
  hw_err_buz_pattern,             // IND_HW_ERR_BARO_INIT
  sw_warn_buz_pattern,            // IND_SW_WARN_LOOP_CYCLE
  sw_err_buz_pattern
};

//-------------------
// Indicator Objects
//-------------------

static Indicator led_indicator(LED_PIN,    p_led_patterns);
static Indicator buz_indicator(BUZZER_PIN, p_buz_patterns);

// Cycles counter

static uint8_t   indicators_cycle_counter = 0;


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

#if PRINT_INDICATORS
    Serial.print(pin, DEC);
    Serial.print("\t");
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
      p_pattern = (const uint8_t *)pgm_read_word(&p_patterns[current_status]);

      if (--cycle_counter == 0)
      {
        if (pgm_read_byte(&p_pattern[++pattern_counter]) == PATTERN_END)
        {
          // End of pattern.

          temp_status = IND_NONE;  // Even if we we're displaying temp

          set(status, true);
        }

        else
        {
          // Get next item in pattern
          
          cycle_counter = pgm_read_byte(&p_pattern[pattern_counter]);
          
          //Serial.println(cycle_counter, DEC);
          
          digitalWrite(pin, (pattern_counter & 1) ^ 1);
        }
      }
    }
  }
};


//================================= set() =====================================

void
Indicator::set(IndicatorStatus status_in,   // In:  Status to indicate
               boolean         is_force)    // In:  Force update

{
  const uint8_t *p_pattern;
  uint8_t        pattern_mode;
  boolean        is_new;


  p_pattern = (const uint8_t *)pgm_read_word(&p_patterns[status_in]);
  pattern_mode = pgm_read_byte(&p_pattern[0]);

  if (((uint8_t)pgm_read_byte(&(p_patterns[status][0])) != PATTERN_PERMANENT)  ||
      (pattern_mode                                     == PATTERN_PERMANENT))
  {
    // Only a permanent pattern can update a permanent pattern
    
    if (status_in == IND_NONE)
    {
      status = IND_NONE;
      temp_status = IND_NONE;
    }

    else
    {
      is_new = false;
      
      // If the new status is temporary, it updates the temp status only
      // if it is different than it
      
      if (pattern_mode == PATTERN_ONCE)
      {
        // The new status is temporary
        
        if (temp_status != status_in)
        {
          // A new temp status is displayed from the beginning only if it is
          // different than the current temp status
          
          temp_status = status_in;
          is_new = true;
        }
      }

      else
      {
        // The new status is not temporary
        
        if ((temp_status == IND_NONE) && (status != status_in))
        {
          // A new status is displayed from the beginning only if it is
          // different than the current status, and there's no ongoing temp
          // status display.
          
          is_new = true;
        }

        status = status_in;
      };

      if (is_new || is_force)
      {
        if (pattern_mode == PATTERN_OFF)
        {
          // Status indication is constantly off
        
          pattern_counter = 0;
          cycle_counter = 0;
          
          digitalWrite(pin, 0);
        }
        
        else if (pattern_mode == PATTERN_ON)
        {
          // Status indication is constantly on
        
          pattern_counter = 0;
          cycle_counter = 0;
          
          digitalWrite(pin, 1);
        }

        else
        {
          // Set the beginning of a new pattern (either temporary or not)
          // The actual pattern starts at entry #1
          
          pattern_counter = 1;
          cycle_counter = pgm_read_byte(&p_pattern[1]);

          digitalWrite(pin, 0);
        }
      }
    }
  }
  
#if PRINT_INDICATORS
  Serial.print(pin, DEC);
  Serial.print("\t");
  Serial.print(pattern_mode, DEC);
  Serial.print("\t");
  Serial.print(status_in, DEC);
  Serial.print("\t");
  Serial.print(status, DEC);
  Serial.print("\t");
  Serial.print(temp_status, DEC);
  Serial.print("\t");
  Serial.print(pattern_counter, DEC);
  Serial.print("\t");
  Serial.println(cycle_counter, DEC);
#endif
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
  // Update happens every INDICATOR_TICK_MSEC
  
  if (++indicators_cycle_counter >= (uint8_t)(INDICATOR_TICK_MSEC / CONTROL_LOOP_CYCLE_MSEC))
  {
    indicators_cycle_counter = 0;
    
    led_indicator.update();
    buz_indicator.update();
  };
};


//========================= indicators_set() ==================================
//
// Indicate a new status
  
void
indicators_set(IndicatorStatus status)  // In:  Status to indicate

{
#if PRINT_INDICATORS_TEXT
  Serial.print("st: ");
  Serial.println((int)status, DEC);
#endif

  led_indicator.set(status, false);
  buz_indicator.set(status, false);

  // Make sure the first cycle takes a full INDICATOR_TICK_MSEC
  
  indicators_cycle_counter = 0;
};

