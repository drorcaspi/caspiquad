#ifndef __INDICATORS_H__
#define __INDICATORS_H__

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
// Public Definitions
//
//=============================================================================

typedef enum
{
  IND_NONE,
  IND_SETUP,       // Normal mode after power up
  IND_SETUP_NEXT1, // Prompt to do next step of setup
  IND_SETUP_NEXT2, // Prompt to do next step of setup
  IND_SETUP_NEXT3, // Prompt to do next step of setup
  IND_SETUP_ERR,   // Some error in setup, e.g., gyros not stable
  IND_ARMING,      // Warning - arming motors
  IND_FLIGHT,      // Everything ready & armed, motors running
  IND_BAT_WARN,    // Battery warning
  IND_BAT_LOW,     // Battery empty
  IND_HW_ERR,      // Hardware error, e.g., can't read accelerometers 
  IND_SW_WARN,     // Software warning (e.g., didn't complete main cycle on time)
  IND_SW_ERR       // Software error
} IndicatorStatus;


//=============================================================================
//
// Public Functions
//
//=============================================================================

//======================== indicators_init() ==================================
//
// Initialize the status indicators
  
void
indicators_init(void);
  

//======================== indicators_update() ================================
//
// Called periodically
  
void
indicators_update(void);


//========================= indicators_set() ==================================
//
// Indicate a new status

void
indicators_set(IndicatorStatus status);  // In:  Status to indicate


//========================= INDICATORS_SET() ==================================
//
// Macro wrapper to indicator_set() to log text to the serial line in debug
// mode

#if INDICATORS_DEBUG_TEXT
#define INDICATORS_SET(_status) ({                           \
                                   Serial.print(__FILE__);   \
                                   Serial.print('#');        \
                                   Serial.print(__LINE__);   \
                                   Serial.print(": ");       \
                                   Serial.println(#_status); \
                                })
#else
#define INDICATORS_SET(_status) indicators_set(_status)
#endif

#endif
