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
  IND_NONE                            =  0,
  IND_SETUP                           =  1, // Normal mode after power up
  IND_SETUP_NEXT1                     =  2, // Prompt to do next step of setup
  IND_SETUP_NEXT2                     =  3, // Prompt to do next step of setup
  IND_SETUP_NEXT3                     =  4, // Prompt to do next step of setup
  IND_SETUP_ERR_SENSORS_SETUP_TIMEOUT =  5, // Some error in setup, e.g., gyros not stable
  IND_SETUP_ERR_THROTTLE_MIN_TIMEOUT  =  6, // Timeout waiting for throttle @ min
  IND_SETUP_ROLL_CENTER               =  7, // Setting the roll trim
  IND_SETUP_PITCH_CENTER              =  8, // Setting the pitch trim
  IND_ROT_POSITIVE                    =  9, // Indicate rotation stick > 0
  IND_ROT_NEGATIVE                    = 10, // Indicate rotation stick < 0
  IND_ARMING                          = 11, // Warning - arming motors
  IND_FLIGHT                          = 12, // Everything ready & armed, motors running
  IND_FLIGHT_ALT_HOLD                 = 13, // Flight with altitude hold active
  IND_FLIGHT_WITH_ACCEL               = 14, // Valid angle estimation using accelerometer
  IND_FLIGHT_WITHOUT_ACCEL            = 15, // No valid angle estimation using accelerometer
  IND_BAT_WARN                        = 16, // Battery warning
  IND_BAT_LOW                         = 17, // Battery empty
  IND_HW_ERR_ACCEL_INIT               = 18, // Hardware error: can't read accelerometers 
  IND_HW_ERR_BARO_INIT                = 19, // Hardware error: can't read barometer 
  IND_SW_WARN_LOOP_CYCLE              = 20, // Software warning: didn't complete main cycle on time
  IND_SW_ERR                          = 21, // Software error
  IND_NUM
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


#endif
