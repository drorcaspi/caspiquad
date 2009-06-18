#ifndef __RECEIVER_H__
#define __RECEIVER_H__

//=============================================================================
//
// Receiver API
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


// Receiver Channels

#define THROTTLE_CH   0
#define ROLL_CH       1   // Aileron
#define PITCH_CH      2   // Elevator
#define YAW_CH        3   // Rudder
#define GEAR_CH       4
#define AUX1_CH       5
#define NUM_CH        6   // Number of channels
#define FIRST_CH      THROTTLE_CH

// Receiver Tick defines the measuement units of raw receiver data.  Currently
// this is 1 uSec but it may change for the sake of optimization.

#define RECEIVER_TICK 1   // uSec

// Nominal minimum, middle and maximum values of raw receiver data

#define RECEIVER_NOM_MIN (1000 / RECEIVER_TICK)
#define RECEIVER_NOM_MID (1500 / RECEIVER_TICK)
#define RECEIVER_NOM_MAX (2000 / RECEIVER_TICK)


//============================ receiver_init() ================================
//
// Initialize the receiver module
// Should be called on system initalization

boolean             // Ret: true if OK, false if failed
receiver_init(void);


//======================== receiver_get_status() ==============================
//
// Get the current status of the receiver

boolean                    // Ret: true if OK, false if not OK
receiver_get_status(void);


//====================== receiver_get_current_raw() ===========================
//
// Get the current raw receiver data (that has been read before from the h/w)

uint16_t                              // Ret: raw data, in units of RECEIVER_TICK
receiver_get_current_raw(uint8_t ch); // In:  channel


//========================== receiver_print_stats() ===========================
//
// Print some statistics (for debug)

void receiver_print_stats(void);


#endif
