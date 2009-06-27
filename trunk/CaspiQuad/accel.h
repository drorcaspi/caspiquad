#ifndef __ACCEL_H__
#define __ACCEL_H__

//=============================================================================
//
// Accelerometer API
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
// Accelerometer Device Definitions
//
// The following defintions are based on the LIS302DL datasheet
//
//=============================================================================

#define ACCEL_MAX          127
#define ACCEL_MIN         -128

#define ACCEL_MIN_RANGE_G    4.0          // Minimal range (G)
#define ACCEL_TYP_RANGE_G    4.6          // Typical range (G)
#define ACCEL_MIN_SENS_G     0.0162       // Minimal sensitivity (G / digit)
#define ACCEL_TYP_SENS_G     0.018        // Typical sensitivity (G / digit)
#define ACCEL_MAX_SENS_G     0.0198       // Maximal sensitivity (G / digit)

// Derived Definitions

#define ACCEL_RANGE   (ACCEL_MAX - ACCEL_MIN + 1)
#define ACCEL_1G      (1.0 / 0.018)   // Nominal accelerometer reading @ 1G
#define ACCEL_MIN_1G  (1.0 / ACCEL_MAX_SENS_G)   // Nominal accelerometer reading @ 1G
#define ACCEL_TYP_1G  (1.0 / ACCEL_TYP_SENS_G)   // Nominal accelerometer reading @ 1G
#define ACCEL_MAX_1G  (1.0 / ACCEL_MIN_SENS_G)   // Nominal accelerometer reading @ 1G


//=============================== accel_init() ================================
//
// Initialize the accelerometers module
// Should be called on system initalization

boolean             // Ret: true if OK, false if failed
accel_init(void);


//=============================== accel_read() ================================
//
// Read the accelerometers

uint16_t                                   // Ret: Sum of 3 axis squared
accel_read(int8_t accel_data[NUM_AXIS]);   // Out: 3 axis data


//========================== accel_get_current() ==============================
//
// Get the current accelerometers data (that has been read before from the h/w)
// This function is intended for reading of telemetry.

uint16_t                                          // Ret: Sum of 3 axis squared
accel_get_current(int8_t accel_data[NUM_AXIS]);   // Out: 3 axis data


//========================== accel_print_stats() ==============================
//
// Print some statistics (for debug)

void accel_print_stats(void);


#endif
