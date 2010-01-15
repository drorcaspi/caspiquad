#ifndef __BARO_H__
#define __BARO_H__

//=============================================================================
//
// Barometric Pressure Sensor API
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

// None


//=============================================================================
//
// Public Functions
//
//=============================================================================

//=============================== baro_init() =================================
//
// Initialize the barometric sensor module
// Should be called on system initalization

boolean             // Ret: true if OK, false if failed
baro_init(void);


//=============================== baro_update() ===============================
//
// Update the barometric sensor readings from the h/w

void
baro_update(void);


//========================= baro_alt_estimate_zero() ==========================
//
// Zero the barometric sensor altitude estimate to the current average

void
baro_alt_estimate_zero(void);


//========================= baro_alt_estimate_get() ===========================
//
// Get the barometric sensor altitude estimate

int16_t
baro_alt_estimate_get(void);

#endif
