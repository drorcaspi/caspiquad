#ifndef __EEPROM_UTILS_H__
#define __EEPROM_UTILS_H__

//=============================================================================
//
// EEPROM Utilities API
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

#define EEPROM_VER 3  // Version #.  Increment when changes are not backward-
                      // compatible.
                      
//-----------------------------------------------------------------------------
// EEPROM Base Addresses
// 
// Base addresses are defined per functional block.  Offsets of individual
// variables stored in the EEPROM are the responsibility of each block.
//-----------------------------------------------------------------------------

#define EEPROM_UTILS_BASE_ADDR           0  // EEPROM Handler block
#define EEPROM_FLIGHT_CONTROL_BASE_ADDR  8


//=============================================================================
//
// Public Functions
//
//=============================================================================

//============================= eeprom_init() =================================
//
// Initializes the EEPROM handler.  Compares the version number read from the
// EEPROM with the current number.

void eeprom_init(void);


//============================= eeprom_is_ok() ================================
//
// Returns the EEPROM data validity status

boolean             // Ret: status, true if OK, false if EEPROM is
                    //      does not contain valid data.  In this case it must
                    //      be written first.
eeprom_is_ok(void);


//============================= eeprom_write_ver() ============================
//
// Write the current EEPROM version number

void eeprom_write_ver(void);


//============================= eeprom_read_float() ===========================
//
// Read a floating point number (4 bytes) from the given EEPROM address

float eeprom_read_float(int address);


//============================ eeprom_write_float() ===========================
//
// Write a floating point number (4 bytes) to the given EEPROM address

void eeprom_write_float(int   address,
                        float data);


#endif
