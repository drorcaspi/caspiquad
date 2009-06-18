//=============================================================================
//
// EEPROM Utilities
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
#include "eeprom_utils.h"


//=============================================================================
//
// Private Definitions
//
//=============================================================================

#define EEPROM_VER_ADDR     (EEPROM_UTILS_BASE_ADDR + 0)  // Version #
#define EEPROM_VER_INV_ADDR (EEPROM_UTILS_BASE_ADDR + 1)  // Inverted version #


//=============================================================================
//
// Static Variables
//
//=============================================================================

static boolean eeprom_ok;


//=============================================================================
//
// Public Functions
//
//=============================================================================

//============================= eeprom_init() =================================
//
// Initializes the EEPROM handler.  Compares the version number read from the
// EEPROM with the current number.

void eeprom_init(void)

{
  eeprom_ok = (EEPROM.read(EEPROM_VER_ADDR)     == (uint8_t)EEPROM_VER) &&
              (EEPROM.read(EEPROM_VER_INV_ADDR) == (uint8_t)~EEPROM_VER);
#if PRINT_EEPROM
  Serial.print("EEPROM Read Version: ");
  Serial.print(EEPROM.read(EEPROM_VER_ADDR), HEX);
  Serial.print("  EEPROM Read ~Version: ");
  Serial.print(EEPROM.read(EEPROM_VER_INV_ADDR), HEX);
  Serial.print("  EEPROM OK: ");
  Serial.println(eeprom_ok, DEC);
#endif
};


//============================= eeprom_is_ok() ================================
//
// Returns the EEPROM data validity status

boolean             // Ret: status, true if OK, false if EEPROM is
                    //      does not contain valid data.  In this case it must
                    //      be written first.
eeprom_is_ok(void)

{
  return eeprom_ok;
};


//============================= eeprom_write_ver() ============================
//
// Write the current EEPROM version number

void eeprom_write_ver(void)

{
  EEPROM.write(EEPROM_VER_ADDR, (uint8_t)EEPROM_VER);
  EEPROM.write(EEPROM_VER_INV_ADDR, (uint8_t)~EEPROM_VER);

  eeprom_ok = true;

#if PRINT_EEPROM
  Serial.println("Wrote EEPROM version");
#endif
};


//============================= eeprom_read_float() ===========================
//
// Read a floating point number (4 bytes) from the given EEPROM address

float eeprom_read_float(int address)

{
  float    data;
  uint8_t *p_byte;
  uint8_t  i;


  // Read the data byte-by-byte
  
  p_byte = (uint8_t *)&data;
  
  for (int i = 0; i < sizeof(data); i++) 
    *p_byte++ = EEPROM.read(address++);

#if PRINT_EEPROM
  Serial.print("Read EEPROM @ ");
  Serial.print(address - sizeof(data), HEX);
  Serial.print(": ");
  Serial.println(data);
#endif
  return data;
};


//============================ eeprom_write_float() ===========================
//
// Write a floating point number (4 bytes) to the given EEPROM address

void eeprom_write_float(int   address,
                        float data)

{
  uint8_t *p_byte;
  uint8_t  i;


  // Read the data byte-by-byte
  
  p_byte = (uint8_t *)&data;
  
#if PRINT_EEPROM
  Serial.print("Write EEPROM @ ");
  Serial.print(address, HEX);
  Serial.print(": ");
  Serial.println(data);
#endif

  for (int i = 0; i < sizeof(data); i++) 
    EEPROM.write(address++, *p_byte++);

};

