//=============================================================================
//
// I2C Utilities
//
//=============================================================================

/*-----------------------------------------------------------------------------
  CaspiQuad 1
  Copyright (c) 2010 Dror Caspi.  All rights reserved.

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
#include <Wire.h>
#include "i2c.h"


//=============================================================================
//
// Private Definitions
//
//=============================================================================

#define I2C_MAX_READ_LOOP 1000   // Set the maximum time to wait on I2C read


//=============================================================================
//
// Static Variables
//
//=============================================================================

//


//=============================================================================
//
// Public Variables
//
//=============================================================================

//


//=============================================================================
//
// Public Functions
//
//=============================================================================

//=========================== i2c_init() ======================================
//
// Initialize the I2C
//

void
i2c_init()

{
  Wire.begin();   // join i2c bus (address optional for master)
}


//================================= i2c_write_8() =============================
//
// Write an 8-bit value to a register
//

void
i2c_write_8(uint8_t device_addr,
             uint8_t register_addr,
             uint8_t register_data)

{
  Wire.beginTransmission(device_addr);
  Wire.send(register_addr); 
  Wire.send(register_data);
  Wire.endTransmission();
}


//=========================== i2c_start_read() ================================
//
// Start a read cycle
//

void
i2c_start_read(uint8_t device_addr,
               uint8_t register_addr,
               uint8_t bytes_num)

{
  Wire.beginTransmission(device_addr);
  Wire.send(register_addr); 
  Wire.endTransmission();
  Wire.requestFrom(device_addr, bytes_num);
}


//=========================== i2c_read_next_8() ===============================
//
// Read next 8 bits
//

uint8_t                              // Ret: value read from I2C
i2c_read_next_8(boolean *p_status)   // Out: is OK?

{
  int16_t i;
  uint8_t value;


  for (i = I2C_MAX_READ_LOOP; (Wire.available() == 0) && (i > 0); i--)
  {
    // Do nothing, wait for available data
  }

  if (i > 0)
  {
    *p_status = true;
    value = Wire.receive();
  }

  else
  {
    *p_status = false;
    value = 0;
  }
  
  return value;
}


//=========================== i2c_read_next_16() ==============================
//
// Read next 16 bits
//

uint16_t                              // Ret: value read from I2C
i2c_read_next_16(boolean *p_status)   // Out: is OK?

{
  uint16_t value;


  value = (uint16_t)i2c_read_next_8(p_status) << 8;
  if (*p_status)
    value += i2c_read_next_8(p_status);
  
  return value; 
}


//================================== i2c_read_8() =============================
//
// Read 8 bits
//

uint8_t                            // Ret: value read from I2C
i2c_read_8(uint8_t device_addr,    // In:  device address
           uint8_t register_addr,  // In:  register address
           boolean *p_status)      // Out: is OK?

{
  i2c_start_read(device_addr, register_addr, 1);
  return i2c_read_next_8(p_status);
}


//================================== i2c_read_16() ============================
//
// Read 16 bits
//

uint16_t                            // Ret: value read from I2C
i2c_read_16(uint8_t device_addr,    // In:  device address
            uint8_t register_addr,  // In:  register address
            boolean *p_status)      // Out: is OK?

{
  i2c_start_read(device_addr, register_addr, 2);
  return i2c_read_next_16(p_status);
}


//================================== i2c_read_24() ============================
//
// Read 24 bits
//

uint32_t                            // Ret: value read from I2C
i2c_read_24(uint8_t device_addr,    // In:  device address
            uint8_t register_addr,  // In:  register address
            boolean *p_status)      // Out: is OK?

{
  uint32_t value;

  
  i2c_start_read(device_addr, register_addr, 3);
  value = (uint32_t)i2c_read_next_8(p_status) << 16;
  if (*p_status)
    value += (uint16_t)i2c_read_next_8(p_status) << 8;
  if (*p_status)
    value += i2c_read_next_8(p_status);

  return value; 
}

