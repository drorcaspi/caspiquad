#ifndef __I2C_H__
#define __I2C_H__

//=============================================================================
//
// I2C  Module Public API
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

//=============================================================================
//
// Public Definitions
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
i2c_init();


//================================= i2c_write_8() =============================
//
// Write an 8-bit value to a register
//

void
i2c_write_8(uint8_t device_addr,
            uint8_t register_addr,
            uint8_t register_data);


//=========================== i2c_start_read() ================================
//
// Start a read cycle
//

void
i2c_start_read(uint8_t device_addr,
               uint8_t register_addr,
               uint8_t bytes_num);


//=========================== i2c_read_next_8() ===============================
//
// Read next 8 bits
//

uint8_t
i2c_read_next_8(void);


//=========================== i2c_read_next_16() ==============================
//
// Read next 16 bits
//

uint16_t
i2c_read_next_16(void);


//================================== i2c_read_8() =============================
//
// Read 8 bits
//

uint8_t
i2c_read_8(uint8_t device_addr,
           uint8_t register_addr);


//================================== i2c_read_16() ============================
//
// Read 16 bits
//

uint16_t i2c_read_16(uint8_t device_addr,
                     uint8_t register_addr);


//================================== i2c_read_24() ============================
//
// Read 24 bits
//

uint32_t i2c_read_24(uint8_t device_addr,
                     uint8_t register_addr);

#endif
