//=============================================================================
//
// LIS302DL Accelerometers Handler
//
// Uses TWI (I2C) communicate with the accelerometer
//
// Using the Wire library (created by Nicholas Zambetti)
// http://wiring.org.co/reference/libraries/Wire/index.html
// On the Arduino board, Analog In 4 is SDA, Analog In 5 is SCL
// These correspond to pin 27 (PC4/ADC4/SDA) and pin 28 (PC5/ADC5/SCL) on the Atmega8
// The Wire class handles the TWI transactions, abstracting the nitty-gritty to make
// prototyping easy.
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
#include "i2c.h"
#include <avr/pgmspace.h>
#include "atan.h"
#include "accel.h"


//=============================================================================
//
// Local Definitions
//
//=============================================================================

//-----------------------------------------------------------------------------
//
// LIS302DL Accelerometer Definitions
//
//-----------------------------------------------------------------------------

#define LIS302DL_0_ADDRESS      0x1C   // Device Addresss

// Device Registers

#define LIS302DL_WHO_AM_I       0x0F
#define LIS302DL_WHO_AM_I_VALUE 0x3B   // Constant value of WHO_AM_I

#define LIS302DL_CTRL_REG1      0x20

#define LIS302DL_OUT_X          0x29
#define LIS302DL_OUT_Y          0x2B
#define LIS302DL_OUT_Z          0x2D

//-----------------------------------------------------------------------------
//
// Rotation Measurement Definitions
//
//-----------------------------------------------------------------------------

// Minimum Z accelerometer reading, below which the reading is not considered
// reiable for rotation calculation

#define ACCEL_ROTATION_MEASURE_RAW_MIN (ACCEL_TYP_1G / 4)

// Absolute acceleration must be close to 1G for taking into account

#define ACCEL_ROTATION_MEASURE_SQ_MAX  ((ACCEL_MAX_1G * 1.1) * (ACCEL_MAX_1G * 1.1))
#define ACCEL_ROTATION_MEASURE_SQ_MIN  ((ACCEL_MIN_1G * 0.9) * (ACCEL_MIN_1G * 0.9))


//=============================================================================
//
// Static Variables
//
//=============================================================================

// Current accelerometer readings (raw)

static int8_t current_accel_data[NUM_AXES];

#if SUPPORT_ACCEL_CALIBRATION
// Accelerometer readings on a flat surface.  Should be calibrated to acheive
// best results.

static int8_t flat_accel_data[NUM_AXES] = {0, 0, ACCEL_TYP_1G};

// Long-time avergae of accelerometer readings.  Scale is shift left from
// raw readings by ACCEL_LONG_AVG_SHIFT

static int16_t accel_long_avg[NUM_AXES];
#endif


//=============================== accel_init() ================================
//
// Initialize the accelerometers module
// Should be called on system initalization

boolean             // Ret: true if OK, false if failed
accel_init(void)
{
  uint8_t who_am_i_val;
  

  current_accel_data[X_AXIS] = 0;
  current_accel_data[Y_AXIS] = 0;
  current_accel_data[Z_AXIS] = (int8_t)ACCEL_TYP_1G;
      
  // Read the WHO_AM_I register to make sure it's there

  who_am_i_val = i2c_read_8(LIS302DL_0_ADDRESS, LIS302DL_WHO_AM_I);
  if (who_am_i_val != (uint8_t)LIS302DL_WHO_AM_I_VALUE)
  {
#if PRINT_ACCEL
    Serial.print("ACC WAI: ");
    Serial.println(who_am_i_val, HEX);
#endif
    return false;
  }

  // Write CTRL_REG1 

  i2c_write_8(LIS302DL_0_ADDRESS,
              LIS302DL_CTRL_REG1,
              0x47);  // Device on, 100hz, normal mode, all axes enabled,
                      // +/- 2.3G range
  return true;
};


//=============================== accel_update() ==============================
//
// Update the accelerometers readings from the h/w

void
accel_update(void)

{
  current_accel_data[X_AXIS] = (int8_t)i2c_read_8(LIS302DL_0_ADDRESS,
                                                  LIS302DL_OUT_X);
  current_accel_data[Y_AXIS] = (int8_t)i2c_read_8(LIS302DL_0_ADDRESS,
                                                  LIS302DL_OUT_Y);
  current_accel_data[Z_AXIS] = (int8_t)i2c_read_8(LIS302DL_0_ADDRESS,
                                                  LIS302DL_OUT_Z);
}


//============================ accel_get_rotations() ==========================
//
// Get the roll and pitch rotation measurements, based on accelerometer
// readings

void 
accel_get_rotations(
  int16_t rot_rad[2])  // Out: Measured rotation values, in (rad / ROT_RAD)
                       //      ROT_NONE if no valid measurement
                    
{
  int8_t atan_col;
  int8_t atan_row;


  rot_rad[ROLL]  = (int16_t)ROT_NONE;
  rot_rad[PITCH] = (int16_t)ROT_NONE;

  // Use the Z axis accelerometer reading as-is, do not subtract flat-surface
  // reading since we use earth gravity here
  
  atan_row = current_accel_data[Z_AXIS];

  if ((atan_row >= (int8_t)ATAN_ROW_MIN) &&
      (atan_row <= (int8_t)ATAN_ROW_MAX))
  {
    atan_row -= (int8_t)ATAN_ROW_MIN;
    
    // Find roll angle based on atan2(Z, Y)
    // Correct the Y accelerometer reading based on flat surface reading.

    atan_col = current_accel_data[Y_AXIS];
#if SUPPORT_ACCEL_CALIBRATION
    atan_col -= flat_accel_data[Y_AXIS];
#endif
    
    // Get the atan value from the table.  Note that the table resides in
    // program memory, therefore we must use pgm_read_word() to access it.
    
    if (atan_col >= 0)
    {
      if (atan_col <= (int8_t)ATAN_COL_MAX)
        rot_rad[ROLL] = pgm_read_word(&(atan_table[atan_row][atan_col]));
    }
    
    else
    {
      atan_col = -atan_col;
      
      if (atan_col <= (int8_t)ATAN_COL_MAX)
        rot_rad[ROLL] = -pgm_read_word(&(atan_table[atan_row][atan_col]));
    };
    
    // Find pitch angle based on atan2(Z, X)
    // Correct the X accelerometer reading based on flat surface reading.
    
    atan_col = current_accel_data[X_AXIS];
#if SUPPORT_ACCEL_CALIBRATION
    atan_col -= flat_accel_data[X_AXIS];
#endif
    
    if (atan_col >= 0)
    {
      if (atan_col <= (int8_t)ATAN_COL_MAX)
        rot_rad[PITCH] = pgm_read_word(&(atan_table[atan_row][atan_col]));
    }
    
    else
    {
      atan_col = -atan_col;
      
      if (atan_col <= (int8_t)ATAN_COL_MAX)
        rot_rad[PITCH] = -pgm_read_word(&(atan_table[atan_row][atan_col]));
    };
  }

#if PRINT_ACCEL
  Serial.print(rot_rad[ROLL], DEC);
  Serial.print("\t");
  Serial.println(rot_rad[PITCH], DEC);
#endif
}


//========================== accel_get_current() ==============================
//
// Get the current accelerometers data (that has been read before from the h/w)
// This function is intended for reading of telemetry.

void
accel_get_current(int8_t accel_data[NUM_AXES])   // Out: 3 axis data

{
  accel_data[X_AXIS] = current_accel_data[X_AXIS];
  accel_data[Y_AXIS] = current_accel_data[Y_AXIS];
  accel_data[Z_AXIS] = current_accel_data[Z_AXIS];
};


#if PRINT_ACCEL
//========================== accel_print_stats() ==============================
//
// Print some statistics (for debug)

void accel_print_stats(void)

{
  Serial.print(current_accel_data[X_AXIS], DEC);
  Serial.print("\t");
  Serial.print(current_accel_data[Y_AXIS], DEC);
  Serial.print("\t");
  Serial.println(current_accel_data[Z_AXIS], DEC);
};
#endif

