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
#include "accel.h"

#include <Wire.h>

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

static int8_t current_accel_data[NUM_AXES];


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
      
  Wire.begin();   // join i2c bus (address optional for master)

  // Read the WHO_AM_I register to make sure it's there
  
  Wire.beginTransmission(LIS302DL_0_ADDRESS);
  Wire.send(LIS302DL_WHO_AM_I); 
  Wire.endTransmission();
  Wire.requestFrom(LIS302DL_0_ADDRESS, 1);
  while(Wire.available())
  {
    who_am_i_val = Wire.receive();
  };

  if (who_am_i_val != LIS302DL_WHO_AM_I_VALUE)
    return false;

  // Write CTRL_REG1    
  Wire.beginTransmission(LIS302DL_0_ADDRESS);
  Wire.send(LIS302DL_CTRL_REG1);
  Wire.send(0x47);    // Device on, 100hz, normal mode, all axes enabled,
                      // +/- 2.3G range
  Wire.endTransmission();

  return true;
};


//=============================== accel_update() ==============================
//
// Update the accelerometers readings from the h/w

void
accel_update(void)

{
  // Read X
  
  Wire.beginTransmission(LIS302DL_0_ADDRESS);
  Wire.send(LIS302DL_OUT_X); 
  Wire.endTransmission();
  Wire.requestFrom(LIS302DL_0_ADDRESS, 1);
  while(Wire.available())
  {
    current_accel_data[X_AXIS] = (int8_t)Wire.receive();
  };

  // Read Y
  
  Wire.beginTransmission(LIS302DL_0_ADDRESS);
  Wire.send(LIS302DL_OUT_Y); 
  Wire.endTransmission();
  Wire.requestFrom(LIS302DL_0_ADDRESS, 1);
  while(Wire.available())
  {
    current_accel_data[Y_AXIS] = (int8_t)Wire.receive();
  };

  // Read Z
  
  Wire.beginTransmission(LIS302DL_0_ADDRESS);
  Wire.send(LIS302DL_OUT_Z); 
  Wire.endTransmission();
  Wire.requestFrom(LIS302DL_0_ADDRESS, 1);
  while(Wire.available())
  {
    current_accel_data[Z_AXIS] = (int8_t)Wire.receive();
  };
}


//============================ accel_get_rotations() ==========================
//
// Get the roll and pitch rotation measurements, based on accelerometer
// readings

void accel_get_rotations(float rot_rad[2])  // Out: Measured rotation values, in rad
                                            //      NAN if no valid measurement
                    
{
  int16_t accel_abs_sq;


  accel_abs_sq =
    ((int16_t)current_accel_data[X_AXIS] * (int16_t)current_accel_data[X_AXIS]) + 
    ((int16_t)current_accel_data[Y_AXIS] * (int16_t)current_accel_data[Y_AXIS]) +
    ((int16_t)current_accel_data[Z_AXIS] * (int16_t)current_accel_data[Z_AXIS]);

  if ((accel_abs_sq <= (uint16_t)ACCEL_ROTATION_MEASURE_SQ_MAX)  &&  // TODO: change the name
      (accel_abs_sq >= (uint16_t)ACCEL_ROTATION_MEASURE_SQ_MIN)  &&
      (current_accel_data[Z_AXIS] >= (int8_t)ACCEL_ROTATION_MEASURE_RAW_MIN))

  {
    // We only calculate the rotation if:
    // - Absolute acceleration is about 1G
    // - Z axis is at least at some minimum (rotation angle not too steep)

    rot_rad[ROLL] = atan2(current_accel_data[Y_AXIS], current_accel_data[Z_AXIS]);
    rot_rad[PITCH] = atan2(current_accel_data[X_AXIS], current_accel_data[Z_AXIS]);
  }

  else
  {
    rot_rad[ROLL] = NAN;
    rot_rad[PITCH] = NAN;
  };
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

