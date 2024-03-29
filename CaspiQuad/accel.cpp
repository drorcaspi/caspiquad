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
#include "axis_translation.h"
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
// Accelerometer Low-Pass Filtering Definitions
//
//-----------------------------------------------------------------------------

#define ACCEL_SHIFT             3


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


//-----------------------------------------------------------------------------
//
// Earth-Axis Translation Definitions
//
//-----------------------------------------------------------------------------

// Shift factor between accelerometer units  and estimated acceleration

#define ACCEL_TRANSLATION_SHIFT  6


//=============================================================================
//
// Static Variables
//
//=============================================================================

// Current accelerometer readings (raw)

static int16_t current_accel_data[NUM_AXES];

#if SUPPORT_ACCEL_CALIBRATION
// Accelerometer readings on a flat surface.  Should be calibrated to acheive
// best results.

static int8_t flat_accel_data[NUM_AXES] = {0, 0, ACCEL_TYP_1G};

// Long-time avergae of accelerometer readings.  Scale is shift left from
// raw readings by ACCEL_LONG_AVG_SHIFT

static int16_t accel_long_avg[NUM_AXES];
#endif

// Earth-axis Z acceleration @ 1G (static)

static int16_t accel_ez_1g;


//=============================== accel_init() ================================
//
// Initialize the accelerometers module
// Should be called on system initalization

boolean             // Ret: true if OK, false if failed
accel_init(void)
{
  uint8_t who_am_i_val;
  boolean status;
  

  current_accel_data[X_AXIS] = 0;
  current_accel_data[Y_AXIS] = 0;
  current_accel_data[Z_AXIS] = (int16_t)ACCEL_TYP_1G;
      
  // Read the WHO_AM_I register to make sure it's there

  who_am_i_val = i2c_read_8(LIS302DL_0_ADDRESS, LIS302DL_WHO_AM_I, &status);
  if (! status)
    return false;
  
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

boolean             // Ret: true if OK, false if failed
accel_update(void)

{
  boolean status;


  current_accel_data[X_AXIS] -= current_accel_data[X_AXIS] >> ACCEL_SHIFT;
  current_accel_data[X_AXIS] += (int16_t)(int8_t)i2c_read_8(LIS302DL_0_ADDRESS,
                                                            LIS302DL_OUT_X,
                                                            &status);
  
  if (status)
  {
    current_accel_data[Y_AXIS] -= current_accel_data[Y_AXIS] >> ACCEL_SHIFT;
    current_accel_data[Y_AXIS] += (int16_t)(int8_t)i2c_read_8(LIS302DL_0_ADDRESS,
                                                              LIS302DL_OUT_Y,
                                                              &status);
  }
  
  if (status)
  {
    current_accel_data[Z_AXIS] -= current_accel_data[Z_AXIS] >> ACCEL_SHIFT;
    current_accel_data[Z_AXIS] += (int16_t)(int8_t)i2c_read_8(LIS302DL_0_ADDRESS,
                                                              LIS302DL_OUT_Z,
                                                              &status);
  }

  return status;
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
  
  atan_row = current_accel_data[Z_AXIS] >> ACCEL_SHIFT;

  if ((atan_row >= (int8_t)ATAN_ROW_MIN) &&
      (atan_row <= (int8_t)ATAN_ROW_MAX))
  {
    atan_row -= (int8_t)ATAN_ROW_MIN;
    
    // Find roll angle based on atan2(Z, Y)
    // Correct the Y accelerometer reading based on flat surface reading.

    atan_col = current_accel_data[Y_AXIS] >> ACCEL_SHIFT;
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
    
    atan_col = current_accel_data[X_AXIS] >> ACCEL_SHIFT;
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


//============================ accel_zero_earth_z() ===========================
//
// Measure the earth 1G at rest

void
accel_zero_earth_z(void)

{
  accel_ez_1g = sqrt(((int32_t)current_accel_data[X_AXIS] * (int32_t)current_accel_data[X_AXIS]) +
                     ((int32_t)current_accel_data[Y_AXIS] * (int32_t)current_accel_data[Y_AXIS]) +
                     ((int32_t)current_accel_data[Z_AXIS] * (int32_t)current_accel_data[Z_AXIS]));
};

  
//============================ accel_estimate_earth_z() =======================
//
// Estimate the acceleration in the earth Z axis, given accelerations along the
// aircraft axes and pitch/roll rotations

int16_t                              // Ret: Estimate of Z acceleration in earth axis 
accel_estimate_earth_z(
  const int16_t rot_rad[2])          // In:  Measured rotation values, in (rad / ROT_RAD)
                                     //      ROT_NONE if no valid measurement
                    
{
  uint8_t rot;
  int16_t temp_rot_rad;
  uint8_t i_table[2];
  boolean rot_negative[2];
  int16_t z_accel;
#if PRINT_TRANSLATED_ACCEL
  static float velocity = 0;
#endif


  // Convert rotation angle inputs to table indices.
  
  for (rot = ROLL; rot <= PITCH; rot++)
  {
    temp_rot_rad = rot_rad[rot];
    
#if PRINT_TRANSLATED_ACCEL
    Serial.print(ROT_TO_DEG(temp_rot_rad), DEC);
    Serial.print('\t');
#endif

    // Find absolute value, but remember if it was negative or positive
    // for later usage.
    
    if (temp_rot_rad < 0)
    {
      temp_rot_rad = -temp_rot_rad;
      rot_negative[rot] = true;
    }

    else
      rot_negative[rot] = false;

    // Make sure we don't overflow the tables ranges
    
    if (temp_rot_rad > COEFF_TABLE_MAX_ANGLE)
      temp_rot_rad = COEFF_TABLE_MAX_ANGLE;

    // Round and scale down to table index scale
    
    i_table[rot] = ((uint16_t)temp_rot_rad + (1 << (COEFF_TABLE_ANGLE_SHIFT - 1))) >>
                                                               COEFF_TABLE_ANGLE_SHIFT;
  };

#if PRINT_TRANSLATED_ACCEL
  Serial.print((uint16_t)pgm_read_byte(&(z_acc_z_coeff_table[i_table[ROLL]][i_table[PITCH]])), DEC);
  Serial.print('\t');
  Serial.print((uint16_t)pgm_read_byte(&(z_acc_xy_coeff_table[i_table[ROLL]][i_table[PITCH]])), DEC);
  Serial.print('\t');
  Serial.print((uint16_t)pgm_read_byte(&(z_acc_xy_coeff_table[i_table[PITCH]][i_table[ROLL]])), DEC);
  Serial.print('\t');
#endif

  // Get the measured Z factor from the table and multiple by the
  // measured Z acceleration
  
  z_accel = ((uint8_t)pgm_read_byte(&(z_acc_z_coeff_table[i_table[ROLL]][i_table[PITCH]])) *
             (current_accel_data[Z_AXIS] >> ACCEL_SHIFT)) >> 1;

  // Get the measured X factor from the table and multiply by the
  // measured X acceleration, then subtract or add if the pitch
  // was positive/negative
  
  if (rot_negative[PITCH])
    z_accel -= ((uint8_t)pgm_read_byte(&(z_acc_xy_coeff_table[i_table[ROLL]][i_table[PITCH]])) *
                (current_accel_data[X_AXIS] >> ACCEL_SHIFT)) >> 2;
  else
    z_accel += ((uint8_t)pgm_read_byte(&(z_acc_xy_coeff_table[i_table[ROLL]][i_table[PITCH]])) *
                (current_accel_data[X_AXIS] >> ACCEL_SHIFT)) >> 2;
    
  // Get the measured Y factor from the table and multiply by the
  // measured Y acceleration, then subtract or add if the roll
  // was positive/negative
    
  if (rot_negative[ROLL])
    z_accel -= ((uint8_t)pgm_read_byte(&(z_acc_xy_coeff_table[i_table[PITCH]][i_table[ROLL]])) *
                (current_accel_data[Y_AXIS] >> ACCEL_SHIFT)) >> 2;
  else
    z_accel += ((uint8_t)pgm_read_byte(&(z_acc_xy_coeff_table[i_table[PITCH]][i_table[ROLL]])) *
                (current_accel_data[Y_AXIS] >> ACCEL_SHIFT)) >> 2;

#if PRINT_TRANSLATED_ACCEL
  Serial.print(current_accel_data[X_AXIS] >> ACCEL_SHIFT, DEC);
  Serial.print('\t');
  Serial.print(current_accel_data[Y_AXIS] >> ACCEL_SHIFT, DEC);
  Serial.print('\t');
  Serial.print(current_accel_data[Z_AXIS] >> ACCEL_SHIFT, DEC);
  Serial.print('\t');
  if (rot_negative[PITCH])
    Serial.print('-');
  Serial.print(i_table[PITCH], DEC);
  Serial.print('\t');
  if (rot_negative[ROLL])
    Serial.print('-');
  Serial.print(i_table[ROLL], DEC);
  Serial.print('\t');
  Serial.print(z_accel, DEC);
#endif

  z_accel >>= ACCEL_TRANSLATION_SHIFT;
  z_accel -= (accel_ez_1g >> ACCEL_SHIFT);

#if PRINT_TRANSLATED_ACCEL
  Serial.print('\t');
  Serial.print(z_accel, DEC);
  Serial.print('\t');
  Serial.print((float)z_accel/5.34);    // Scale to 1m/sec^2 
  velocity += ((float)z_accel/534);    // Scale to 1m/sec^2 and cycle time
  Serial.print('\t');
  Serial.print(accel_ez_1g);
  Serial.print('\t');
  Serial.println(velocity);
#endif

  return z_accel;
}


//========================== accel_get_current() ==============================
//
// Get the current accelerometers data (that has been read before from the h/w)
// This function is intended for reading of telemetry.

void
accel_get_current(int8_t accel_data[NUM_AXES])   // Out: 3 axis data

{
  accel_data[X_AXIS] = current_accel_data[X_AXIS] >> ACCEL_SHIFT;
  accel_data[Y_AXIS] = current_accel_data[Y_AXIS] >> ACCEL_SHIFT;
  accel_data[Z_AXIS] = current_accel_data[Z_AXIS] >> ACCEL_SHIFT;
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

