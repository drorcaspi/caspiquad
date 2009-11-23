#if ESTIMATE_EARTH_ACCEL
//=============================================================================
//
// Acceleration Translation to Earth Axes
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
#include <avr/pgmspace.h>
//#include "accel.h"
#include "accel_translation.h"
#include "axis_translation.h"

//=============================================================================
//
// Local Definitions
//
//=============================================================================

//=============================================================================
//
// Static Variables
//
//=============================================================================


//============================ estimate_earth_z_accel() =======================
//
// Estimate the acceleration in the earth Z axis, given accelerations along the
// aircraft axes and pitch/roll rotations

int16_t                              // Ret: Estimate of Z acceleration in earth axis 
estimate_earth_z_accel(
  const int8_t  meas_accel[NUM_AXES],// In:  Measured accelerations
  const int16_t rot_rad[2])          // In:  Measured rotation values, in (rad / ROT_RAD)
                                     //      ROT_NONE if no valid measurement
                    
{
  uint8_t rot;
  int16_t temp_rot_rad;
  uint8_t i_table[2];
  boolean rot_negative[2];
  int16_t z_accel;
  
  
  // Convert rotation angle inputs to table indices.
  
  for (rot = ROLL; rot <= PITCH; rot++)
  {
    temp_rot_rad = rot_rad[rot];

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
             (int16_t)meas_accel[Z_AXIS]) >> 2;

  // Get the measured X factor from the table and multiply by the
  // measured X acceleration, then subtract or add if the pitch
  // was positive/negative
  
  if (rot_negative[PITCH])
    z_accel += ((uint8_t)pgm_read_byte(&(z_acc_xy_coeff_table[i_table[ROLL]][i_table[PITCH]])) *
                (int16_t)meas_accel[X_AXIS]) >> 2;
  else
    z_accel -= ((uint8_t)pgm_read_byte(&(z_acc_xy_coeff_table[i_table[ROLL]][i_table[PITCH]])) *
                (int16_t)meas_accel[X_AXIS]) >> 2;
    
  // Get the measured Y factor from the table and multiply by the
  // measured Y acceleration, then subtract or add if the roll
  // was positive/negative
    
  if (rot_negative[ROLL])
    z_accel += ((uint8_t)pgm_read_byte(&(z_acc_xy_coeff_table[i_table[PITCH]][i_table[ROLL]])) *
                (int16_t)meas_accel[Y_AXIS]) >> 2;
  else
    z_accel -= ((uint8_t)pgm_read_byte(&(z_acc_xy_coeff_table[i_table[PITCH]][i_table[ROLL]])) *
                (int16_t)meas_accel[Y_AXIS]) >> 2;

#if PRINT_TRANSLATED_ACCEL
  Serial.print((int16_t)meas_accel[X_AXIS], DEC);
  Serial.print('\t');
  Serial.print((int16_t)meas_accel[Y_AXIS], DEC);
  Serial.print('\t');
  Serial.print((int16_t)meas_accel[Z_AXIS], DEC);
  Serial.print('\t');
  if (rot_negative[PITCH])
    Serial.print('-');
  Serial.print(i_table[PITCH], DEC);
  Serial.print('\t');
  if (rot_negative[ROLL])
    Serial.print('-');
  Serial.print(i_table[ROLL], DEC);
  Serial.print('\t');
  Serial.println(z_accel, DEC);
#endif

  return z_accel;
}


#endif

