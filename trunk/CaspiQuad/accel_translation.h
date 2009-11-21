#ifndef __ACCEL_TRANSLATION_H__
#define __ACCEL_TRANSLATION_H__

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


//============================ estimate_earth_z_accel() =======================
//
// Estimate the acceleration in the earth Z axis, given accelerations along the
// aircraft axes and pitch/roll rotations

int16_t                              // Ret: Estimate of Z acceleration in earth axis 
estimate_earth_z_accel(
  const int8_t  meas_accel[NUM_AXES],// In:  Measured accelerations
  const int16_t rot_rad[2]);         // In:  Measured rotation values, in (rad / ROT_RAD)
                                     //      ROT_NONE if no valid measurement

#endif
