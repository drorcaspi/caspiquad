#ifndef __BAT_SENSOR_H__
#define __BAT_SENSOR_H__

//=============================================================================
//
// Battery Sensor API
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

typedef enum
{
  BAT_OK,
  BAT_WARN,
  BAT_LOW
} BatStatus;


//=============================================================================
//
// Public Functions
//
//=============================================================================

//========================= bat_sensor_init() =================================
//
// Initialize the battery sensor

void
bat_sensor_init(void);


//========================= bat_sensor_get() ==================================
//
// Get the battery status

BatStatus
bat_sensor_get(void);

#endif
