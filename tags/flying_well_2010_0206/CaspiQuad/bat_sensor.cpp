//=============================================================================
//
// Battery Sensor Handler
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
#include "adc.h"
#include "bat_sensor.h"


//=============================================================================
//
// Battery Sensor Definitions
//
//=============================================================================

// Sensor output voltage thresholds, in volts

//#define BAT_SENSOR_WARN_THR_V          2.0   // TODO: ????????
//#define BAT_SENSOR_LOW_THR_V           1.0   // TODO: ????????

// Derived Definitions in Numbers

#define BAT_SENSOR_NO_BAT              200
#define BAT_SENSOR_ONE_BAT             400
#define BAT_SENSOR_WARN_THR            750
#define BAT_SENSOR_LOW_THR             720


//=============================================================================
//
// Public Functions
//
//=============================================================================

//========================= bat_sensor_init() =================================
//
// Initialize the battery sensor

void
bat_sensor_init(void)

{
  // Nothing
}


//========================= bat_sensor_get() ==================================
//
// Get the battery status

BatStatus
bat_sensor_get(void)

{
  uint16_t  sense;
  BatStatus status;


  sense = adc_get_data_no_gain(BAT_SENSOR_PIN);

#if PRINT_BAT_SENSOR
  Serial.print("Bat: ");
  Serial.println(sense, DEC);
#endif

#if 1
  if ((sense > BAT_SENSOR_WARN_THR) || (sense < BAT_SENSOR_ONE_BAT))
    status = BAT_OK;
  else if (sense > BAT_SENSOR_LOW_THR)
    status = BAT_WARN;
  else
    status = BAT_LOW;
#else
  status = BAT_OK;
#endif  

  return status;
}

