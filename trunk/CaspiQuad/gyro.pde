//=============================================================================
//
// LIS300AL Gyroscopes Handler
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
#include "gyro.h"


//=============================================================================
//
// Gyro Device Definitions
//
// The following defintions are based on the LISY300AL datasheet
//
//=============================================================================

#define GYRO_RANGE_DEG_PER_SEC      600.0     // Measurement range (deg/sec)
#define GYRO_SENS_V_PER_DEG_PER_SEC   0.0033  // Sensitivity (volts / deg/sec)
#define GYRO_ZERO_V                   1.65    // Zero-rate level (volts)
#define GYRO_BW_HZ                   85.0     // Bandwidth (Hz)

// Derived Definitions in Volts

#define GYRO_RANGE_V                (GYRO_RANGE_DEG_PER_SEC * GYRO_SENS_V_PER_DEG_PER_SEC)

// Derived Definitions in Radians

#define GYRO_RANGE_RAD_PER_SEC      (GYRO_RANGE_DEG_PER_SEC / DEG_IN_RAD)
#define GYRO_SENS_V_PER_RAD_PER_SEC (GYRO_SENS_V_PER_DEG_PER_SEC * DEG_IN_RAD)

// Derived Definitions in Numbers

#define GYRO_RANGE                  (GYRO_RANGE_V / ADC_SENS_V)
#define GYRO_SENS_PER_DEG_PER_SEC   (GYRO_SENS_V_PER_DEG_PER_SEC / ADC_SENS_V)
#define GYRO_SENS_PER_RAD_PER_SEC   (GYRO_SENS_PER_DEG_PER_SEC * DEG_IN_RAD)
#define GYRO_ZERO                   (GYRO_ZERO_V / ADC_SENS_V)

//-----------------------------------------------------------------------------
//
// Gyro rest state determination

#define GYRO_REST_AVG_DEV_MAX       1    // How much the gyro reading can
                                         // deviate from the long-term average
                                         // and still be considered stable.
#define GYRO_REST_ZERO_DEV_MAX    100    // How much the gyro reading can
                                         // deviate from zero and still be
                                         // zero if stable.
#define GYRO_LONG_AVG_FACTOR        6    // Determines the averaging period. For
                                         // 10 msec rate this is 2^6 * 10 which
                                         // is a little more than 0.5 sec.
#define GYRO_REST_MSEC_MIN       2000    // Time (in msec the) gyro must be
                                         // at rest to flag a stable condition
#define GYRO_REST_CYCLES_MIN     (GYRO_REST_MSEC_MIN / CONTROL_LOOP_CYCLE_MSEC)
                                         

//=============================================================================
//
// Static Variables
//
//=============================================================================

int   Gyro::eeprom_base_addr;    // Base address in EEPROM


//=================================== set_*() =================================
//

// None

//============================= Constructor ===================================

Gyro::Gyro(void)

{
  cycle_counter = 0;
  raw_zero = (uint16_t)(GYRO_ZERO * ADC_GAIN);
  raw = (uint16_t)(GYRO_ZERO * ADC_GAIN);
  raw_avg = (uint16_t)GYRO_ZERO << GYRO_LONG_AVG_FACTOR;
  last_unstable_cycle = 0;
  stable_flag = false;
  rad_per_sec = 0;
};


//================================= init() ====================================
//
// Initialize a gyro object

void
Gyro::init(uint8_t ain_in)

{
  ain = ain_in;
};


//=============================== update() ====================================
//
// Read the gyroscope's raw data from the h/w and perform all the required
// calculations.  Called periodically.

void
Gyro::update(void)

{
  int16_t avg_diff;
  int16_t zero_diff;


  // Read the gyro input.  Note the reading has a gain of ADC_GAIN over the
  // original range.
  
  raw = adc_get_data(ain);

  // Calculate long-term average, in units of (1 << GYRO_LONG_AVG_FACTOR)

  avg_diff = (int16_t)((raw >> ADC_GAIN_SHIFT) - (raw_avg >> GYRO_LONG_AVG_FACTOR));
  raw_avg += avg_diff;

  // Calculate the difference from the zero point, in units of ADC_GAIN
  
  zero_diff = (int16_t)(raw - raw_zero);
  
  // To be stable, the current reading must not deviate from average too much.
  // It also must not deviate from zero too much.

  if ((avg_diff > (int16_t)GYRO_REST_AVG_DEV_MAX)                 ||
      (avg_diff < (int16_t)-GYRO_REST_AVG_DEV_MAX)                ||
      (zero_diff > (int16_t)(GYRO_REST_ZERO_DEV_MAX * ADC_GAIN))  ||
      (zero_diff < (int16_t)(-GYRO_REST_ZERO_DEV_MAX * ADC_GAIN))
     )
  {
    // Not stable
    
    stable_flag = false;
    last_unstable_cycle = cycle_counter;
  }

  else if ((uint8_t)(cycle_counter - last_unstable_cycle) >= (uint8_t)GYRO_REST_CYCLES_MIN)
  {
    // We have been at rest for long enough, flag a stable condition

    stable_flag = true;
  }
  
  // Convert to rad/sec.
  
  rad_per_sec = (float)zero_diff / ((float)GYRO_SENS_PER_RAD_PER_SEC * (float)ADC_GAIN);

  cycle_counter++;
};


//================================= zero() ====================================
//
// Set the gyroscope's zero point to the current long-time average.

void Gyro::zero(void)

{
  raw_zero = raw_avg >> (GYRO_LONG_AVG_FACTOR - ADC_GAIN_SHIFT);
};


#if PRINT_GYRO
//============================= print_stats() =================================
//
// Print some statistics (for debug)

void
Gyro::print_stats(void)

{
   Serial.print(raw >> ADC_GAIN_SHIFT, DEC);
   Serial.print("\t");
   Serial.print(raw_avg >> GYRO_LONG_AVG_FACTOR, DEC);
   Serial.print("\t");
   Serial.print(raw_zero >> ADC_GAIN_SHIFT, DEC);
   Serial.print("\t");
   Serial.print(rad_per_sec);
   Serial.print("\t");
   Serial.print(stable_flag, BIN);
   Serial.print("\t");
};
#endif


//============================== read_eeprom() ==============================
//
// Read the configuration parameters from EEPROM, if valid.  If not, set
// defaults.

int                                           // Ret: Next address in EEPROM
Gyro::read_eeprom(int   eeprom_base_addr_in)  // In: Base address in EEPROM

{
  eeprom_base_addr = eeprom_base_addr_in;
  
  // Do nothing.  A float variable place is reserved for backward
  // compatibility  
  
  return eeprom_base_addr_in + sizeof(float);
};


//============================== write_eeprom() =============================
//
// Write the configuration parameters to EEPROM

void Gyro::write_eeprom(void)

{
  // Do nothing
};


