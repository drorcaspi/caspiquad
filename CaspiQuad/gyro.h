#ifndef __GYRO_H__
#define __GYRO_H__

//=============================================================================
//
// Gyroscope API
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


class Gyro

{
private:
  static int   eeprom_base_addr;    // Base address in EEPROM
  
  uint8_t      ain;                 // Arduino analog input number
  uint8_t      cycle_counter;       // Count the number of cycles
  uint16_t     raw;                 // Raw gyro reading (in units of ADC_GAIN)
  uint16_t     raw_zero;            // Raw gyro reading at zero G (in units of ADC_GAIN)
  uint16_t     raw_avg;             // Average of raw readings
  uint8_t      last_unstable_cycle; // Last cycle when the reading was too far
                                    // from the average and from zero
  boolean      stable_flag;         // Flags that the gyro reading is stable
  float        rad_per_sec;         // Gyro reading converted to rad/sec units

public:
  //============================= Constructor ===================================

  Gyro(void);
  
  //================================== init() ===================================
  //
  // Initialize a gyro object

  void init(uint8_t ain_in);
  
  //=============================== update() ====================================
  //
  // Read the gyroscope's raw data from the h/w and perform all the required
  // calculations.  Called periodically.

  void update(void);

  //================================= zero() ====================================
  //
  // Set the gyroscope's zero point to the current long-time average.

  void zero(void);

  //=================================== set_*() =================================
  //

  // Nothing
  
  //=================================== get_*() =================================
  //

  uint16_t         get_raw(void)          {return raw >> ADC_GAIN_SHIFT;};
  uint16_t         get_raw_zero(void)     {return raw_zero >> ADC_GAIN_SHIFT;};
  float            get_rad_per_sec(void)  {return rad_per_sec;};
  volatile boolean is_stable(void)        {return stable_flag;};

  //============================= print_stats() =================================
  //
  // Print some statistics (for debug)

  void print_stats(void);

  //============================== read_eeprom() ==============================
  //
  // Read the configuration parameters from EEPROM, if valid.  If not, set
  // defaults.

  static
  int                                      // Ret: Next address in EEPROM
  read_eeprom(int   eeprom_base_addr_in);  // In: Base address in EEPROM

  //============================== write_eeprom() =============================
  //
  // Write the configuration parameters to EEPROM

  static
  void
  write_eeprom(void);
};

#endif
