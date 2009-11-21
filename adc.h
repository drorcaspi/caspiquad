#ifndef __ADC_H__
#define __ADC_H__

//=============================================================================
//
// ADC Module Public API
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

#define ADC_CYCLE_USEC   1024   // ADC sampling cycle

#define ADC_SMOOTH_SHIFT    6   // Shift factor used in smooting the ADC data
#define ADC_GAIN_SHIFT      ADC_SMOOTH_SHIFT
#define ADC_GAIN            (1 << ADC_GAIN_SHIFT)


//=============================================================================
//
// Public Variables
//
//=============================================================================

extern uint8_t adc_cycles;


//=============================================================================
//
// Public Functions
//
//=============================================================================

//=========================== adc_init() ======================================
//
// Initialize the ADC module
//

void adc_init(void);


//=========================== adc_get_data() ==================================
//
// Get the data of a single ADC channel, with ADC_GAIN over the sampled value 
//

uint16_t                   // Ret: ADC data
adc_get_data(uint8_t ch);  // In : ADC channel


//======================= adc_get_data_no_gain() ==============================
//
// Get the data of a single ADC channel, with no gain
//

uint16_t                           // Ret: ADC data
adc_get_data_no_gain(uint8_t ch);  // In : ADC channel


//=========================== adc_print_stats() ===============================
//
// Print the ADC statistics
//

void adc_print_stats(void);

#endif
