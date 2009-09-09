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
// Get the data of a single ADC channel
//

uint16_t                   // Ret: ADC data
adc_get_data(uint8_t ch);  // In : ADC channel


//=========================== adc_print_stats() ===============================
//
// Print the ADC statistics
//

void adc_print_stats(void);

#endif
