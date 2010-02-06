//=============================================================================
//
// ADC Module
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

//=============================================================================
//
// Private Definitions
//
//=============================================================================

#define NUM_ADC_CH 4


//=============================================================================
//
// Static Variables
//
//=============================================================================

static uint16_t adc_data[NUM_ADC_CH];
static uint16_t num_samples;


//=============================================================================
//
// Public Variables
//
//=============================================================================

uint8_t adc_cycles;


//=============================================================================
//
// Public Functions
//
//=============================================================================

//=========================== adc_init() ======================================
//
// Initialize the ADC module
//

void adc_init()

{
  DIDR0 = 0x3F & (0xFF << NUM_ADC_CH);    // Disable all unused channels

  ADCSRA = (0 << ADEN)  |                 // Disable ADC
           (0 << ADSC)  |                 // Don't start convertion
           (1 << ADATE) |                 // Auto trigger
           (1 << ADIF)  |                 // Clear ADC interrupt flag
           (0 << ADIE)  |                 // Disable ADC interrupt
           (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
                                          // Prescaler division = 128, ADC clock = 125KHz
                                          // Higher frequency would reduce accuracy

  ADCSRB = (1 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);
                                          // Auto-trigger on Timer/Counter 0 overflow.
                                          // With Arduino that's every 1024 uSec
 
  ADMUX = (0 << REFS1) | (0 << REFS0) |   // Voltage reference = AREF
          (0 << ADLAR) |                  // Right-adjust result
          0;                              // Analog input

  ADCSRA |= (1 << ADEN);                  // Enable ADC
  ADCSRA |= (1 << ADIE);                  // Enable ADC interrupt

  ADCSRA |= (1 << ADSC);                  // Start the first conversion
}


//=========================== adc_get_data() ==================================
//
// Get the data of a single ADC channel, with ADC_GAIN over the sampled value 
//

uint16_t                  // Ret: ADC data
adc_get_data(uint8_t ch)  // In : ADC channel

{
  uint16_t data;
  uint8_t  old_sreg;


  // Save the interrupt status and disable interrupts.
  // This is required to assure consistent reading of the data (that may change
  // during multi-byte reads)
  
  old_sreg = SREG;
  cli();

  data = adc_data[ch];

  // Restore the interrupt status
  
  SREG = old_sreg;

  return data;
}


//======================= adc_get_data_no_gain() ==============================
//
// Get the data of a single ADC channel, with no gain
//

uint16_t                           // Ret: ADC data
adc_get_data_no_gain(uint8_t ch)   // In : ADC channel

{
  uint16_t data;
  uint8_t  old_sreg;


  // Save the interrupt status and disable interrupts.
  // This is required to assure consistent reading of the data (that may change
  // during multi-byte reads)
  
  old_sreg = SREG;
  cli();

  data = adc_data[ch];

  // Restore the interrupt status
  
  SREG = old_sreg;

  return data >> ADC_SMOOTH_SHIFT;
}


//=========================== adc_print_stats() ===============================
//
// Print the ADC statistics
//

void adc_print_stats()

{
  uint16_t last_num_samples;
  uint8_t  old_sreg;


  // Save the interrupt status and disable interrupts.
  // This is required to assure consistent reading of the data (that may change
  // during multi-byte reads)
  
  old_sreg = SREG;
  cli();

  // Restore the interrupt status
  
  SREG = old_sreg;
}


//=============================================================================
//
// ADC Interrupt Handler
//

ISR(ADC_vect)

{
  static uint8_t  current_ch = 0;
  static uint8_t  next_ch    = 1;
  uint16_t        current_data;


  // Read the ADC registers, ADCL first then ADCH to guarantee consistency

  current_data = ADCL + (ADCH << 8);

  // Enable interrupts (from other sources) to avoid long latency 

  sei();

  // Set the mux for the next channel

  ADMUX = (0 << REFS1) | (0 << REFS0) |   // Voltage reference = AREF
          (0 << ADLAR) |                  // Right-adjust result
          next_ch;                        // Analog input

  // Add filtering here
  
  adc_data[current_ch] -= adc_data[current_ch] >> ADC_SMOOTH_SHIFT;
  adc_data[current_ch] += current_data;
  
  current_ch = next_ch;

  // Caluculate the next channel in the cyclic reading
  
  if (++next_ch >= NUM_ADC_CH)
    next_ch = 0;
  
  if (current_ch != 0)
  {
    // Start the next conversion

    ADCSRA |= (1 << ADSC);
  }
  
  else
    adc_cycles++;
}

