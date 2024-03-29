//=============================================================================
//
// BMP085 Barometric Pressure Sensor Handler
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
  Copyright (c) 2010 Dror Caspi.  All rights reserved.

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

#if SUPPORT_BARO

#include "baro.h"


//=============================================================================
//
// Local Definitions
//
//=============================================================================

// Test flag.  If != 0, use example data from the BMP085 data sheet instead of
// real data.

#define BARO_EXAMPLE        0   

// Read temperature once (if != 0) or ongoing (if == 0)

#define BARO_READ_TEMP_ONCE 1

//-----------------------------------------------------------------------------
//
// BMP085 Air Pressure Sensor Definitions
//
//-----------------------------------------------------------------------------

#define BMP085_ADDRESS          0x77   // Device Addresss

#if BARO_EXAMPLE
  #define BMP085_OSS            0      // Over-sampling setting
#else
  #define BMP085_OSS            3      // Over-sampling setting
#endif

// Device Registers

typedef enum
{
  BMP085_AC1_REG              = 0xAA,
  BMP085_AC2_REG              = 0xAC,
  BMP085_AC3_REG              = 0xAE,
  BMP085_AC4_REG              = 0xB0,
  BMP085_AC5_REG              = 0xB2,
  BMP085_AC6_REG              = 0xB4,
  BMP085_B1_REG               = 0xB6,
  BMP085_B2_REG               = 0xB8,
  BMP085_MB_REG               = 0xBA,
  BMP085_MC_REG               = 0xBC,
  BMP085_MD_REG               = 0xBE,
  BMP085_CONTROL_REG          = 0xF4,
  BMP085_SENSOR_MSB_REG       = 0xF6,
  BMP085_SENSOR_LSB_REG       = 0xF7,
  BMP085_SENSOR_XLSB_REG      = 0xF8
} Bmp085RegAddresses;

typedef enum
{
  BMP085_TEMPERATURE = 0x2E,
  BMP085_PRESSURE_0  = 0x34,
  BMP085_PRESSURE_1  = 0x74,
  BMP085_PRESSURE_2  = 0xB4,
  BMP085_PRESSURE_3  = 0xF4
} Bmp085ControlValues;

typedef enum
{
  BMP085_AC1,
  BMP085_AC2,
  BMP085_AC3,
  BMP085_AC4,
  BMP085_AC5,
  BMP085_AC6,
  BMP085_B1,
  BMP085_B2,
  BMP085_MB,
  BMP085_MC,
  BMP085_MD,
  BMP085_EEPROM_NUM
} BmpEepromData;

typedef enum
{
  BARO_CYCLE_READ_TEMP      = 0,  // At least 4.5msec to sample
  BARO_CYCLE_WAIT_RESSURE_1 = 1,
  BARO_CYCLE_WAIT_RESSURE_2 = 2,
  BARO_CYCLE_READ_PRESSURE  = 3,  // At least 22.5msec to sample
  BARO_CYCLE_NUM            = 4
} BaroCycle;

#if (CONTROL_LOOP_CYCLE_MSEC != 10)
  #error The above definition of BaroCycle needs to be fixed
#endif

// Averaging definitions

#define BARO_PRESSURE_AVG_INIT   0x80000000
#define BARO_PRESSURE_AVG_SHIFT  5   // LPF @ ~ 3Hz 

#if (CONTROL_LOOP_CYCLE_MSEC != 10)
  #error The above definition of BARO_PRESSURE_AVG_SHIFT needs to be fixed
#endif


//=============================================================================
//
// Static Variables
//
//=============================================================================

static int16_t bmp085_eeprom[BMP085_EEPROM_NUM]
#if BARO_EXAMPLE
// Initialize with the example data in the BMP085 data sheet

= {408, -72, -14383, 32741, 32757, 23153, 6190, 4, -32768, -8711, 2868}
#endif
;

static int32_t pressure_pa;
static int32_t baro_pressure_avg   = BARO_PRESSURE_AVG_INIT;
static int32_t baro_pressure_zero;


//=============================================================================
//
// Public Functions
//
//=============================================================================

//=============================== baro_init() =================================
//
// Initialize the barometric sensor module
// Should be called on system initalization

boolean             // Ret: true if OK, false if failed
baro_init(void)

{
  uint8_t i;
  int16_t temp;
  boolean status;
  

#if (! BARO_EXAMPLE)
  // Serial.print("BMP085 EEPROM\t");
  
  // Read the EEPROM calibration data

  i2c_start_read(BMP085_ADDRESS, BMP085_AC1_REG, BMP085_EEPROM_NUM * 2);

  for (i = 0; i < BMP085_EEPROM_NUM; i++)
  {
    temp = i2c_read_next_16(&status);
    if (! status)
      return false;
    
#if PRINT_BARO
    Serial.print(temp, DEC);
    Serial.print('\t');
#endif

    if ((temp == 0) || (temp == 0xFFFF))
    {
      // This is an error, must not happen
      
      return false;
    };
    
    bmp085_eeprom[i] = temp;
  };
  
#if PRINT_BARO
  Serial.println();
#endif

  // Initiate 1st temperature reading
  
  i2c_write_8(BMP085_ADDRESS, BMP085_CONTROL_REG, BMP085_TEMPERATURE);

#endif  // ! BARO_EXAMPLE

  return true;
};


//=============================== baro_update() ===============================
//
// Update the barometric sensor readings from the h/w

boolean             // Ret: true if OK, false if failed
baro_update(void)

{
  // Variable names and types are per the algorithm described in the BMP085
  // data sheet
  
  static uint8_t baro_cycle   = BARO_CYCLE_READ_TEMP;
  static int16_t b6;
  int16_t        temperature_01c;
  uint16_t       ut;
  uint32_t       up;
  int32_t        b3;
  uint32_t       b4;
  int16_t        b5;
  uint32_t       b7;
  int16_t        x1s;
  int16_t        x2s;
  int32_t        x1;
  int32_t        x2;
  int32_t        x3;
  boolean        status;

  
  //Serial.print(baro_cycle, DEC);
  //Serial.print('\t');
  
  switch (baro_cycle)
  {
    case BARO_CYCLE_READ_TEMP:
      // Read uncompensated temperature value (16 bits)
      
#if BARO_EXAMPLE
      ut = 27898;
#else
      ut = i2c_read_16(BMP085_ADDRESS, BMP085_SENSOR_MSB_REG, &status);
      if (! status)
        return false;
#endif

      // Calculate true temperature
      
      x1s = ((uint32_t)(ut - bmp085_eeprom[BMP085_AC6]) *
             (uint16_t)(bmp085_eeprom[BMP085_AC5])) >> 15;
      x2s = ((int32_t)bmp085_eeprom[BMP085_MC] << 11) / (x1s + bmp085_eeprom[BMP085_MD]);
      //Serial.print(x1s, DEC);
      //Serial.print('\t');
      //Serial.print(x2s, DEC);
      //Serial.print('\t');
      b5 = x1s + x2s;
      b6 = b5 - 4000;   // Note b6 is static since it's used in a later invokation
      
#if PRINT_BARO
      temperature_01c = (b5 + 8) >> 4;
      Serial.print(temperature_01c, DEC);
      Serial.print('\t');
#endif
      
      // Initiate uncompensated pressure value read

#if (! BARO_EXAMPLE)
      i2c_write_8(BMP085_ADDRESS, BMP085_CONTROL_REG, BMP085_PRESSURE_3);
#endif

      break;

    case BARO_CYCLE_READ_PRESSURE:

      // Read uncompensated pressure value (19 bits)
      
#if BARO_EXAMPLE
      up = 23843;
#else
      up = i2c_read_24(BMP085_ADDRESS, BMP085_SENSOR_MSB_REG, &status);
      if (! status)
        return false;
      
      up >>= (8 - BMP085_OSS);

      //Serial.print(up, DEC);
      //Serial.print('\t');

      // Initiate next temperature reading

#if BARO_READ_TEMP_ONCE      
      i2c_write_8(BMP085_ADDRESS, BMP085_CONTROL_REG, BMP085_PRESSURE_3);
#else
      i2c_write_8(BMP085_ADDRESS, BMP085_CONTROL_REG, BMP085_TEMPERATURE);
#endif
#endif  // ! BARO_EXAMPLE

      // Calculate true pressure

      //Serial.print(b6, DEC);
      //Serial.print('\t');
      x1 = ((int32_t)(bmp085_eeprom[BMP085_B2]) * (((int32_t)b6 * (int32_t)b6) >> 12)) >> 11;
      x2 = ((int32_t)(bmp085_eeprom[BMP085_AC2]) * (int32_t)b6) >> 11;
      x3 = x1 + x2;
      b3 = (((((int32_t)(bmp085_eeprom[BMP085_AC1]) << 2) + x3) << BMP085_OSS) + 2) >> 2;
      //Serial.print(b3, DEC);
      //Serial.print('\t');
      x1 = ((int32_t)(bmp085_eeprom[BMP085_AC3]) * (int32_t)b6) >> 13;
      x2 = ((int32_t)(bmp085_eeprom[BMP085_B1]) * (((int32_t)b6 * (int32_t)b6) >> 12)) >> 16;
      x3 = ((x1 + x2) + 2) >> 2;
      //Serial.print(x1, DEC);
      //Serial.print('\t');
      //Serial.print(x2, DEC);
      //Serial.print('\t');
      //Serial.print(x3, DEC);
      //Serial.print('\t');
      b4 = ((uint16_t)(bmp085_eeprom[BMP085_AC4]) * (uint32_t)(x3 + 32768)) >> 15;
      //Serial.print(b4, DEC);
      //Serial.print('\t');
      b7 = (up - (uint32_t)b3) * (50000 >> BMP085_OSS);
      //Serial.print(b7, HEX);
      //Serial.print('\t');
      if (b7 < (uint32_t)0x80000000)
        pressure_pa = (b7 << 1) / b4;
      else
        pressure_pa = (b7 / b4) << 1;
      //Serial.print(pressure_pa, DEC);
      //Serial.print('\t');
      x1 = (pressure_pa >> 8) * (pressure_pa >> 8);
      //Serial.print(x1, DEC);
      //Serial.print('\t');
      x1 = (x1 * 3038) >> 16;
      x2 = (-7357 * pressure_pa) >> 16;
      //Serial.print(x1, DEC);
      //Serial.print('\t');
      //Serial.print(x2, DEC);
      //Serial.print('\t');
      pressure_pa += (x1 + x2 + 3791) >> 4;
      
#if PRINT_BARO
      Serial.print(pressure_pa, DEC);
      Serial.print('\t');
#endif

      break;

    default:
      break;
  };

  if (++baro_cycle >= BARO_CYCLE_NUM)
  {
#if BARO_READ_TEMP_ONCE      
    baro_cycle = BARO_CYCLE_WAIT_RESSURE_1;
#else
    baro_cycle = BARO_CYCLE_READ_TEMP;
#endif
  }

  return true;
}


//========================= baro_alt_estimate_zero() ==========================
//
// Zero the barometric sensor altitude estimate to the current average

void
baro_alt_estimate_zero(void)

{
  baro_pressure_zero = baro_pressure_avg;
}


//========================= baro_alt_estimate_get() ===========================
//
// Get the barometric sensor altitude estimate

int32_t                   // Ret: Altitude diff. from the zero point, in cm
baro_alt_estimate_get(void)

{
  int32_t        altitude_cm;
  
  
  // Do a simple IIR averaging of the barometric pressue, and calculate the
  // derivative at the same time
  
  if (baro_pressure_avg == BARO_PRESSURE_AVG_INIT)
  {
     baro_pressure_avg = pressure_pa << BARO_PRESSURE_AVG_SHIFT;
  }
     
  else
  {
    // Calculate the IIR average of the barometric pressure
    
    baro_pressure_avg += pressure_pa - (baro_pressure_avg >> BARO_PRESSURE_AVG_SHIFT);
  }
  
  // Calculate the altitude in units of 1 cm, roughly 1/8 Pa (should be 8.43/8 cm,
  // but this approximation is good enough)
  
  altitude_cm = (baro_pressure_zero - baro_pressure_avg) >> (BARO_PRESSURE_AVG_SHIFT - 3);

#if PRINT_BARO
  Serial.println(altitude_cm, DEC);
#endif

  return altitude_cm;
}

#endif // SUPPORT_BARO

