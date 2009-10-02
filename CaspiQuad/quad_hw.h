#ifndef __QUAD_HW_H__
#define __QUAD_HW_H__

/*-----------------------------------------------------------------------------
  CaspiQuad 1
  Copyright (c) 2009 Dror Caspi.  All rights reserved.
  An Open Source Arduino based quadrocopter.

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
// Quad Hardware Definitions
//
//=============================================================================

//-----------------------------------------------------------------------------
// Arduino Pins Usage
//-----------------------------------------------------------------------------

// Analog Sensor Pins

#define PITCH_RATE_PIN   2   // Pitch gyro
#define ROLL_RATE_PIN    1   // Roll gyro
#define YAW_RATE_PIN     0   // Yaw gyro

#define BAT_SENSOR_PIN   3   // Battery sensor

// Motor Control Pins

#define FRONT_MOTOR_PIN 10   // OC1B
#define REAR_MOTOR_PIN   9   // OC1A
#define RIGHT_MOTOR_PIN  3   // OC2B
#define LEFT_MOTOR_PIN  11   // OC2A

// Receiver Channel Pins

#define THROTTLE_CH_PIN  4
#define ROLL_CH_PIN      2   // Aileron
#define PITCH_CH_PIN     5   // Elevator
#define YAW_CH_PIN       6   // Rudder
#define GEAR_CH_PIN      7
#define AUX1_CH_PIN      8

// Other Pins

#define LED_PIN         13
#define BUZZER_PIN      12



//-----------------------------------------------------------------------------
// Analog-to-Digital
//-----------------------------------------------------------------------------

//#define ANALOG_REFERENCE DEFAULT
//#define ANALOG_REFERENCE INTERNAL
#define ANALOG_REFERENCE EXTERNAL

#if (ANALOG_REFERENCE == DEFAULT)
  #error default analog reference not supported  // #define ANALOG_REFERENCE_V  5.0
#elif (ANALOG_REFERENCE == INTERNAL)
  #error internal analog reference not supported // #define ANALOG_REFERENCE_V  1.1
#elif (ANALOG_REFERENCE == EXTERNAL)
  #define ANALOG_REFERENCE_V  3.25
#else
  #error Illegal ANALOG_REFERENCE
#endif

#define ADC_RANGE      1024         // 10 bits
#define ADC_RANGE_V       3.25      // ANALOG_REFERENCE_V         // Range in Volts
#define ADC_SENS_V        0.003174  // (ADC_RANGE_V / ADC_RANGE)  // Sensitivy (volts / digit)


//-----------------------------------------------------------------------------
// Other Hardware
//-----------------------------------------------------------------------------

#define ACCELL_MODEL_LIS302DL   // Accelerometer device model (3-axis, I2C)
#define GYRO_MODEL_LISY300AL    // Gyroscope device model (1-axis, analog)

#endif
