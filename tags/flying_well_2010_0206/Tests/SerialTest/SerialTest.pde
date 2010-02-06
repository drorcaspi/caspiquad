#include <stdlib.h>
#include <math.h>

#include "hardware_serial2.h"



//=============================================================================
//
// Global Definitions
//
//=============================================================================

uint16_t cycles = 0;

//=============================== setup() =====================================
//
// Arduino Initialization
//
//=============================================================================

void setup()
{
  Serial.begin(115200);

}


//=============================== loop() ======================================
//
// Arduino Main Loop Body
//
//=============================================================================

void loop()
{
  uint32_t time;
  
  
  time = micros();
  Serial.print("Printing 18 takes ");
  Serial.print(micros() - time);
  Serial.println(" usec");
  delay(50);
}
