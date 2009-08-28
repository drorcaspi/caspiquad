#include <stdlib.h>
#include <math.h>

#include "quad.h"


//=============================== setup() =====================================
//
// Arduino Initialization
//
//=============================================================================

void setup()
{
  analogReference(ANALOG_REFERENCE);
  Serial.begin(115200);

  pinMode(BAT_SENSOR_PIN, INPUT);
}


//=============================== loop() ======================================
//
// Arduino Main Loop Body
//
//=============================================================================

void loop()
{
  delay(1000);  // msec
  
  Serial.print("Battery: ");
  Serial.println(analogRead(BAT_SENSOR_PIN), DEC);
}
