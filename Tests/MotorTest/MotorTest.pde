#include "quad.h"
#include "motors.h"


//=============================================================================
//
// Global Definitions
//
//=============================================================================


//=============================================================================
//
// Global Variables
//
//=============================================================================


//=============================== setup() =====================================
//
// Arduino Initialization
//
//=============================================================================

void setup()
{
  uint8_t  dir;
  char     c;
  uint8_t  throttle = MOTOR_THROTTLE_MIN;

  
  Serial.begin(115200);

  Serial.println("ESC Throttle Range Setup");
  Serial.println("Setting Throttle High");

  analogWrite(FRONT_MOTOR_PIN, MOTOR_THROTTLE_MIN);
  analogWrite(REAR_MOTOR_PIN,  MOTOR_THROTTLE_MAX);
  analogWrite(RIGHT_MOTOR_PIN, MOTOR_THROTTLE_MAX);
  analogWrite(LEFT_MOTOR_PIN,  MOTOR_THROTTLE_MAX);

  Serial.print("Connect battery to ESC, wait ~2sec until \"beep-beep\" > ");
  while (! Serial.available());
  c = Serial.read();

  Serial.println("Setting Throttle Low");

  analogWrite(FRONT_MOTOR_PIN, MOTOR_THROTTLE_MIN);
  analogWrite(REAR_MOTOR_PIN,  MOTOR_THROTTLE_MIN);
  analogWrite(RIGHT_MOTOR_PIN, MOTOR_THROTTLE_MIN);
  analogWrite(LEFT_MOTOR_PIN,  MOTOR_THROTTLE_MIN);

  Serial.print("Wait for 3 \"beeps\", then long \"beep\" > ");
  while (! Serial.available());
  c = Serial.read();

  Serial.println("Enter + to increase throttle, - to decrease throttle");
  while (true)
  {
    while (! Serial.available());
    c = Serial.read();
    if ((c == '+') && (throttle <= (MOTOR_THROTTLE_MIN - 10)))
      throttle += 10;
    else if ((c == '-') && (throttle >= (MOTOR_THROTTLE_MIN + 10)))
      throttle -= 10;
    Serial.println((int16_t)throttle, DEC);

    analogWrite(FRONT_MOTOR_PIN, throttle);
    analogWrite(REAR_MOTOR_PIN,  throttle);
    analogWrite(RIGHT_MOTOR_PIN, throttle);
    analogWrite(LEFT_MOTOR_PIN,  throttle);
  };
}


//=============================== loop() ======================================
//
// Arduino Main Loop Body
//
//=============================================================================

void loop ()
{
}
