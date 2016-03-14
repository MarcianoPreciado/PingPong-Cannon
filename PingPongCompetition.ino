/*
  Created by: Marciano C. Preciado
  Latest Update: 03-05-16

  This libary was created to facilitate the necessary calculations
  to operate the ME 1010 ping-pong cannon.

   _PINS_    _HARDWARE_        _VARIABLE_NAME_
  1  RX
  2  TX
  3  *~
  4         Motor Direction     MotorDirectionPin
  5  *~     Motor Power         MotorPowerPin
  6  *~     Solenoid Power      SolenoidPowerPin
  7         Solenoid Direction  SolenoidDirectionPin
  8
  9  *~     Aiming Servo        cannonServo     (Servo Instance)
  10 *~     Loader Servo        loaderServo     (Servo Instance)
  11 *~     Right Bumper        bumperR
  12        Left Bumper         bumperL
  13

  A0
  A1
  A2
  A3
  A4
  A5        IR Sensor           IR
  A6
  A7        Built-In Buttons    btn

*/


#include <Servo.h>
#include "PingPong.h"

Servo cannonServo;
Servo loaderServo;

void setup() {
  Serial.begin(9600);
  cannonServo.attach(9);
  loaderServo.attach(10);

}

void loop() {
  
  delay(10);
}
