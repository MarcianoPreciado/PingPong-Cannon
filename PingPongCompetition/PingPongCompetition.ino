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

#define L1 0.1313
#define L2 0.0475
#define L3 0.0880
#define L4 0.0960
// d_vector:
#define d1  0.031
#define d2  0.19
#define d3  0.067
// offsets:
#define thetaS0  5.9
#define thetaL0  14.0
// initial ball velocity:
#define v0  3.761904022909760

Cannon Wallace(L1, L2, L3, L4, d1, d2, d3, thetaS0, thetaL0, v0);
Servo cannonServo;
Servo loaderServo;

void setup() {
  Serial.begin(9600);
  cannonServo.attach(9);
  loaderServo.attach(10);

  pinMode(13, OUTPUT);
  pinMode(MotorDirectionPin,OUTPUT);
  pinMode(SolenoidDirectionPin,OUTPUT);
  pinMode(bumperL,OUTPUT);
  pinMode(bumperR,OUTPUT);
}

double targetX;
double targetZ;
int angleLowerBound = 33;
int angleUpperBound = 83;
double thetaS;

void loop() {
  digitalWrite(13, HIGH);
  if ( //we are given a target ){
    thetaS = getServoAngle(angleLowerBound, angleUpperBound, targetX);
    cannonServo.write(thetaS);

    Wallace.moveTo(targetZ);

    analogWrite(SolenoidPowerPin, fullPower);
    delay(onTime);
    analogWrite(SolenoidPowerPin, 0);

    Wallace.reload();
}

delay(10);
}
