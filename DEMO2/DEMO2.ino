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
  9  *~     Aiming Servo        cannonServo         (Servo Instance)
  10 *~     Loader Servo        loaderServo         (Servo Instance)
  11 *~     Right Bumper        bumperR
  12        Left Bumper         bumperL
  13        IR LED              LED

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


double targetX;
double targetZ;
double thetaL;
double thetaS;
byte junk;
int angleLowerBound = 33.0;
int angleUpperBound = 83.0;
int i = 0;
int j = 0;
double targetA1[6][2] = {{0.8, 30},      //O   O
                         {0.9, 25},      //  O
                         {1.0, 20},
                         {0.95, 15},
                         {1.2, 10},
                         {0.7, 5}
};
double targetA2[6][2] = {{0.75, 3},     //O
  {0.85, 18},     //  O
  {1.1, 34},      //    O
  {1.05, 3},
  {1.15, 18},
  {0.8, 34}
};

void setup() {
  Serial.begin(9600);
  cannonServo.attach(9);
  loaderServo.attach(10);
  cannonServo.write(35);
  loaderServo.write(0);
  pinMode(13, OUTPUT);
  pinMode(MotorDirectionPin, OUTPUT);
  pinMode(SolenoidDirectionPin, OUTPUT);
  pinMode(bumperL, INPUT_PULLUP);
  pinMode(bumperR, INPUT_PULLUP);
  
  digitalWrite(MotorDirectionPin, LEFT);
  analogWrite(MotorPowerPin, 255);
  while(digitalRead(bumperL) != HIGH);
  delay(5);
  digitalWrite(MotorDirectionPin, !LEFT);
  analogWrite(MotorPowerPin, fullPower);
  delay(50);
  analogWrite(MotorPowerPin, 0);
}
double val;
int pos;
void loop() {

  if (Serial.available() > 0) {
  

    
    targetX = Serial.read();
    Serial.println(targetX);
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    Serial.println("enter for loop");
    pos = 0;
    for (i = 0; i < 6; i++) {

      val = targetA1[i][0];
      Serial.print("X: ");
      Serial.print(val);
      thetaL = Wallace.getServoAngle(33.0, 88.0, val);
      thetaS = Wallace.servoAngle(thetaL);
      

      val = targetA1[i][1];
      Serial.print(" Z: ");
      Serial.println(val);
      Serial.print("ThetaL: ");
      Serial.print(thetaL);
      Serial.print(" ThetaS: ");
      Serial.println(thetaS);
      Wallace.moveTo((int)targetA1[i][1], pos);
      cannonServo.write(thetaS);
      delay(450);
      analogWrite(SolenoidPowerPin, fullPower);
      delay(onTime);
      analogWrite(SolenoidPowerPin, 0);
      Wallace.reload(cannonServo, loaderServo);
      pos = 38;
    }

    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);
  }
  delay(10);
}
